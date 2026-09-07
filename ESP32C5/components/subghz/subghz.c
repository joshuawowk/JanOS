/* SubGHz orchestration: esp_console command handlers + streaming tasks that
 * drive the CC1101 and emit the byte-exact [SUBGHZ_*] tokens the M5MonsterC5-
 * Tab5 app parses. Console output is via printf()+fflush (the UART0 the Tab5
 * reads); ESP_LOG* is compiled out in this firmware, so it is never used here.
 *
 * A single radio operation runs at a time (the Tab5 uses one tab/screen at a
 * time and always issues subghz_stop before switching). Each streaming command
 * spawns one FreeRTOS task guarded by s_op_stop so subghz_stop can end it while
 * the REPL stays responsive.
 */
#include "subghz.h"
#include "subghz_priv.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_console.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "nvs.h"

/* ---- shared radio state ---- */
cc1101_t g_subghz_radio;
bool     g_subghz_radio_ok = false;
float    g_subghz_freq = SUBGHZ_DEFAULT_FREQ;
float    g_subghz_correction = 0.0f;

static bool s_radio_init_attempted = false;
static bool s_enabled = true;   /* radio arbiter: false => yield the shared header to the nRF24 */

#define NVS_NS   "subghz"
#define NVS_CORR "corr"   /* i32 hundredths of MHz */

/* ---- op management ---- */
typedef enum { OP_NONE, OP_LISTEN, OP_LISTEN_RAW, OP_JAM, OP_ANALYZER,
               OP_HUNT, OP_SCANNER, OP_WEATHER, OP_SPECTRUM, OP_BRUTE, OP_JAMDET } op_t;
static volatile op_t  s_op = OP_NONE;
static volatile bool  s_op_stop = false;
static volatile TaskHandle_t s_op_task = NULL;
static portMUX_TYPE   s_op_mux = portMUX_INITIALIZER_UNLOCKED;

/* op parameters */
static int      s_rssi_floor = -80;
static uint32_t s_scan_dwell = 80;
static uint32_t s_scan_edges = 4;
static uint32_t s_hunt_timeout = 2000;

/* common ISM candidate frequencies (MHz) */
static const float SCAN_FREQS[] = { 315.00f, 390.00f, 433.92f, 434.42f, 868.35f, 915.00f };
#define SCAN_FREQS_N (int)(sizeof(SCAN_FREQS)/sizeof(SCAN_FREQS[0]))

float subghz_effective_freq(void) { return g_subghz_freq + g_subghz_correction; }

static void radio_tune(float mhz) { cc1101_set_frequency(&g_subghz_radio, mhz + g_subghz_correction); }

static void radio_rx_ook(float mhz) {
    cc1101_set_idle(&g_subghz_radio);
    cc1101_set_modulation(&g_subghz_radio, CC1101_MOD_ASK);
    cc1101_set_ccmode(&g_subghz_radio, false);   /* async serial (raw) */
    /* set_ccmode() writes MDMCFG4 with a hardcoded ~116 kHz RX bandwidth, which
     * clobbers the wider config_base value. Restore 232 kHz so SAW-drifted 315/433
     * remotes (often off by >100 kHz) still land inside the receive channel. */
    cc1101_set_rxbw(&g_subghz_radio, 232.0f);
    radio_tune(mhz);
    cc1101_flush_rx(&g_subghz_radio);
    cc1101_set_rx(&g_subghz_radio);
}

bool subghz_ensure_radio(void) {
    if (!s_enabled) {
        /* The arbiter handed the shared SPI2 CS=GPIO3 / GDO0=GPIO4 header to the
         * nRF24 jammer. Do not touch the bus or GPIO4; report the CC1101 absent
         * so subghz_status prints radio=none. */
        s_radio_init_attempted = true;
        g_subghz_radio_ok = false;
        return false;
    }
    if (!s_radio_init_attempted) {
        cc1101_default_config(&g_subghz_radio);
        g_subghz_radio.mhz = subghz_effective_freq();
        g_subghz_radio_ok = cc1101_init(&g_subghz_radio);
        s_radio_init_attempted = true;
    } else if (!g_subghz_radio_ok) {
        g_subghz_radio_ok = cc1101_init(&g_subghz_radio);
    }
    if (g_subghz_radio_ok) printf("CC1101 initialized\n");
    else                   printf("CC1101 NOT DETECTED\n");
    fflush(stdout);
    return g_subghz_radio_ok;
}

void subghz_set_enabled(bool enabled) {
    s_enabled = enabled;
    if (!enabled) {
        /* Forget any prior CC1101 bring-up so a later re-enable re-probes
         * cleanly. cc1101_init() removes its SPI device on absence. */
        s_radio_init_attempted = false;
        g_subghz_radio_ok = false;
    }
}

bool subghz_detect(void) {
    if (!s_enabled) return false;
    /* Force a fresh probe: cc1101_init() adds an SPI2 device on CS=GPIO3, reads
     * VERSION, and (if absent) removes the device again, leaving GPIO4 a plain
     * input -- safe for the nRF24 backend to take over. */
    s_radio_init_attempted = false;
    return subghz_ensure_radio();
}

/* Stop any running op and wait for its task to exit. Safe to call from a
 * command handler (REPL task). */
static void subghz_stop_all(void) {
    if (s_op == OP_NONE && s_op_task == NULL) return;
    s_op_stop = true;
    for (int i = 0; i < 60 && s_op_task != NULL; i++) vTaskDelay(pdMS_TO_TICKS(50));
    /* Claim deletion ownership atomically vs the worker's own worker_exit(). */
    taskENTER_CRITICAL(&s_op_mux);
    TaskHandle_t h = s_op_task;
    s_op_task = NULL;
    taskEXIT_CRITICAL(&s_op_mux);
    if (h != NULL) {                   /* worker hung; we own the delete */
        vTaskDelete(h);
        subghz_cap_rx_stop();
        if (g_subghz_radio_ok) cc1101_set_idle(&g_subghz_radio);
    }
    s_op = OP_NONE;
    s_op_stop = false;
}

/* Worker task tail: exactly one side ever deletes a given TCB. If the worker
 * wins the critical section it self-deletes; if subghz_stop_all already claimed
 * ownership (s_op_task == NULL) the worker suspends and stop_all deletes it. */
static void worker_exit(void) {
    taskENTER_CRITICAL(&s_op_mux);
    bool i_own = (s_op_task != NULL);
    if (i_own) { s_op = OP_NONE; s_op_task = NULL; }
    taskEXIT_CRITICAL(&s_op_mux);
    if (i_own) vTaskDelete(NULL);
    else       vTaskSuspend(NULL);   /* stop_all owns and will delete us */
}

/* ---- listen / hunter shared: emit a decoded or raw capture ---- */
static char    s_last_serial[32] = {0};
static int     s_last_btn = -1;
static int64_t s_last_us = 0;
static int     s_last_idx = 0;

static void emit_capture(const uint16_t *timings, int edges, float freq, bool raw_mode) {
    subghz_signal_t sig;
    memset(&sig, 0, sizeof(sig));
    sig.freq = freq;
    sig.edges = edges;
    if (edges > SUBGHZ_MAX_EDGES) edges = SUBGHZ_MAX_EDGES;
    memcpy(sig.timings, timings, edges * sizeof(uint16_t));
    sig.edges = edges;

    if (raw_mode) {
        snprintf(sig.type, sizeof(sig.type), "RAW");
        snprintf(sig.proto, sizeof(sig.proto), "RAW");
        sig.te = 0;
        int idx = subghz_store_add(&sig);
        printf("[SUBGHZ_RAW] idx=%d freq=%.2f edges=%d te=%d\n", idx, freq, edges, sig.te);
        fflush(stdout);
        return;
    }

    if (!subghz_decode_ook(timings, edges, &sig)) return;   /* not a framed signal */

    int64_t now = esp_timer_get_time();
    bool dup = (s_last_idx > 0 && s_last_btn == sig.btn &&
                strncmp(s_last_serial, sig.serial, sizeof(s_last_serial)) == 0 &&
                (now - s_last_us) < 1500000);
    if (dup) {
        subghz_signal_t *st = subghz_store_get(s_last_idx);
        int cnt = st ? (st->cnt + 1) : 2;
        if (st) st->cnt = cnt;
        s_last_us = now;
        printf("[SUBGHZ_RX_DUP] idx=%d type=%s serial=%s btn=%d cnt=%d\n",
               s_last_idx, sig.type, sig.serial, sig.btn, cnt);
        fflush(stdout);
        return;
    }
    sig.cnt = 1;
    int idx = subghz_store_add(&sig);
    s_last_idx = idx;
    s_last_btn = sig.btn;
    s_last_us = now;
    snprintf(s_last_serial, sizeof(s_last_serial), "%s", sig.serial);
    printf("[SUBGHZ_RX] idx=%d type=%s freq=%.2f bits=%d serial=%s btn=%d proto=%s learn=%s mf=%s cnt=%d te=%d name=%s\n",
           idx, sig.type[0] ? sig.type : "Unknown", freq, sig.bits,
           sig.serial[0] ? sig.serial : "-", sig.btn,
           sig.proto[0] ? sig.proto : "-", sig.learn[0] ? sig.learn : "-",
           sig.mf[0] ? sig.mf : "-", sig.cnt, sig.te,
           sig.name[0] ? sig.name : "-");
    fflush(stdout);
}

/* ---- tasks ---- */
static uint16_t s_cap_buf[SUBGHZ_MAX_EDGES];

static void listen_task(void *pv) {
    bool raw = (s_op == OP_LISTEN_RAW);
    float freq = subghz_effective_freq();
    radio_rx_ook(freq);
    if (subghz_cap_rx_start(g_subghz_radio.gdo0_pin) != 0) {
        printf("SubGHz receive start failed: rmt\n"); fflush(stdout);
        goto done;
    }
    s_last_idx = 0; s_last_btn = -1; s_last_serial[0] = 0;
    int64_t next_rssi = 0;
    while (!s_op_stop) {
        int n = subghz_cap_rx_get(s_cap_buf, SUBGHZ_MAX_EDGES, 16, 120);
        if (n > 0) {
            int rssi = cc1101_get_rssi(&g_subghz_radio);
            if (rssi >= s_rssi_floor) emit_capture(s_cap_buf, n, freq, raw);
        }
        int64_t now = esp_timer_get_time();
        if (now >= next_rssi) {
            printf("[SUBGHZ_RSSI] %d\n", cc1101_get_rssi(&g_subghz_radio));
            fflush(stdout);
            next_rssi = now + 150000;   /* ~6.6 Hz */
        }
    }
done:
    subghz_cap_rx_stop();
    if (g_subghz_radio_ok) cc1101_set_idle(&g_subghz_radio);
    worker_exit();
}

static void jammer_task(void *pv) {
    float freq = subghz_effective_freq();
    cc1101_set_idle(&g_subghz_radio);
    cc1101_set_modulation(&g_subghz_radio, CC1101_MOD_ASK);
    cc1101_set_ccmode(&g_subghz_radio, false);
    radio_tune(freq);
    cc1101_set_pa(&g_subghz_radio, 12);
    /* Hold GDO0 high so the async OOK modulator keeps the carrier on. */
    gpio_set_direction(g_subghz_radio.gdo0_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(g_subghz_radio.gdo0_pin, 1);
    cc1101_set_tx(&g_subghz_radio);
    printf("SubGHz jammer ACTIVE\n"); fflush(stdout);
    while (!s_op_stop) vTaskDelay(pdMS_TO_TICKS(100));
    cc1101_set_idle(&g_subghz_radio);
    gpio_set_level(g_subghz_radio.gdo0_pin, 0);
    gpio_set_direction(g_subghz_radio.gdo0_pin, GPIO_MODE_INPUT);
    worker_exit();
}

static void scanner_task(void *pv) {
    while (!s_op_stop) {
        for (int i = 0; i < SCAN_FREQS_N && !s_op_stop; i++) {
            radio_rx_ook(SCAN_FREQS[i]);
            vTaskDelay(pdMS_TO_TICKS(2));
            unsigned edges = subghz_cap_edge_count(g_subghz_radio.gdo0_pin, s_scan_dwell);
            int rssi = cc1101_get_rssi(&g_subghz_radio);
            if (edges >= s_scan_edges && rssi >= s_rssi_floor) {
                printf("[SUBGHZ_SCAN_HIT] freq=%.2f edges=%u rssi=%d\n",
                       SCAN_FREQS[i], edges, rssi);
                fflush(stdout);
            }
        }
        printf("[SUBGHZ_SCAN_PASS]\n"); fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    if (g_subghz_radio_ok) cc1101_set_idle(&g_subghz_radio);
    worker_exit();
}

static void analyzer_task(void *pv) {
    bool hunt = (s_op == OP_HUNT);
    printf("[SUBGHZ_FA_START]\n"); fflush(stdout);
    s_last_idx = 0; s_last_serial[0] = 0; s_last_btn = -1;
    while (!s_op_stop) {
        /* Coarse RSSI sweep to find the strongest candidate. */
        float best_f = 0; int best_r = -200;
        for (int i = 0; i < SCAN_FREQS_N && !s_op_stop; i++) {
            radio_rx_ook(SCAN_FREQS[i]);
            vTaskDelay(pdMS_TO_TICKS(4));
            int r = cc1101_get_rssi(&g_subghz_radio);
            if (r > best_r) { best_r = r; best_f = SCAN_FREQS[i]; }
        }
        if (best_r >= s_rssi_floor && best_f > 0) {
            printf("[SUBGHZ_FA] freq=%.2f rssi=%d stage=lock\n", best_f, best_r);
            fflush(stdout);
            g_subghz_freq = best_f;
            if (hunt) {
                radio_rx_ook(best_f);
                if (subghz_cap_rx_start(g_subghz_radio.gdo0_pin) == 0) {
                    int n = subghz_cap_rx_get(s_cap_buf, SUBGHZ_MAX_EDGES, 24, s_hunt_timeout);
                    if (n > 0) {
                        subghz_signal_t probe;
                        memset(&probe, 0, sizeof(probe));
                        bool ok = subghz_decode_ook(s_cap_buf, n, &probe);
                        bool dup = ok && s_last_serial[0] &&
                                   strncmp(s_last_serial, probe.serial, sizeof(s_last_serial)) == 0;
                        if (dup) { printf("[SUBGHZ_FA] hunt duplicate\n"); }
                        else {
                            emit_capture(s_cap_buf, n, best_f, false);
                            if (ok) snprintf(s_last_serial, sizeof(s_last_serial), "%s", probe.serial);
                            printf("[SUBGHZ_FA] hunt capture freq=%.2f\n", best_f);
                        }
                        fflush(stdout);
                    } else {
                        printf("[SUBGHZ_FA] hunt timeout\n"); fflush(stdout);
                    }
                    subghz_cap_rx_stop();
                } else {
                    printf("[SUBGHZ_FA] hunt error\n"); fflush(stdout);
                }
            }
        } else {
            printf("[SUBGHZ_FA] silent\n"); fflush(stdout);
        }
        vTaskDelay(pdMS_TO_TICKS(hunt ? 30 : 120));
    }
    if (g_subghz_radio_ok) cc1101_set_idle(&g_subghz_radio);
    worker_exit();
}

static void weather_task(void *pv) {
    float freq = subghz_effective_freq();
    if (freq < 300 || freq > 930) freq = 433.92f;
    radio_rx_ook(freq);
    printf("[SUBGHZ_WEATHER_START] freq=%.2f\n", freq); fflush(stdout);
    if (subghz_cap_rx_start(g_subghz_radio.gdo0_pin) != 0) {
        printf("SubGHz receive start failed: rmt\n"); fflush(stdout);
        goto done;
    }
    while (!s_op_stop) {
        int n = subghz_cap_rx_get(s_cap_buf, SUBGHZ_MAX_EDGES, 40, 200);
        if (n > 0) {
            char proto[24], ch[8], temp[16], hum[8], batt[8];
            unsigned long id = 0;
            if (subghz_decode_weather(s_cap_buf, n, proto, sizeof(proto), &id,
                                      ch, sizeof(ch), temp, sizeof(temp),
                                      hum, sizeof(hum), batt, sizeof(batt))) {
                printf("[SUBGHZ_WEATHER] proto=%s id=0x%lX ch=%s temp=%s hum=%s batt=%s\n",
                       proto, id, ch, temp, hum, batt);
                fflush(stdout);
            }
        }
    }
done:
    subghz_cap_rx_stop();
    if (g_subghz_radio_ok) cc1101_set_idle(&g_subghz_radio);
    worker_exit();
}

static bool start_op(op_t op, TaskFunction_t fn, const char *name, uint32_t stack) {
    subghz_stop_all();
    s_op_stop = false;
    s_op = op;
    TaskHandle_t th = NULL;
    /* Suspend scheduling so the (higher-priority) worker can't reach worker_exit
     * before s_op_task is published; also avoids the volatile-qualifier discard. */
    vTaskSuspendAll();
    BaseType_t ok = xTaskCreate(fn, name, stack, NULL, 5, &th);
    if (ok == pdPASS) s_op_task = th;
    xTaskResumeAll();
    if (ok != pdPASS) { s_op = OP_NONE; s_op_task = NULL; return false; }
    return true;
}

/* ---- argument helpers ---- */
static bool find_int_token(int argc, char **argv, const char *key, long *out) {
    size_t kl = strlen(key);
    for (int i = 1; i < argc; i++)
        if (strncmp(argv[i], key, kl) == 0) { *out = strtol(argv[i] + kl, NULL, 10); return true; }
    return false;
}
static bool has_token(int argc, char **argv, const char *tok) {
    for (int i = 1; i < argc; i++) if (strcmp(argv[i], tok) == 0) return true;
    return false;
}

/* ---- command handlers ---- */

/* ===================== Ported radio apps (CC1101) ==========================
 * Spectrum analyzer/waterfall (#8), de Bruijn / brute-force OOK (#9), protocol
 * TX presets (#10) and an RSSI jamming detector (#11). Ported from Bruce
 * (rf_spectrum / rf_waterfall / rf_bruteforce / protocols/rf_registry) and
 * ESP32-DIV (SubBrute / jammingdetector). See docs/radio_apps_port.md.
 * All emit byte-exact [SUBGHZ_*] tokens the Tab5 parses.
 * ------------------------------------------------------------------------- */

/* ---- #9/#10 OOK protocol table (signed us: +carrier ON, -carrier OFF) ---- */
typedef struct { const char *name; int bits; int16_t zero[2], one[2], pre[2], post[2]; } brute_proto_t;
static const brute_proto_t BRUTE_PROTOS[] = {
    {"came",        12, {-320,640},  {-640,320},  {-11520,320}, {0,0}},
    {"nice",        12, {-700,1400}, {-1400,700}, {-25200,700}, {0,0}},
    {"holtek",      12, {-870,430},  {-430,870},  {-15480,430}, {0,0}},
    {"ansonic",     12, {-1111,555}, {-555,1111}, {-19425,555}, {0,0}},
    {"linear",      10, {500,-1500}, {1500,-500}, {0,0},        {500,-21500}},
    {"chamberlain",  9, {-870,430},  {-430,870},  {0,0},        {-3000,1000}},
};
#define BRUTE_PROTOS_N (int)(sizeof(BRUTE_PROTOS)/sizeof(BRUTE_PROTOS[0]))

static int find_brute_proto(const char *name) {
    for (int i = 0; i < BRUTE_PROTOS_N; i++)
        if (strcmp(name, BRUTE_PROTOS[i].name) == 0) return i;
    return -1;
}

/* Emit one signed-us half-pulse pair by bit-banging GDO0 (ASK async TX). */
static void ook_emit_pair(int gdo0, const int16_t p[2]) {
    for (int k = 0; k < 2; k++) {
        int e = p[k];
        if (e == 0) continue;
        gpio_set_level(gdo0, e > 0 ? 1 : 0);
        esp_rom_delay_us((uint32_t)(e > 0 ? e : -e));
    }
}
static void ook_emit_code(int gdo0, const brute_proto_t *p, uint32_t code, int bits) {
    ook_emit_pair(gdo0, p->pre);
    for (int j = bits - 1; j >= 0; j--)
        ook_emit_pair(gdo0, ((code >> j) & 1) ? p->one : p->zero);
    ook_emit_pair(gdo0, p->post);
}
static void ook_tx_setup(float f) {
    cc1101_set_idle(&g_subghz_radio);
    cc1101_set_modulation(&g_subghz_radio, CC1101_MOD_ASK);
    cc1101_set_ccmode(&g_subghz_radio, false);
    cc1101_set_frequency(&g_subghz_radio, f);
    cc1101_set_pa(&g_subghz_radio, 12);
    gpio_set_direction(g_subghz_radio.gdo0_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(g_subghz_radio.gdo0_pin, 0);
    cc1101_set_tx(&g_subghz_radio);
}
static void ook_tx_teardown(void) {
    gpio_set_level(g_subghz_radio.gdo0_pin, 0);
    cc1101_set_idle(&g_subghz_radio);
    gpio_set_direction(g_subghz_radio.gdo0_pin, GPIO_MODE_INPUT);
}

/* ---- #8 spectrum analyzer / waterfall ---- */
static float s_spec_lo = 433.0f, s_spec_hi = 434.0f, s_spec_step = 0.01f;
static int   s_spec_dwell_ms = 2;

static void spectrum_task(void *pv) {
    (void)pv;
    float lo = s_spec_lo, hi = s_spec_hi, step = s_spec_step;
    if (step < 0.001f) step = 0.001f;
    if (hi < lo) { float t = lo; lo = hi; hi = t; }
    int bins = (int)((hi - lo) / step) + 1;
    if (bins < 1) bins = 1;
    if (bins > 256) bins = 256;                 /* fits one compact line */
    static uint8_t data[256];
    static char hexbuf[513];
    static const char HEX[] = "0123456789abcdef";
    cc1101_set_idle(&g_subghz_radio);
    cc1101_set_modulation(&g_subghz_radio, CC1101_MOD_2FSK);
    cc1101_set_ccmode(&g_subghz_radio, false);
    cc1101_set_rxbw(&g_subghz_radio, 200.0f);
    printf("[SUBGHZ_SPECTRUM_START] lo=%.3f hi=%.3f step=%.4f bins=%d\n", lo, hi, step, bins);
    fflush(stdout);
    while (!s_op_stop) {
        float peak_f = lo; int peak_r = -200;
        for (int i = 0; i < bins && !s_op_stop; i++) {
            float f = lo + step * i;
            cc1101_set_frequency(&g_subghz_radio, f + g_subghz_correction);
            cc1101_set_rx(&g_subghz_radio);
            esp_rom_delay_us((uint32_t)s_spec_dwell_ms * 1000);
            int r = cc1101_get_rssi(&g_subghz_radio);
            if (r > peak_r) { peak_r = r; peak_f = f; }
            int b = r + 128; if (b < 0) b = 0; if (b > 255) b = 255;
            data[i] = (uint8_t)b;
        }
        for (int i = 0; i < bins; i++) { hexbuf[2*i] = HEX[data[i] >> 4]; hexbuf[2*i+1] = HEX[data[i] & 0xF]; }
        hexbuf[2*bins] = 0;
        printf("[SUBGHZ_SPECTRUM] lo=%.3f step=%.4f n=%d peak=%.3f prssi=%d data=%s\n",
               lo, step, bins, peak_f, peak_r, hexbuf);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (g_subghz_radio_ok) cc1101_set_idle(&g_subghz_radio);
    worker_exit();
}

/* ---- #9 de Bruijn / brute-force ---- */
static int s_brute_proto = 1;   /* default: nice */
static int s_brute_bits  = 0;   /* 0 => protocol default */
static int s_brute_reps  = 2;

static void brute_task(void *pv) {
    (void)pv;
    const brute_proto_t *p = &BRUTE_PROTOS[s_brute_proto];
    int bits = s_brute_bits > 0 ? s_brute_bits : p->bits;
    if (bits > 24) bits = 24;
    uint32_t total = 1u << bits;
    int gdo0 = g_subghz_radio.gdo0_pin;
    ook_tx_setup(subghz_effective_freq());
    printf("[SUBGHZ_BRUTE_START] proto=%s bits=%d total=%lu freq=%.2f\n",
           p->name, bits, (unsigned long)total, subghz_effective_freq());
    fflush(stdout);
    for (uint32_t code = 0; code < total && !s_op_stop; code++) {
        for (int r = 0; r < s_brute_reps && !s_op_stop; r++)
            ook_emit_code(gdo0, p, code, bits);
        if ((code & 0x3F) == 0) {
            printf("[SUBGHZ_BRUTE] code=%lu total=%lu\n", (unsigned long)code, (unsigned long)total);
            fflush(stdout);
        }
        vTaskDelay(1);   /* feed WDT + inter-code gap */
    }
    ook_tx_teardown();
    printf("[SUBGHZ_BRUTE_DONE]\n"); fflush(stdout);
    worker_exit();
}

/* ---- #11 jamming detector (adaptive floor + duty streak, DIV recipe) ---- */
static void jamdet_task(void *pv) {
    (void)pv;
    float freq = subghz_effective_freq();
    cc1101_set_idle(&g_subghz_radio);
    cc1101_set_modulation(&g_subghz_radio, CC1101_MOD_ASK);
    cc1101_set_ccmode(&g_subghz_radio, false);
    cc1101_set_rxbw(&g_subghz_radio, 650.0f);
    cc1101_set_frequency(&g_subghz_radio, freq);
    cc1101_set_rx(&g_subghz_radio);
    printf("[SUBGHZ_JAMDET_START] freq=%.2f\n", freq); fflush(stdout);
    float floor = -95.0f;
    int64_t streak_us = 0, last_emit = 0;
    float ring[20]; int rp = 0, rn = 0;
    const int JD_SAMPLES = 100;
    while (!s_op_stop) {
        int64_t t0 = esp_timer_get_time();
        float busyThresh = floor + 18.0f;
        if (busyThresh < -75.0f) busyThresh = -75.0f;
        int busy = 0, mn = 127;
        for (int i = 0; i < JD_SAMPLES && !s_op_stop; i++) {
            int r = cc1101_get_rssi(&g_subghz_radio);
            if (r < mn) mn = r;
            if ((float)r > busyThresh) busy++;
            esp_rom_delay_us(200);
        }
        float duty = (float)busy / JD_SAMPLES;
        if (duty < 0.2f) floor = 0.95f * floor + 0.05f * (float)mn;
        ring[rp] = duty; rp = (rp + 1) % 20; if (rn < 20) rn++;
        float avg = 0; for (int i = 0; i < rn; i++) avg += ring[i]; avg /= (rn ? rn : 1);
        int64_t elapsed = esp_timer_get_time() - t0;
        if (duty >= 0.5f) streak_us += elapsed; else streak_us = 0;
        bool jam = (streak_us >= 400000) || (avg >= 0.8f);
        const char *state = jam ? "jammed" : (duty >= 0.1f ? "activity" : "clear");
        int64_t now = esp_timer_get_time();
        if (now - last_emit >= 200000) {
            printf("[SUBGHZ_JAMDET] state=%s duty=%d avg=%d floor=%d\n",
                   state, (int)(duty * 100), (int)(avg * 100), (int)floor);
            fflush(stdout);
            last_emit = now;
        }
        vTaskDelay(1);   /* yield: let the REPL set s_op_stop + feed the idle WDT */
    }
    if (g_subghz_radio_ok) cc1101_set_idle(&g_subghz_radio);
    worker_exit();
}

/* ---- new command handlers ---- */
static int cmd_spectrum(int argc, char **argv) {
    if (argc >= 2) s_spec_lo = strtof(argv[1], NULL);
    if (argc >= 3) s_spec_hi = strtof(argv[2], NULL);
    if (argc >= 4) s_spec_step = strtof(argv[3], NULL);
    if (s_spec_lo <= 0) s_spec_lo = 433.0f;
    if (s_spec_hi <= 0) s_spec_hi = 434.0f;
    if (s_spec_step <= 0) s_spec_step = 0.01f;
    if (!subghz_ensure_radio()) return 0;
    start_op(OP_SPECTRUM, spectrum_task, "sg_spec", 4096);
    return 0;
}
static int cmd_brute(int argc, char **argv) {
    if (argc >= 2) { int pi = find_brute_proto(argv[1]); if (pi >= 0) s_brute_proto = pi; }
    long v;
    s_brute_bits = 0;
    if (find_int_token(argc, argv, "bits=", &v)) s_brute_bits = (int)v;
    if (find_int_token(argc, argv, "reps=", &v) && v > 0) s_brute_reps = (int)v;
    if (!subghz_ensure_radio()) return 0;
    start_op(OP_BRUTE, brute_task, "sg_brute", 4096);
    return 0;
}
static int cmd_jamdet(int argc, char **argv) {
    if (argc >= 2) { float f = strtof(argv[1], NULL); if (f > 0) g_subghz_freq = f; }
    if (!subghz_ensure_radio()) return 0;
    start_op(OP_JAMDET, jamdet_task, "sg_jd", 4096);
    return 0;
}

static int cmd_freq(int argc, char **argv) {
    if (argc >= 2) { float f = strtof(argv[1], NULL); if (f > 0) g_subghz_freq = f; }
    return 0;
}

static int cmd_rx(int argc, char **argv) {
    long floor = -80;
    if (!find_int_token(argc, argv, "rssi=", &floor)) floor = -80;
    s_rssi_floor = (int)floor;
    bool raw = has_token(argc, argv, "raw");
    if (!subghz_ensure_radio()) return 0;   /* verdict already printed */
    start_op(raw ? OP_LISTEN_RAW : OP_LISTEN, listen_task, "sg_listen", 5120);
    return 0;
}

static int cmd_stop(int argc, char **argv) { (void)argc;(void)argv; subghz_stop_all(); return 0; }

static int cmd_save(int argc, char **argv) {
    int idx = (argc >= 2) ? atoi(argv[1]) : 0;
    subghz_signal_t *s = subghz_store_get(idx);
    if (!s) { printf("[SUBGHZ_SAVE_ERR] idx=%d reason=noidx\n", idx); fflush(stdout); return 0; }
    char name[64];
    if (subghz_sd_save(s, name, sizeof(name)) == 0) {
        printf("[SUBGHZ_SAVE] idx=%d name=%s\n", idx, name);
    } else {
        printf("[SUBGHZ_SAVE_ERR] idx=%d reason=nosd\n", idx);
    }
    fflush(stdout);
    return 0;
}

static int cmd_tx(int argc, char **argv) {
    if (argc >= 2 && strcmp(argv[1], "tesla") == 0) {
        if (!subghz_ensure_radio()) return 0;
        subghz_stop_all();
        /* Tesla charge-port opener: public 315/433.92MHz OOK burst, ~400us base,
         * pattern repeated. Uses whatever freq the UI set (315.00 for US). */
        static const uint16_t tesla[] = {
            400,400, 400,400, 400,400, 400,400, 400,400,
            800,400, 400,800, 800,400, 400,800, 800,400,
            400,800, 800,400, 400,800, 800,400, 400,800,
            400,400, 400,400, 400,400, 400,400, 400,400
        };
        cc1101_set_idle(&g_subghz_radio);
        cc1101_set_modulation(&g_subghz_radio, CC1101_MOD_ASK);
        cc1101_set_ccmode(&g_subghz_radio, false);
        radio_tune(subghz_effective_freq());
        cc1101_set_pa(&g_subghz_radio, 12);
        cc1101_set_tx(&g_subghz_radio);
        esp_rom_delay_us(500);
        subghz_cap_tx_replay(g_subghz_radio.gdo0_pin, tesla,
                             (int)(sizeof(tesla)/sizeof(tesla[0])), 5);
        cc1101_set_idle(&g_subghz_radio);
        printf("[SUBGHZ_TX] idx=0\n");
        printf("[SUBGHZ_STATUS] tesla sent\n");
        fflush(stdout);
        return 0;
    }

    /* protocol TX preset: subghz_tx <proto> code=<n> [bits=<n>] (#10) */
    if (argc >= 2) {
        int pi = find_brute_proto(argv[1]);
        if (pi >= 0) {
            long code = 0, bv = 0;
            find_int_token(argc, argv, "code=", &code);
            int nb = (find_int_token(argc, argv, "bits=", &bv) && bv > 0)
                       ? (int)bv : BRUTE_PROTOS[pi].bits;
            if (nb > 32) nb = 32;
            if (!subghz_ensure_radio()) return 0;
            subghz_stop_all();
            int gdo0 = g_subghz_radio.gdo0_pin;
            ook_tx_setup(subghz_effective_freq());
            for (int r = 0; r < 3; r++) ook_emit_code(gdo0, &BRUTE_PROTOS[pi], (uint32_t)code, nb);
            ook_tx_teardown();
            printf("[SUBGHZ_TX] preset=%s code=%ld bits=%d\n", BRUTE_PROTOS[pi].name, code, nb);
            fflush(stdout);
            return 0;
        }
    }

    int idx = (argc >= 2) ? atoi(argv[1]) : 0;
    bool sd = (argc >= 3 && strncmp(argv[2], "sd", 2) == 0);
    subghz_signal_t sig;
    subghz_signal_t *src = NULL;
    if (sd) { if (subghz_sd_load(idx, &sig) == 0) src = &sig; }
    else    { src = subghz_store_get(idx); }
    if (!src || src->edges < 2) {
        printf("[SUBGHZ_STATUS] tx error idx=%d\n", idx); fflush(stdout);
        return 0;
    }
    if (!subghz_ensure_radio()) return 0;
    subghz_stop_all();
    cc1101_set_idle(&g_subghz_radio);
    cc1101_set_modulation(&g_subghz_radio, CC1101_MOD_ASK);
    cc1101_set_ccmode(&g_subghz_radio, false);
    cc1101_set_frequency(&g_subghz_radio, src->freq + g_subghz_correction);
    cc1101_set_pa(&g_subghz_radio, 12);
    cc1101_set_tx(&g_subghz_radio);
    esp_rom_delay_us(500);
    subghz_cap_tx_replay(g_subghz_radio.gdo0_pin, src->timings, src->edges, 5);
    cc1101_set_idle(&g_subghz_radio);
    printf("[SUBGHZ_TX] idx=%d\n", idx); fflush(stdout);
    return 0;
}

static int cmd_list(int argc, char **argv) {
    const char *src = (argc >= 2) ? argv[1] : "mem";
    subghz_list_emit(src);
    return 0;
}

static int cmd_rename(int argc, char **argv) {
    int idx = (argc >= 2) ? atoi(argv[1]) : 0;
    const char *nm = (argc >= 3) ? argv[2] : "";
    char reason[32] = {0};
    if (nm[0] && subghz_sd_rename(idx, nm, reason, sizeof(reason)) == 0) {
        char base[48]; snprintf(base, sizeof(base), "%s", nm);
        for (char *p = base; *p; p++) if (*p == ' ' || *p == '/' || *p == '\\') *p = '_';
        printf("[SUBGHZ_RENAME] idx=%d name=%s\n", idx, base);
    } else {
        printf("[SUBGHZ_RENAME_ERR] idx=%d reason=%s\n", idx, reason[0] ? reason : "bad_args");
    }
    fflush(stdout);
    return 0;
}

static int cmd_delete(int argc, char **argv) {
    int idx = (argc >= 2) ? atoi(argv[1]) : 0;
    char reason[32] = {0};
    if (subghz_sd_delete(idx, reason, sizeof(reason)) == 0) {
        printf("[SUBGHZ_DELETE] idx=%d\n", idx);
    } else {
        printf("[SUBGHZ_DELETE_ERR] idx=%d reason=%s\n", idx, reason[0] ? reason : "error");
    }
    fflush(stdout);
    return 0;
}

static int cmd_jam(int argc, char **argv) {
    (void)argc; (void)argv;
    if (!subghz_ensure_radio()) { printf("SubGHz jammer start failed: no radio\n"); fflush(stdout); return 0; }
    start_op(OP_JAM, jammer_task, "sg_jam", 3072);
    return 0;
}

static int cmd_freq_analyzer(int argc, char **argv) {
    long floor = -70;
    if (argc >= 2) floor = strtol(argv[1], NULL, 10);
    s_rssi_floor = (int)floor;
    bool hunt = has_token(argc, argv, "hunt");
    long to = 2000;
    if (find_int_token(argc, argv, "timeout=", &to) && to > 0) s_hunt_timeout = (uint32_t)to;
    if (!subghz_ensure_radio()) return 0;
    start_op(hunt ? OP_HUNT : OP_ANALYZER, analyzer_task, "sg_fa", 5120);
    return 0;
}

static int cmd_scanner(int argc, char **argv) {
    long dwell = 80, edges = 4, floor = -80;
    if (find_int_token(argc, argv, "dwell=", &dwell)) s_scan_dwell = (uint32_t)(dwell > 0 ? dwell : 80);
    else s_scan_dwell = 80;
    if (find_int_token(argc, argv, "edges=", &edges)) s_scan_edges = (uint32_t)(edges >= 0 ? edges : 4);
    else s_scan_edges = 4;
    /* the bare signed integer arg is the rssi floor */
    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "dwell=", 6) == 0 || strncmp(argv[i], "edges=", 6) == 0) continue;
        if (strcmp(argv[i], "fast") == 0) continue;
        char *end = NULL; long v = strtol(argv[i], &end, 10);
        if (end && end != argv[i]) { floor = v; break; }
    }
    s_rssi_floor = (int)floor;
    if (!subghz_ensure_radio()) return 0;
    start_op(OP_SCANNER, scanner_task, "sg_scan", 4096);
    return 0;
}

static void save_correction(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, NVS_CORR, (int32_t)lroundf(g_subghz_correction * 100.0f));
        nvs_commit(h);
        nvs_close(h);
    }
}
static void load_correction(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        int32_t v = 0;
        if (nvs_get_i32(h, NVS_CORR, &v) == ESP_OK) g_subghz_correction = v / 100.0f;
        nvs_close(h);
    }
}

static int cmd_get_corr(int argc, char **argv) {
    (void)argc; (void)argv;
    printf("[SUBGHZ_FREQ_CORRECTION] %+.2f\n", g_subghz_correction);
    fflush(stdout);
    return 0;
}
static int cmd_set_corr(int argc, char **argv) {
    if (argc >= 2) {
        float v = strtof(argv[1], NULL);
        if (v > 5.0f) v = 5.0f;
        if (v < -5.0f) v = -5.0f;
        g_subghz_correction = v;
        save_correction();
    }
    printf("[SUBGHZ_FREQ_CORRECTION] %+.2f\n", g_subghz_correction);
    fflush(stdout);
    return 0;
}

static int cmd_status(int argc, char **argv) {
    (void)argc; (void)argv;
    printf("[SUBGHZ_STATUS] radio=%s freq=%.2f corr=%+.2f op=%d slots=%d\n",
           (s_radio_init_attempted && g_subghz_radio_ok) ? "ok" :
           (s_radio_init_attempted ? "none" : "unknown"),
           subghz_effective_freq(), g_subghz_correction, (int)s_op, subghz_store_count());
    fflush(stdout);
    return 0;
}

static int cmd_weather(int argc, char **argv) {
    (void)argc; (void)argv;
    if (!subghz_ensure_radio()) return 0;
    start_op(OP_WEATHER, weather_task, "sg_wx", 5120);
    return 0;
}

static int cmd_init(int argc, char **argv) { (void)argc;(void)argv; subghz_ensure_radio(); return 0; }

#define REG(cmdname, fn, helptxt) do { \
    const esp_console_cmd_t _c = { .command = cmdname, .help = helptxt, .hint = NULL, .func = (fn), .argtable = NULL }; \
    esp_console_cmd_register(&_c); } while (0)

void subghz_register_commands(void) {
    subghz_store_init();
    load_correction();
    REG("subghz_freq", cmd_freq, "Set SubGHz frequency (MHz)");
    REG("subghz_rx", cmd_rx, "Start SubGHz receive [raw] rssi=<dBm>");
    REG("subghz_stop", cmd_stop, "Stop SubGHz operations");
    REG("subghz_save", cmd_save, "Save capture <idx> to SD");
    REG("subghz_tx", cmd_tx, "Transmit <idx> mem|sd, or tesla");
    REG("subghz_list", cmd_list, "List stored signals <mem|sd>");
    REG("subghz_rename", cmd_rename, "Rename SD signal <idx> <name>");
    REG("subghz_delete", cmd_delete, "Delete SD signal <idx>");
    REG("subghz_jam", cmd_jam, "Start SubGHz jammer on current freq");
    REG("subghz_freq_analyzer", cmd_freq_analyzer, "Freq analyzer <rssi> [hunt timeout=<ms>]");
    REG("subghz_scanner", cmd_scanner, "Scan ISM freqs dwell= edges= <rssi> [fast]");
    REG("subghz_get_freq_correction", cmd_get_corr, "Get freq correction (MHz)");
    REG("subghz_set_freq_correction", cmd_set_corr, "Set freq correction (MHz)");
    REG("subghz_status", cmd_status, "Print SubGHz status");
    REG("subghz_weather", cmd_weather, "Receive weather sensors");
    REG("init_cc1101", cmd_init, "Init/detect CC1101");
    REG("subghz_spectrum", cmd_spectrum, "Spectrum sweep [lo hi step] MHz");
    REG("subghz_brute", cmd_brute, "OOK brute-force <proto> [bits= reps=]");
    REG("subghz_jamdet", cmd_jamdet, "Jamming detector [freq]");
}

void subghz_early_init(void) {
    cc1101_default_config(&g_subghz_radio);
    g_subghz_radio.mhz = subghz_effective_freq();
    g_subghz_radio_ok = cc1101_init(&g_subghz_radio);
    s_radio_init_attempted = true;
}

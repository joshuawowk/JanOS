/* RMT-based OOK capture (RX) and replay (TX) on the CC1101 GDO0 pin, plus a
 * lightweight GPIO edge counter for the scanner/analyzer. ESP32-C5 / IDF 5.4.
 *
 * Async serial CC1101 mode: during RX the CC1101 drives GDO0 with demodulated
 * data (ESP reads it via RMT RX); during TX the CC1101 samples GDO0 as input
 * (ESP drives it via RMT TX). We create/destroy channels per operation so the
 * two never contend for the pad, and so the NeoPixel RMT TX channel is spared.
 */
#include "subghz_priv.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_heap_caps.h"

#define SUBGHZ_RMT_RES_HZ   1000000u   /* 1 tick = 1 us */
#define RX_USER_SYMBOLS     256        /* up to 512 edges per burst */
#define RX_MEM_BLOCK        48         /* C5: 48 words/channel */
/* Glitch filter and end-of-packet idle for OOK remotes. Bench-tunable. */
#define RX_MIN_NS           (50u * 1000u)     /* ignore <50us pulses */
#define RX_MAX_NS           (12u * 1000u * 1000u) /* 12ms idle = burst end */

static rmt_channel_handle_t s_rx_chan = NULL;
static QueueHandle_t        s_rx_queue = NULL;
static rmt_symbol_word_t   *s_rx_symbols = NULL;
static rmt_receive_config_t s_rx_cfg;

static bool IRAM_ATTR rx_done_cb(rmt_channel_handle_t ch,
                                 const rmt_rx_done_event_data_t *edata, void *user) {
    (void)ch; (void)user;
    BaseType_t hp = pdFALSE;
    if (s_rx_queue) xQueueSendFromISR(s_rx_queue, edata, &hp);
    return hp == pdTRUE;
}

int subghz_cap_rx_start(int gdo0_pin) {
    if (s_rx_chan) return 0;
    bool enabled = false;
    if (!s_rx_symbols) {
        s_rx_symbols = heap_caps_malloc(RX_USER_SYMBOLS * sizeof(rmt_symbol_word_t),
                                        MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
        if (!s_rx_symbols) goto fail;
    }
    if (!s_rx_queue) {
        s_rx_queue = xQueueCreate(4, sizeof(rmt_rx_done_event_data_t));
        if (!s_rx_queue) goto fail;
    }
    rmt_rx_channel_config_t cfg = {
        .gpio_num = gdo0_pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = SUBGHZ_RMT_RES_HZ,
        .mem_block_symbols = RX_MEM_BLOCK,
        .flags = { .invert_in = false, .with_dma = false },
    };
    if (rmt_new_rx_channel(&cfg, &s_rx_chan) != ESP_OK) { s_rx_chan = NULL; goto fail; }
    rmt_rx_event_callbacks_t cbs = { .on_recv_done = rx_done_cb };
    if (rmt_rx_register_event_callbacks(s_rx_chan, &cbs, NULL) != ESP_OK) goto fail;
    if (rmt_enable(s_rx_chan) != ESP_OK) goto fail;
    enabled = true;
    s_rx_cfg.signal_range_min_ns = RX_MIN_NS;
    s_rx_cfg.signal_range_max_ns = RX_MAX_NS;
    s_rx_cfg.flags.en_partial_rx = 0;
    if (rmt_receive(s_rx_chan, s_rx_symbols,
                    RX_USER_SYMBOLS * sizeof(rmt_symbol_word_t), &s_rx_cfg) != ESP_OK) goto fail;
    return 0;
fail:
    if (enabled && s_rx_chan) rmt_disable(s_rx_chan);
    if (s_rx_chan) { rmt_del_channel(s_rx_chan); s_rx_chan = NULL; }
    if (s_rx_queue) { vQueueDelete(s_rx_queue); s_rx_queue = NULL; }
    if (s_rx_symbols) { heap_caps_free(s_rx_symbols); s_rx_symbols = NULL; }
    return -1;
}

void subghz_cap_rx_stop(void) {
    if (s_rx_chan) {
        rmt_disable(s_rx_chan);
        rmt_del_channel(s_rx_chan);
        s_rx_chan = NULL;
    }
    if (s_rx_queue) { vQueueDelete(s_rx_queue); s_rx_queue = NULL; }
    if (s_rx_symbols) { heap_caps_free(s_rx_symbols); s_rx_symbols = NULL; }
}

int subghz_cap_rx_get(uint16_t *timings, int max_edges, int min_edges, uint32_t timeout_ms) {
    if (!s_rx_chan || !s_rx_queue) return 0;
    rmt_rx_done_event_data_t ev;
    if (xQueueReceive(s_rx_queue, &ev, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) return 0;
    int n = 0;
    for (size_t i = 0; i < ev.num_symbols && n < max_edges - 1; i++) {
        if (ev.received_symbols[i].duration0) timings[n++] = ev.received_symbols[i].duration0;
        if (ev.received_symbols[i].duration1) timings[n++] = ev.received_symbols[i].duration1;
    }
    /* Re-arm for the next burst (buffer is now consumed). */
    rmt_receive(s_rx_chan, s_rx_symbols,
                RX_USER_SYMBOLS * sizeof(rmt_symbol_word_t), &s_rx_cfg);
    if (n < min_edges) return 0;
    return n;
}

unsigned subghz_cap_edge_count(int gdo0_pin, uint32_t dwell_ms) {
    int last = gpio_get_level(gdo0_pin);
    unsigned edges = 0;
    int64_t end = esp_timer_get_time() + (int64_t)dwell_ms * 1000;
    while (esp_timer_get_time() < end) {
        int cur = gpio_get_level(gdo0_pin);
        if (cur != last) { edges++; last = cur; }
    }
    return edges;
}

int subghz_cap_tx_replay(int gdo0_pin, const uint16_t *timings, int edges, int repeats) {
    if (edges < 2 || repeats < 1) return -1;
    rmt_channel_handle_t tx = NULL;
    rmt_tx_channel_config_t cfg = {
        .gpio_num = gdo0_pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = SUBGHZ_RMT_RES_HZ,
        .mem_block_symbols = RX_MEM_BLOCK,
        .trans_queue_depth = 4,
        .flags = { .invert_out = false, .with_dma = false },
    };
    if (rmt_new_tx_channel(&cfg, &tx) != ESP_OK) return -1;

    rmt_encoder_handle_t enc = NULL;
    rmt_copy_encoder_config_t ecfg = {};
    if (rmt_new_copy_encoder(&ecfg, &enc) != ESP_OK) { rmt_del_channel(tx); return -1; }

    /* A 15-bit RMT duration maxes at 32767us; split longer edges (common in
     * imported .sub inter-frame gaps) across extra same-level half-slots so the
     * replayed waveform stays faithful across the full uint16_t range. */
    size_t halves = 0;
    for (int i = 0; i < edges; i++) {
        uint32_t d = timings[i] ? timings[i] : 1;
        halves += (d + 0x7FFEu) / 0x7FFFu;      /* ceil(d/32767), >=1 */
    }
    size_t nsym = (halves + 1) / 2;
    rmt_symbol_word_t *sym = heap_caps_calloc(nsym, sizeof(rmt_symbol_word_t),
                                              MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
    if (!sym) { rmt_del_encoder(enc); rmt_del_channel(tx); return -1; }
    size_t half = 0;
    for (int i = 0; i < edges; i++) {
        int level = (i % 2 == 0) ? 1 : 0;       /* first edge = carrier ON */
        uint32_t remaining = timings[i] ? timings[i] : 1;
        while (remaining > 0) {
            uint16_t chunk = (remaining > 0x7FFFu) ? 0x7FFFu : (uint16_t)remaining;
            remaining -= chunk;
            size_t si = half >> 1;
            if ((half & 1u) == 0) { sym[si].level0 = level; sym[si].duration0 = chunk; }
            else                  { sym[si].level1 = level; sym[si].duration1 = chunk; }
            half++;
        }
    }
    /* Odd trailing half stays {level0? ...} with second half {0,0} = end marker. */

    int rc = -1;
    if (rmt_enable(tx) == ESP_OK) {
        rmt_transmit_config_t tc = { .loop_count = 0 };
        rc = 0;
        for (int r = 0; r < repeats; r++) {
            if (rmt_transmit(tx, enc, sym, nsym * sizeof(rmt_symbol_word_t), &tc) != ESP_OK) { rc = -1; break; }
            rmt_tx_wait_all_done(tx, 1000);
            if (r + 1 < repeats) esp_rom_delay_us(20000); /* ~20ms inter-frame gap */
        }
        rmt_disable(tx);
    }
    heap_caps_free(sym);
    rmt_del_encoder(enc);
    rmt_del_channel(tx);
    /* Restore GDO0 as input for subsequent RX. */
    gpio_set_direction(gdo0_pin, GPIO_MODE_INPUT);
    return rc;
}

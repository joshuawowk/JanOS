#include "nrf24_jammer.h"
#include "nrf24.h"

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "soc/spi_periph.h"

#define TAG "nrf24_jammer"

/* Pin map: the nRF24 shares the SD card's SPI2 bus (SCK/MOSI/MISO) and uses its
 * own CS + CE. Change these if you wire the module differently. */
#define NRF24_SPI_HOST   SPI2_HOST
#define NRF24_SCK_PIN    6   /* shared with SD_CLK */
#define NRF24_MOSI_PIN   8   /* shared with SD_MOSI (moved off strapping GPIO7) */
#define NRF24_MISO_PIN   5   /* shared with SD_MISO (moved off strapping GPIO2) */
#define NRF24_CS_PIN     3   /* dedicated chip-select (was CC1101 CS) */
#define NRF24_CE_PIN     4   /* dedicated chip-enable (was CC1101 GDO0) */
/* NOTE: GPIO2 (MTMS) and GPIO7 are ESP32-C5 strapping pins; the DevKitC-1 puts
 * a boot-stabilizing cap on them, so they only pass clean SPI up to ~250 kHz.
 * MISO/MOSI were moved to clean pins GPIO5/GPIO8 for full 2 MHz. Verified with
 * the `spitest pad <n>` scanner: 2 & 7 = RC-loaded, 5/8/9/6/3/4 = clean. */

/* PA max with the LNA bit set (RF_SETUP low nibble = 0b0111), matching the
 * Arduino RF24 setPALevel(RF24_PA_MAX, true). */
#define NRF24_TX_POWER   7

/* The jam task runs below the console, so `stop` is handled by preemption and
 * we no longer need frequent yields. The only remaining reason to block is to
 * feed the idle-task watchdog (5 s timeout): sweep continuously for this long,
 * then take a single 10 ms (1 tick) breather. ~2 s keeps the carrier sweeping
 * ~99.5% of the time. */
#define JAM_WDT_FEED_US  2000000

static nrf24_device_t s_dev;
static bool s_initialized = false;

static volatile bool s_jam_stop = true;
static volatile bool s_jam_running = false;
static nrf24_jam_band_t s_band = JAM_ALL;
static TaskHandle_t s_jam_task = NULL;

void nrf24_jammer_spi_selftest(void) {
    if (!s_initialized) {
        memset(&s_dev, 0, sizeof(s_dev));
        s_dev.host = NRF24_SPI_HOST;
        s_dev.sck_pin = NRF24_SCK_PIN;
        s_dev.mosi_pin = NRF24_MOSI_PIN;
        s_dev.miso_pin = NRF24_MISO_PIN;
        s_dev.cs_pin = NRF24_CS_PIN;
        s_dev.ce_pin = NRF24_CE_PIN;
        s_dev.initialized = false;
        if (!nrf24_init(&s_dev)) {
            printf("[SPI-TEST] SPI device init failed\n");
            return;
        }
        s_initialized = true;
    }
    uint8_t tx[4] = {0xA5, 0x3C, 0x0F, 0xF0};
    uint8_t rx[4] = {0xEE, 0xEE, 0xEE, 0xEE};
    nrf24_spi_trx(&s_dev, tx, rx, 4, 100);
    printf("[SPI-TEST] tx=A5 3C 0F F0  rx=%02X %02X %02X %02X\n",
           rx[0], rx[1], rx[2], rx[3]);
    printf("[SPI-TEST]  With MOSI(GPIO8) jumpered to MISO(GPIO5), nRF24 unplugged:\n");
    printf("[SPI-TEST]   rx == tx (A5 3C 0F F0) => C5 SPI + GPIO2 are OK; fault is the module or its MISO wire.\n");
    printf("[SPI-TEST]   rx all 00 / all FF     => GPIO2 / SPI2 not working on the C5 (firmware/board issue).\n");
    fflush(stdout);
}

static bool ensure_dev(void) {
    if (!s_initialized) {
        memset(&s_dev, 0, sizeof(s_dev));
        s_dev.host = NRF24_SPI_HOST;
        s_dev.sck_pin = NRF24_SCK_PIN;
        s_dev.mosi_pin = NRF24_MOSI_PIN;
        s_dev.miso_pin = NRF24_MISO_PIN;
        s_dev.cs_pin = NRF24_CS_PIN;
        s_dev.ce_pin = NRF24_CE_PIN;
        s_dev.initialized = false;
        if (!nrf24_init(&s_dev)) {
            printf("[SPI-TEST] SPI device init failed\n");
            return false;
        }
        s_initialized = true;
    }
    return true;
}

/* Spare GPIO the firmware never wires to anything (GDO2 was dropped, freeing
 * GPIO5). Used purely as a scratch node for the internal loopback so the real
 * SPI pins (and their IO_MUX config) are never disturbed. */
#define NRF24_LOOPBACK_SCRATCH_PIN 9   /* free clean pad; GPIO5 is now MISO */

void nrf24_jammer_spi_selftest_internal(void) {
    if (!ensure_dev()) return;
    /* True internal loopback: fan the MOSI OUTPUT signal (spid_out) out to a
     * spare GPIO, and route the MISO INPUT signal (spiq_in) to read that same
     * spare GPIO. The shift register clocks its own tx bits back in, entirely
     * inside the chip -- no jumper, no module, and none of the real SPI pins are
     * touched. If this echoes the pattern, the SPI peripheral + shift path are
     * provably healthy and the 0x00 on the real bus is downstream (GPIO2 net,
     * SCK/MISO wiring, or the module itself). */
    uint32_t spid_out = spi_periph_signal[NRF24_SPI_HOST].spid_out;
    uint32_t spiq_in  = spi_periph_signal[NRF24_SPI_HOST].spiq_in;
    int scratch = NRF24_LOOPBACK_SCRATCH_PIN;

    gpio_reset_pin((gpio_num_t)scratch);
    gpio_set_direction((gpio_num_t)scratch, GPIO_MODE_INPUT_OUTPUT);
    esp_rom_gpio_connect_out_signal((uint32_t)scratch, spid_out, false, false); /* MOSI out -> scratch */
    esp_rom_gpio_connect_in_signal((uint32_t)scratch, spiq_in, false);          /* scratch  -> MISO in */

    uint8_t tx[4] = {0xA5, 0x3C, 0x0F, 0xF0};
    uint8_t rx[4] = {0xEE, 0xEE, 0xEE, 0xEE};
    nrf24_spi_trx(&s_dev, tx, rx, 4, 100);

    /* Restore MISO input to the real MISO pin (GPIO2). MOSI's real routing is
     * left as-is (still driven on GPIO7); a reboot fully restores the scratch. */
    esp_rom_gpio_connect_in_signal((uint32_t)NRF24_MISO_PIN, spiq_in, false);

    printf("[SPI-INT] internal loopback via spare GPIO%d (real SPI pins untouched)  tx=A5 3C 0F F0  rx=%02X %02X %02X %02X\n",
           scratch, rx[0], rx[1], rx[2], rx[3]);
    printf("[SPI-INT]   rx == tx (A5 3C 0F F0) => SPI peripheral + shift path HEALTHY; 0x00 is downstream (module/MISO/SCK wiring).\n");
    printf("[SPI-INT]   rx all 00               => SPI peripheral itself isn't shifting/clocking (firmware/board).\n");
    fflush(stdout);
}

void nrf24_jammer_spi_selftest_slow(int khz) {
    if (!ensure_dev()) return;
    if (khz <= 0) khz = 200;
    /* Real-pin loopback (MOSI GPIO8 -> external jumper -> MISO GPIO5) at a
     * caller-chosen clock. Sweeping the clock finds the fastest rate the RC-
     * loaded GPIO2 pad can still read cleanly, so the production SPI clock can be
     * set just below that knee (with margin) -- no rewiring needed. */
    spi_device_interface_config_t cfg = {
        .clock_speed_hz = khz * 1000,
        .mode = 0,
        .spics_io_num = -1,   /* CS irrelevant for a wire loopback */
        .queue_size = 1,
        .command_bits = 0,
        .address_bits = 0,
    };
    spi_device_handle_t h = NULL;
    esp_err_t r = spi_bus_add_device(NRF24_SPI_HOST, &cfg, &h);
    if (r != ESP_OK) { printf("[SPI-SLOW] add_device failed: %s\n", esp_err_to_name(r)); return; }
    uint8_t tx[4] = {0xA5, 0x3C, 0x0F, 0xF0};
    uint8_t rx[4] = {0xEE, 0xEE, 0xEE, 0xEE};
    spi_transaction_t t = { .length = 32, .rxlength = 32, .tx_buffer = tx, .rx_buffer = rx };
    spi_device_polling_transmit(h, &t);
    spi_bus_remove_device(h);
    bool ok = (rx[0] == tx[0] && rx[1] == tx[1] && rx[2] == tx[2] && rx[3] == tx[3]);
    printf("[SPI-SLOW] %5dkHz real-pin loopback (GPIO8->jumper->GPIO5)  rx=%02X %02X %02X %02X  %s\n",
           khz, rx[0], rx[1], rx[2], rx[3], ok ? "OK (clean echo)" : "GARBLED");
    fflush(stdout);
}

void nrf24_jammer_pad_test(int pin) {
    if (!ensure_dev()) return;
    /* Self-drive + read a single real pad at 2 MHz: route the MOSI output signal
     * and the MISO input signal both to `pin`, so the SPI engine drives the pad
     * and reads it back. A clean, unloaded pad echoes the pattern; an RC/capped
     * pad (like the GPIO2 strapping pin) garbles. Lets us scan candidate pins for
     * a clean MISO/MOSI home with no jumper and no rewiring. */
    uint32_t spid_out = spi_periph_signal[NRF24_SPI_HOST].spid_out;
    uint32_t spiq_in  = spi_periph_signal[NRF24_SPI_HOST].spiq_in;

    gpio_reset_pin((gpio_num_t)pin);
    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT_OUTPUT);
    esp_rom_gpio_connect_out_signal((uint32_t)pin, spid_out, false, false);
    esp_rom_gpio_connect_in_signal((uint32_t)pin, spiq_in, false);

    uint8_t tx[4] = {0xA5, 0x3C, 0x0F, 0xF0};
    uint8_t rx[4] = {0xEE, 0xEE, 0xEE, 0xEE};
    nrf24_spi_trx(&s_dev, tx, rx, 4, 100);

    /* Detach the peripheral output from this pad, restore MISO input to GPIO2. */
    gpio_reset_pin((gpio_num_t)pin);
    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT);
    esp_rom_gpio_connect_in_signal((uint32_t)NRF24_MISO_PIN, spiq_in, false);

    bool ok = (rx[0] == tx[0] && rx[1] == tx[1] && rx[2] == tx[2] && rx[3] == tx[3]);
    printf("[PADTEST] GPIO%-2d self-drive+read @2MHz  rx=%02X %02X %02X %02X  %s\n",
           pin, rx[0], rx[1], rx[2], rx[3], ok ? "CLEAN (fast pad, good for SPI)" : "loaded/garbled (RC pad)");
    fflush(stdout);
}

void nrf24_jammer_probe_at_khz(int khz) {
    if (!ensure_dev()) return;
    if (khz <= 0) khz = 1000;
    /* Re-add the nRF24 SPI device at a given clock and run a full register
     * round-trip (write RF_CH, read it back) plus a STATUS read. Lets us sweep
     * the clock live against the real module to find the rate the RC-loaded MISO
     * line can actually deliver a clean multi-byte read at. */
    if (s_dev.spi) { spi_bus_remove_device(s_dev.spi); s_dev.spi = NULL; }
    spi_device_interface_config_t cfg = {
        .clock_speed_hz = khz * 1000,
        .mode = 0,
        .spics_io_num = NRF24_CS_PIN,
        .queue_size = 1,
        .command_bits = 0,
        .address_bits = 0,
    };
    if (spi_bus_add_device(NRF24_SPI_HOST, &cfg, &s_dev.spi) != ESP_OK) {
        printf("[NRF24-PROBE] %dkHz add_device failed\n", khz);
        return;
    }
    uint8_t status = nrf24_status(&s_dev);
    uint8_t rb1 = 0xAB, rb2 = 0xAB;
    nrf24_write_reg(&s_dev, REG_RF_CH, 0x0A);
    nrf24_read_reg(&s_dev, REG_RF_CH, &rb1, 1);
    nrf24_write_reg(&s_dev, REG_RF_CH, 0x55);
    nrf24_read_reg(&s_dev, REG_RF_CH, &rb2, 1);
    printf("[NRF24-PROBE] %4dkHz  STATUS=0x%02X  RF_CH wr0A->rd%02X  wr55->rd%02X  %s\n",
           khz, status, rb1, rb2, (rb1 == 0x0A && rb2 == 0x55) ? "PASS" : "fail");
    fflush(stdout);
}

/* Bit-banged SPI byte, mode 0 (CPOL0/CPHA0), with a settle delay around every
 * edge so a cap-slowed MISO has time to reach a valid level before we sample. */
static uint8_t bb_xfer(uint8_t out, int settle_us) {
    uint8_t in = 0;
    for (int i = 7; i >= 0; i--) {
        gpio_set_level((gpio_num_t)NRF24_MOSI_PIN, (out >> i) & 1);
        esp_rom_delay_us(settle_us);                 /* MOSI setup */
        gpio_set_level((gpio_num_t)NRF24_SCK_PIN, 1); /* rising edge: data latched */
        esp_rom_delay_us(settle_us);                 /* let MISO settle through the RC */
        in = (uint8_t)((in << 1) | (gpio_get_level((gpio_num_t)NRF24_MISO_PIN) & 1));
        gpio_set_level((gpio_num_t)NRF24_SCK_PIN, 0); /* falling edge */
        esp_rom_delay_us(settle_us);
    }
    return in;
}

void nrf24_jammer_bitbang_probe(int settle_us) {
    if (!ensure_dev()) return;
    if (settle_us <= 0) settle_us = 500;
    /* Release the hardware-SPI device and drive the pins as plain GPIO so we can
     * clock slowly enough to beat the MISO-line capacitance. Reboot restores. */
    if (s_dev.spi) { spi_bus_remove_device(s_dev.spi); s_dev.spi = NULL; }
    gpio_reset_pin((gpio_num_t)NRF24_SCK_PIN);  gpio_set_direction((gpio_num_t)NRF24_SCK_PIN,  GPIO_MODE_OUTPUT); gpio_set_level((gpio_num_t)NRF24_SCK_PIN, 0);
    gpio_reset_pin((gpio_num_t)NRF24_MOSI_PIN); gpio_set_direction((gpio_num_t)NRF24_MOSI_PIN, GPIO_MODE_OUTPUT); gpio_set_level((gpio_num_t)NRF24_MOSI_PIN, 0);
    gpio_reset_pin((gpio_num_t)NRF24_CS_PIN);   gpio_set_direction((gpio_num_t)NRF24_CS_PIN,   GPIO_MODE_OUTPUT); gpio_set_level((gpio_num_t)NRF24_CS_PIN, 1);
    gpio_reset_pin((gpio_num_t)NRF24_MISO_PIN); gpio_set_direction((gpio_num_t)NRF24_MISO_PIN, GPIO_MODE_INPUT);

    /* Write RF_CH = 0x2E */
    gpio_set_level((gpio_num_t)NRF24_CS_PIN, 0); esp_rom_delay_us(settle_us);
    bb_xfer(W_REGISTER | (REGISTER_MASK & REG_RF_CH), settle_us);
    bb_xfer(0x2E, settle_us);
    gpio_set_level((gpio_num_t)NRF24_CS_PIN, 1); esp_rom_delay_us(settle_us);

    /* Read RF_CH back (first byte returns STATUS, second the register value) */
    gpio_set_level((gpio_num_t)NRF24_CS_PIN, 0); esp_rom_delay_us(settle_us);
    uint8_t status = bb_xfer(R_REGISTER | (REGISTER_MASK & REG_RF_CH), settle_us);
    uint8_t val    = bb_xfer(0xFF, settle_us);
    gpio_set_level((gpio_num_t)NRF24_CS_PIN, 1);

    printf("[NRF24-BB] settle=%dus/edge  STATUS=0x%02X  RF_CH wr2E->rd%02X  %s\n",
           settle_us, status, val, (val == 0x2E) ? "PASS -- module reads cleanly (MISO cap workaround works)" : "fail");
    printf("[NRF24-BB]   (SPI torn down for bit-bang; reboot to restore hardware SPI)\n");
    fflush(stdout);
}

const char* nrf24_jammer_band_name(nrf24_jam_band_t band) {
    switch (band) {
        case JAM_BLE: return "ble";
        case JAM_BT: return "bt";
        case JAM_WIFI: return "wifi";
        case JAM_DRONE: return "drone";
        case JAM_ALL: return "all";
        default: return "all";
    }
}

bool nrf24_jammer_init(void) {
    if (!s_initialized) {
        memset(&s_dev, 0, sizeof(s_dev));
        s_dev.host = NRF24_SPI_HOST;
        s_dev.sck_pin = NRF24_SCK_PIN;
        s_dev.mosi_pin = NRF24_MOSI_PIN;
        s_dev.miso_pin = NRF24_MISO_PIN;
        s_dev.cs_pin = NRF24_CS_PIN;
        s_dev.ce_pin = NRF24_CE_PIN;
        s_dev.initialized = false;

        if (!nrf24_init(&s_dev)) {
            ESP_LOGE(TAG, "SPI device init failed");
            return false;
        }
        s_initialized = true;
    }

    bool connected = nrf24_check_connected(&s_dev);
    if (!connected && !s_dev.bb_mode) {
        /* Hardware SPI couldn't read the module. If the MISO line is electrically
         * slow (RC-loaded net or a weak module output driver), no hardware-SPI
         * clock is slow enough -- but a bit-banged read that waits for MISO to
         * settle can still reach the module. Tear down the HW SPI device, switch
         * the pins to GPIO, and retry in bit-bang mode. Writes (channel hops)
         * still work through the same path; reads just become slow. */
        if (s_dev.spi) { spi_bus_remove_device(s_dev.spi); s_dev.spi = NULL; }
        s_dev.bb_mode = true;
        s_dev.bb_settle_us = 500;
        nrf24_bb_setup_pins(&s_dev);
        connected = nrf24_check_connected(&s_dev);
        if (connected) {
            ESP_LOGW(TAG, "nRF24 detected via BIT-BANG fallback (MISO line is slow; "
                          "hardware SPI can't read it -- running bit-banged at %d us/edge)",
                     s_dev.bb_settle_us);
        }
    }
    if (connected) {
        ESP_LOGI(TAG, "nRF24 detected on SPI%d (CS=%d, CE=%d)%s",
                 (int)NRF24_SPI_HOST, NRF24_CS_PIN, NRF24_CE_PIN,
                 s_dev.bb_mode ? " [bit-bang]" : "");
    } else {
        ESP_LOGW(TAG, "nRF24 not responding (check wiring/power)");
    }
    return connected;
}

/* ---- jam loop (single module, constant carrier) ------------------------- */

/* BLE advertising channels: nRF 2/26/80 = 2402/2426/2480 MHz. */
static const uint8_t ble_adv[3] = {2, 26, 80};

/* Fast constant-carrier sweep over channels [lo, hi]. Hops with no per-hop
 * delay and only blocks (10 ms) after JAM_WDT_FEED_US of continuous sweeping to
 * feed the idle watchdog, so the carrier covers the band almost continuously. */
static void jam_sweep(uint8_t lo, uint8_t hi) {
    nrf24_startConstCarrier(&s_dev, NRF24_TX_POWER, lo);

    int64_t last_feed_us = esp_timer_get_time();
    while (!s_jam_stop) {
        for (uint8_t ch = lo; ch <= hi && !s_jam_stop; ch++) {
            nrf24_write_reg(&s_dev, REG_RF_CH, ch);
            if (esp_timer_get_time() - last_feed_us >= JAM_WDT_FEED_US) {
                vTaskDelay(1);
                last_feed_us = esp_timer_get_time();
            }
        }
    }

    nrf24_stopConstCarrier(&s_dev);
}

/* BLE-focused sweep: weave an advertising channel between every band channel so
 * the three advertising frequencies are hit far more often (every ~3 steps)
 * while still covering the whole BLE band (2..80). */
static void jam_ble(void) {
    nrf24_startConstCarrier(&s_dev, NRF24_TX_POWER, ble_adv[0]);

    int64_t last_feed_us = esp_timer_get_time();
    uint8_t a = 0;
    while (!s_jam_stop) {
        for (uint8_t ch = 2; ch <= 80 && !s_jam_stop; ch++) {
            nrf24_write_reg(&s_dev, REG_RF_CH, ble_adv[a]);
            a = (a + 1) % 3;
            nrf24_write_reg(&s_dev, REG_RF_CH, ch);
            if (esp_timer_get_time() - last_feed_us >= JAM_WDT_FEED_US) {
                vTaskDelay(1);
                last_feed_us = esp_timer_get_time();
            }
        }
    }

    nrf24_stopConstCarrier(&s_dev);
}

static void nrf24_jam_task(void* ctx) {
    (void)ctx;
    s_jam_running = true;

    switch (s_band) {
        case JAM_BLE: jam_ble(); break;           /* BLE, adv channels weighted */
        case JAM_BT: jam_sweep(0, 83); break;     /* classic BT band */
        case JAM_WIFI: jam_sweep(1, 84); break;   /* WiFi 2.4 GHz span */
        case JAM_DRONE:
        case JAM_ALL:
        default:
            jam_sweep(0, 125);                    /* full 2.4 GHz */
            break;
    }

    /* Leave the radio idle. */
    nrf24_set_idle(&s_dev);

    s_jam_running = false;
    s_jam_task = NULL;
    vTaskDelete(NULL);
}

bool nrf24_jammer_start(nrf24_jam_band_t band) {
    if (!s_initialized) {
        ESP_LOGW(TAG, "start refused: run init_nrf24 first");
        return false;
    }
    if (s_jam_running || s_jam_task != NULL) {
        ESP_LOGW(TAG, "start refused: jammer already running");
        return false;
    }

    s_band = band;
    s_jam_stop = false;

    /* Priority 1: just above idle, below the priority-2 console task (and all
     * WiFi/system tasks). The console therefore preempts the jammer the instant
     * a `stop` line arrives, so the sweep can run nearly continuously without
     * frequent voluntary yields. */
    BaseType_t ok = xTaskCreate(nrf24_jam_task, "nrf24_jam", 4096, NULL, 1, &s_jam_task);
    if (ok != pdPASS) {
        s_jam_task = NULL;
        s_jam_stop = true;
        ESP_LOGE(TAG, "failed to create jam task");
        return false;
    }
    ESP_LOGI(TAG, "jammer started (band=%s)", nrf24_jammer_band_name(band));
    return true;
}

void nrf24_jammer_stop(void) {
    if (!s_jam_running && s_jam_task == NULL) return;

    s_jam_stop = true;

    /* Wait for the task to exit cleanly. */
    for (int i = 0; i < 40 && s_jam_task != NULL; i++) {
        vTaskDelay(pdMS_TO_TICKS(25));
    }
    if (s_jam_task != NULL) {
        vTaskDelete(s_jam_task);
        s_jam_task = NULL;
        s_jam_running = false;
        ESP_LOGW(TAG, "jam task force-deleted");
    }

    if (s_initialized) {
        nrf24_stopConstCarrier(&s_dev);
        nrf24_set_idle(&s_dev);
    }
    ESP_LOGI(TAG, "jammer stopped");
}

bool nrf24_jammer_is_running(void) {
    return s_jam_running;
}

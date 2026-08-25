/* CC1101 driver for ESP32-C5 (ESP-IDF). See cc1101.h for wiring/attribution. */
#include "cc1101.h"

#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"   /* esp_rom_delay_us */

/* Calibration tables (FSCTRL0 span) per band, from ELECHOUSE. */
static const uint8_t clb1[2] = {24, 28}; /* 315 */
static const uint8_t clb2[2] = {31, 38}; /* 433 */
static const uint8_t clb3[2] = {65, 76}; /* 868 */
static const uint8_t clb4[2] = {77, 79}; /* 915 */

/* PA tables per band (index by power bucket). OOK uses {0, a}. */
static const uint8_t PA_315[8]  = {0x12,0x0D,0x1C,0x34,0x51,0x85,0xCB,0xC2};
static const uint8_t PA_433[8]  = {0x12,0x0E,0x1D,0x34,0x60,0x84,0xC8,0xC0};
static const uint8_t PA_868[10] = {0x03,0x17,0x1D,0x26,0x37,0x50,0x86,0xCD,0xC5,0xC0};
static const uint8_t PA_915[10] = {0x03,0x0E,0x1E,0x27,0x38,0x8E,0x84,0xCC,0xC3,0xC0};

static long imap(long x, long in_min, long in_max, long out_min, long out_max) {
    if (in_max == in_min) return out_min;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* ---- SPI plumbing (mirrors nrf24.c: hardware CS, polling transfer) ---- */
static void cc1101_trx(cc1101_t *dev, const uint8_t *tx, uint8_t *rx, size_t n) {
    if (!dev || !dev->spi || n == 0) return;
    uint8_t local[65];
    if (n > sizeof(local)) n = sizeof(local);
    spi_transaction_t t = {
        .length   = n * 8,
        .rxlength = n * 8,
        .tx_buffer = tx,
        .rx_buffer = rx ? rx : local,
    };
    spi_device_polling_transmit(dev->spi, &t);
}

void cc1101_write_reg(cc1101_t *dev, uint8_t addr, uint8_t value) {
    uint8_t tx[2] = { addr, value };
    cc1101_trx(dev, tx, NULL, 2);
}

uint8_t cc1101_read_reg(cc1101_t *dev, uint8_t addr) {
    uint8_t tx[2] = { (uint8_t)(addr | 0x80), 0 };
    uint8_t rx[2] = { 0, 0 };
    cc1101_trx(dev, tx, rx, 2);
    return rx[1];
}

uint8_t cc1101_read_status(cc1101_t *dev, uint8_t addr) {
    uint8_t tx[2] = { (uint8_t)(addr | 0xC0), 0 };
    uint8_t rx[2] = { 0, 0 };
    cc1101_trx(dev, tx, rx, 2);
    return rx[1];
}

void cc1101_strobe(cc1101_t *dev, uint8_t strobe) {
    uint8_t tx[1] = { strobe };
    cc1101_trx(dev, tx, NULL, 1);
}

void cc1101_write_burst(cc1101_t *dev, uint8_t addr, const uint8_t *buf, uint8_t n) {
    uint8_t tx[65];
    if (n > 64) n = 64;
    tx[0] = (uint8_t)(addr | 0x40);
    memcpy(&tx[1], buf, n);
    cc1101_trx(dev, tx, NULL, n + 1);
}

void cc1101_read_burst(cc1101_t *dev, uint8_t addr, uint8_t *buf, uint8_t n) {
    uint8_t tx[65];
    uint8_t rx[65];
    if (n > 64) n = 64;
    memset(tx, 0, n + 1);
    tx[0] = (uint8_t)(addr | 0xC0);
    cc1101_trx(dev, tx, rx, n + 1);
    memcpy(buf, &rx[1], n);
}

uint8_t cc1101_version(cc1101_t *dev) { return cc1101_read_status(dev, CC1101_VERSION); }
bool cc1101_present(cc1101_t *dev) {
    uint8_t v = cc1101_version(dev);
    return (v != 0x00 && v != 0xFF);
}

/* ---- Frequency + calibration ---- */
void cc1101_set_frequency(cc1101_t *dev, float mhz) {
    dev->mhz = mhz;
    uint32_t w = (uint32_t)lroundf((mhz * 65536.0f) / 26.0f);
    cc1101_write_reg(dev, CC1101_FREQ2, (uint8_t)(w >> 16));
    cc1101_write_reg(dev, CC1101_FREQ1, (uint8_t)(w >> 8));
    cc1101_write_reg(dev, CC1101_FREQ0, (uint8_t)(w));

    /* Per-band FSCTRL0 + TEST0/FSCAL2 fixup (ELECHOUSE Calibrate). */
    if (mhz >= 280 && mhz <= 348) {
        cc1101_write_reg(dev, CC1101_FSCTRL0, (uint8_t)imap((long)mhz, 280, 348, clb1[0], clb1[1]));
        if (mhz < 322.88f) { cc1101_write_reg(dev, CC1101_TEST0, 0x0B); }
        else { cc1101_write_reg(dev, CC1101_TEST0, 0x09);
               uint8_t s = cc1101_read_status(dev, CC1101_FSCAL2);
               if (s < 32) cc1101_write_reg(dev, CC1101_FSCAL2, s + 32); }
    } else if (mhz >= 378 && mhz <= 464) {
        cc1101_write_reg(dev, CC1101_FSCTRL0, (uint8_t)imap((long)mhz, 378, 464, clb2[0], clb2[1]));
        if (mhz < 430.5f) { cc1101_write_reg(dev, CC1101_TEST0, 0x0B); }
        else { cc1101_write_reg(dev, CC1101_TEST0, 0x09);
               uint8_t s = cc1101_read_status(dev, CC1101_FSCAL2);
               if (s < 32) cc1101_write_reg(dev, CC1101_FSCAL2, s + 32); }
    } else if (mhz >= 779 && mhz <= 899.99f) {
        cc1101_write_reg(dev, CC1101_FSCTRL0, (uint8_t)imap((long)mhz, 779, 899, clb3[0], clb3[1]));
        if (mhz < 861) { cc1101_write_reg(dev, CC1101_TEST0, 0x0B); }
        else { cc1101_write_reg(dev, CC1101_TEST0, 0x09);
               uint8_t s = cc1101_read_status(dev, CC1101_FSCAL2);
               if (s < 32) cc1101_write_reg(dev, CC1101_FSCAL2, s + 32); }
    } else if (mhz >= 900 && mhz <= 928) {
        cc1101_write_reg(dev, CC1101_FSCTRL0, (uint8_t)imap((long)mhz, 900, 928, clb4[0], clb4[1]));
        cc1101_write_reg(dev, CC1101_TEST0, 0x09);
        uint8_t s = cc1101_read_status(dev, CC1101_FSCAL2);
        if (s < 32) cc1101_write_reg(dev, CC1101_FSCAL2, s + 32);
    }
    cc1101_set_pa(dev, dev->pa_dbm);  /* PA table is band-dependent */
}

void cc1101_set_pa(cc1101_t *dev, int dbm) {
    dev->pa_dbm = dbm;
    uint8_t a = 0xC0;
    float m = dev->mhz;
    if (m >= 280 && m <= 348) {
        if (dbm <= -30) a = PA_315[0]; else if (dbm <= -20) a = PA_315[1];
        else if (dbm <= -15) a = PA_315[2]; else if (dbm <= -10) a = PA_315[3];
        else if (dbm <= 0) a = PA_315[4]; else if (dbm <= 5) a = PA_315[5];
        else if (dbm <= 7) a = PA_315[6]; else a = PA_315[7];
    } else if (m >= 378 && m <= 464) {
        if (dbm <= -30) a = PA_433[0]; else if (dbm <= -20) a = PA_433[1];
        else if (dbm <= -15) a = PA_433[2]; else if (dbm <= -10) a = PA_433[3];
        else if (dbm <= 0) a = PA_433[4]; else if (dbm <= 5) a = PA_433[5];
        else if (dbm <= 7) a = PA_433[6]; else a = PA_433[7];
    } else if (m >= 779 && m <= 899.99f) {
        if (dbm <= -30) a = PA_868[0]; else if (dbm <= -20) a = PA_868[1];
        else if (dbm <= -15) a = PA_868[2]; else if (dbm <= -10) a = PA_868[3];
        else if (dbm <= -6) a = PA_868[4]; else if (dbm <= 0) a = PA_868[5];
        else if (dbm <= 5) a = PA_868[6]; else if (dbm <= 7) a = PA_868[7];
        else if (dbm <= 10) a = PA_868[8]; else a = PA_868[9];
    } else if (m >= 900 && m <= 928) {
        if (dbm <= -30) a = PA_915[0]; else if (dbm <= -20) a = PA_915[1];
        else if (dbm <= -15) a = PA_915[2]; else if (dbm <= -10) a = PA_915[3];
        else if (dbm <= -6) a = PA_915[4]; else if (dbm <= 0) a = PA_915[5];
        else if (dbm <= 5) a = PA_915[6]; else if (dbm <= 7) a = PA_915[7];
        else if (dbm <= 10) a = PA_915[8]; else a = PA_915[9];
    }
    uint8_t pa_table[8] = {0};
    if (dev->modulation == CC1101_MOD_ASK) { pa_table[0] = 0; pa_table[1] = a; }
    else { pa_table[0] = a; pa_table[1] = 0; }
    cc1101_write_burst(dev, CC1101_PATABLE, pa_table, 8);
}

void cc1101_set_modulation(cc1101_t *dev, uint8_t mod) {
    if (mod > 4) mod = 4;
    dev->modulation = mod;
    uint8_t modfm = 0x00, frend0 = 0x10;
    switch (mod) {
        case CC1101_MOD_2FSK: modfm = 0x00; frend0 = 0x10; break;
        case CC1101_MOD_GFSK: modfm = 0x10; frend0 = 0x10; break;
        case CC1101_MOD_ASK:  modfm = 0x30; frend0 = 0x11; break;
        case CC1101_MOD_4FSK: modfm = 0x40; frend0 = 0x10; break;
        case CC1101_MOD_MSK:  modfm = 0x70; frend0 = 0x10; break;
    }
    /* Preserve DC filter / manchester / sync bits as 0 (no sync) -> raw friendly. */
    cc1101_write_reg(dev, CC1101_MDMCFG2, modfm);
    cc1101_write_reg(dev, CC1101_FREND0, frend0);
    cc1101_set_pa(dev, dev->pa_dbm);
}

void cc1101_set_ccmode(cc1101_t *dev, bool packet_mode) {
    if (packet_mode) {
        cc1101_write_reg(dev, CC1101_IOCFG2, 0x0B);
        cc1101_write_reg(dev, CC1101_IOCFG0, 0x06);
        cc1101_write_reg(dev, CC1101_PKTCTRL0, 0x05);
        cc1101_write_reg(dev, CC1101_MDMCFG3, 0xF8);
        cc1101_write_reg(dev, CC1101_MDMCFG4, 0x07 + 0xB0); /* base + default rxbw */
    } else {
        /* Async serial: GDO0 carries demodulated data (RX) / drives modulator (TX). */
        cc1101_write_reg(dev, CC1101_IOCFG2, 0x0D);
        cc1101_write_reg(dev, CC1101_IOCFG0, 0x0D);
        cc1101_write_reg(dev, CC1101_PKTCTRL0, 0x32);
        cc1101_write_reg(dev, CC1101_MDMCFG3, 0x93);
        cc1101_write_reg(dev, CC1101_MDMCFG4, 0x07 + 0xB0);
    }
    cc1101_set_modulation(dev, dev->modulation);
}

/* RxBW: sets MDMCFG4[7:4]; keeps drate nibble at a low default (0x07). */
void cc1101_set_rxbw(cc1101_t *dev, float khz) {
    int s1 = 3, s2 = 3;
    float f = khz;
    for (int i = 0; i < 3; i++) { if (f > 101.5625f) { f /= 2; s1--; } else break; }
    for (int i = 0; i < 3; i++) { if (f > 58.1f) { f /= 1.25f; s2--; } else break; }
    uint8_t m4 = (uint8_t)((s1 * 64) + (s2 * 16));
    uint8_t drate_nibble = cc1101_read_reg(dev, CC1101_MDMCFG4) & 0x0F;
    cc1101_write_reg(dev, CC1101_MDMCFG4, m4 | drate_nibble);
}

void cc1101_set_drate(cc1101_t *dev, float kbaud) {
    float c = kbaud;
    uint8_t mdmcfg3 = 0, e = 0;
    if (c > 1621.83f) c = 1621.83f;
    if (c < 0.0247955f) c = 0.0247955f;
    for (int i = 0; i < 20; i++) {
        if (c <= 0.0494942f) {
            c = (c - 0.0247955f) / 0.00009685f;
            mdmcfg3 = (uint8_t)c;
            if ((c - mdmcfg3) * 10 >= 5) mdmcfg3++;
            break;
        } else { e++; c /= 2; }
    }
    uint8_t rxbw_nibble = cc1101_read_reg(dev, CC1101_MDMCFG4) & 0xF0;
    cc1101_write_reg(dev, CC1101_MDMCFG4, rxbw_nibble | (e & 0x0F));
    cc1101_write_reg(dev, CC1101_MDMCFG3, mdmcfg3);
}

void cc1101_set_deviation(cc1101_t *dev, float khz) {
    /* DEVIATN = (E<<4)|M, reserved bits 7 and 3 stay 0.
     * dev = (Fxosc/2^17) * (8+M) * 2^E. Pick the minimal E/M that covers khz. */
    const float base = 26000000.0f / 131072.0f / 1000.0f; /* ~0.198364 kHz/step */
    if (khz > 380.859375f) khz = 380.859375f;
    if (khz < 1.586914f)  khz = 1.586914f;
    uint8_t reg = 0x77; /* fallback: max E=7,M=7 */
    for (int e = 0; e < 8; e++) {
        bool done = false;
        for (int m = 0; m < 8; m++) {
            float dev_kHz = base * (float)(8 + m) * (float)(1 << e);
            if (dev_kHz >= khz) { reg = (uint8_t)((e << 4) | m); done = true; break; }
        }
        if (done) break;
    }
    cc1101_write_reg(dev, CC1101_DEVIATN, reg);
}

void cc1101_set_rx(cc1101_t *dev)    { cc1101_strobe(dev, CC1101_SIDLE); cc1101_strobe(dev, CC1101_SRX); }
void cc1101_set_tx(cc1101_t *dev)    { cc1101_strobe(dev, CC1101_SIDLE); cc1101_strobe(dev, CC1101_STX); }
void cc1101_set_idle(cc1101_t *dev)  { cc1101_strobe(dev, CC1101_SIDLE); }
void cc1101_flush_rx(cc1101_t *dev)  { cc1101_strobe(dev, CC1101_SIDLE); cc1101_strobe(dev, CC1101_SFRX); }
void cc1101_flush_tx(cc1101_t *dev)  { cc1101_strobe(dev, CC1101_SIDLE); cc1101_strobe(dev, CC1101_SFTX); }

int cc1101_get_rssi(cc1101_t *dev) {
    int raw = cc1101_read_status(dev, CC1101_RSSI);
    if (raw >= 128) return (raw - 256) / 2 - 74;
    return raw / 2 - 74;
}

/* ---- Baseline register block (ELECHOUSE RegConfigSettings) ---- */
void cc1101_config_base(cc1101_t *dev) {
    cc1101_write_reg(dev, CC1101_FSCTRL1, 0x06);
    cc1101_set_ccmode(dev, false);          /* async serial (raw) by default */
    cc1101_set_frequency(dev, dev->mhz);
    cc1101_write_reg(dev, CC1101_MDMCFG1, 0x02);
    cc1101_write_reg(dev, CC1101_MDMCFG0, 0xF8);
    cc1101_write_reg(dev, CC1101_CHANNR,  0x00);
    cc1101_write_reg(dev, CC1101_DEVIATN, 0x47);
    cc1101_write_reg(dev, CC1101_FREND1,  0x56);
    cc1101_write_reg(dev, CC1101_MCSM0,   0x18);
    cc1101_write_reg(dev, CC1101_FOCCFG,  0x16);
    cc1101_write_reg(dev, CC1101_BSCFG,   0x1C);
    cc1101_write_reg(dev, CC1101_AGCCTRL2,0xC7);
    cc1101_write_reg(dev, CC1101_AGCCTRL1,0x00);
    cc1101_write_reg(dev, CC1101_AGCCTRL0,0xB2);
    cc1101_write_reg(dev, CC1101_FSCAL3,  0xE9);
    cc1101_write_reg(dev, CC1101_FSCAL2,  0x2A);
    cc1101_write_reg(dev, CC1101_FSCAL1,  0x00);
    cc1101_write_reg(dev, CC1101_FSCAL0,  0x1F);
    cc1101_write_reg(dev, CC1101_FSTEST,  0x59);
    cc1101_write_reg(dev, CC1101_TEST2,   0x81);
    cc1101_write_reg(dev, CC1101_TEST1,   0x35);
    cc1101_write_reg(dev, CC1101_TEST0,   0x09);
    cc1101_write_reg(dev, CC1101_PKTCTRL1,0x04);
    cc1101_write_reg(dev, CC1101_ADDR,    0x00);
    cc1101_write_reg(dev, CC1101_PKTLEN,  0x00);
    cc1101_set_rxbw(dev, 232.0f);           /* wide OOK RX bandwidth */
    cc1101_set_drate(dev, 5.0f);            /* low data rate for raw OOK */
}

void cc1101_default_config(cc1101_t *dev) {
    memset(dev, 0, sizeof(*dev));
    dev->host = CC1101_SPI_HOST;
    dev->cs_pin = CC1101_PIN_CS;
    dev->gdo0_pin = CC1101_PIN_GDO0;
    dev->gdo2_pin = CC1101_PIN_GDO2;
    dev->sck_pin = CC1101_PIN_SCK;
    dev->miso_pin = CC1101_PIN_MISO;
    dev->mosi_pin = CC1101_PIN_MOSI;
    dev->mhz = 433.92f;
    dev->modulation = CC1101_MOD_ASK;
    dev->pa_dbm = 10;
}

bool cc1101_init(cc1101_t *dev) {
    if (dev->initialized) return cc1101_present(dev);

    /* GDO0 as input (RX capture). GDO2 configured by higher layers as needed. */
    gpio_config_t gdo0 = {
        .pin_bit_mask = (1ULL << dev->gdo0_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&gdo0);

    /* SD may already own SPI2. INVALID_STATE = bus already up (fine). */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = dev->mosi_pin,
        .miso_io_num = dev->miso_pin,
        .sclk_io_num = dev->sck_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 128,
    };
    esp_err_t ret = spi_bus_initialize(dev->host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return false;

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = CC1101_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = dev->cs_pin,
        .queue_size = 2,
        .command_bits = 0,
        .address_bits = 0,
    };
    ret = spi_bus_add_device(dev->host, &dev_cfg, &dev->spi);
    if (ret != ESP_OK) return false;

    /* Reset sequence. */
    cc1101_strobe(dev, CC1101_SRES);
    esp_rom_delay_us(1000);
    cc1101_strobe(dev, CC1101_SNOP);

    if (!cc1101_present(dev)) {
        spi_bus_remove_device(dev->spi);
        dev->spi = NULL;
        return false;
    }

    dev->modulation = CC1101_MOD_ASK;
    cc1101_config_base(dev);
    cc1101_set_idle(dev);
    dev->initialized = true;
    return true;
}

void cc1101_deinit(cc1101_t *dev) {
    if (!dev->initialized) return;
    cc1101_set_idle(dev);
    if (dev->spi) { spi_bus_remove_device(dev->spi); dev->spi = NULL; }
    dev->initialized = false;
}

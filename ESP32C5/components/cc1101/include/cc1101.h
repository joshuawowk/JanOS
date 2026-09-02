/*
 * CC1101 sub-GHz transceiver driver for ESP32-C5 (ESP-IDF native SPI master).
 *
 * Ported from the ELECHOUSE SmartRC-CC1101 register recipe and the SentinelRF
 * native register-level bring-up. Designed to share SPI2_HOST with the SD card
 * (SD owns the bus; this attaches as a second device with its own CS), exactly
 * like the nRF24 driver in this project.
 *
 * Default wiring: the CC1101 shares ONE swappable radio header with the nRF24
 * jammer on the SD SPI2 bus, so a CC1101 (sub-GHz) or an nRF24 (2.4 GHz) can be
 * socketed interchangeably -- one at a time -- and firmware auto-detects which is
 * present. The CC1101 CS/GDO0 now sit on the exact pins the nRF24 uses for
 * CSN/CE, and the (unused) GDO2 is dropped:
 *   SCK=GPIO6  MOSI=GPIO7  MISO=GPIO2   (already the SD bus)
 *   CS =GPIO3  GDO0=GPIO4               (== nRF24 CSN/CE -- the shared header)
 *   GDO2 disabled (-1)                  (was GPIO5; never used by firmware)
 * Boot-safe: GPIO3 is a strapping pin but its only strap role is an unused SDIO
 * clock-edge bit; GPIO4/MTCK is not a strapping pin (boot mode = GPIO26/27/28).
 * The shared header needs a 10k external pull-up on GPIO3 (no internal pull on
 * MTMS/MTDI) and a 4.7k external pull-down on GPIO4 (to beat its ~45k internal
 * weak pull-up). GPIO15-22 are the in-package flash/PSRAM (MSPI) bus -- never
 * use them. GPIO11/12 are the console UART.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Default pin map (override before cc1101_init if your wiring differs) ---- */
#ifndef CC1101_PIN_SCK
#define CC1101_PIN_SCK   6
#endif
#ifndef CC1101_PIN_MISO
#define CC1101_PIN_MISO  2
#endif
#ifndef CC1101_PIN_MOSI
#define CC1101_PIN_MOSI  7
#endif
#ifndef CC1101_PIN_CS
#define CC1101_PIN_CS    3     /* shared with nRF24 CSN (swappable radio header) */
#endif
#ifndef CC1101_PIN_GDO0
#define CC1101_PIN_GDO0  4     /* shared with nRF24 CE  (swappable radio header) */
#endif
#ifndef CC1101_PIN_GDO2
#define CC1101_PIN_GDO2  (-1)  /* dropped: unused by firmware, frees GPIO5 */
#endif
#ifndef CC1101_SPI_HOST
#define CC1101_SPI_HOST  SPI2_HOST
#endif
#ifndef CC1101_SPI_CLOCK_HZ
#define CC1101_SPI_CLOCK_HZ (6 * 1000 * 1000)   /* CC1101 max ~6.5 MHz */
#endif

/* ---- Modulation formats (MDMCFG2 MOD_FORMAT) ---- */
#define CC1101_MOD_2FSK  0
#define CC1101_MOD_GFSK  1
#define CC1101_MOD_ASK   2   /* ASK/OOK */
#define CC1101_MOD_4FSK  3
#define CC1101_MOD_MSK   4

/* ---- Command strobes ---- */
#define CC1101_SRES   0x30
#define CC1101_SFSTXON 0x31
#define CC1101_SXOFF  0x32
#define CC1101_SCAL   0x33
#define CC1101_SRX    0x34
#define CC1101_STX    0x35
#define CC1101_SIDLE  0x36
#define CC1101_SFRX   0x3A
#define CC1101_SFTX   0x3B
#define CC1101_SNOP   0x3D

/* ---- Config registers ---- */
#define CC1101_IOCFG2   0x00
#define CC1101_IOCFG0   0x02
#define CC1101_FIFOTHR  0x03
#define CC1101_SYNC1    0x04
#define CC1101_SYNC0    0x05
#define CC1101_PKTLEN   0x06
#define CC1101_PKTCTRL1 0x07
#define CC1101_PKTCTRL0 0x08
#define CC1101_ADDR     0x09
#define CC1101_CHANNR   0x0A
#define CC1101_FSCTRL1  0x0B
#define CC1101_FSCTRL0  0x0C
#define CC1101_FREQ2    0x0D
#define CC1101_FREQ1    0x0E
#define CC1101_FREQ0    0x0F
#define CC1101_MDMCFG4  0x10
#define CC1101_MDMCFG3  0x11
#define CC1101_MDMCFG2  0x12
#define CC1101_MDMCFG1  0x13
#define CC1101_MDMCFG0  0x14
#define CC1101_DEVIATN  0x15
#define CC1101_MCSM0    0x18
#define CC1101_FOCCFG   0x19
#define CC1101_BSCFG    0x1A
#define CC1101_AGCCTRL2 0x1B
#define CC1101_AGCCTRL1 0x1C
#define CC1101_AGCCTRL0 0x1D
#define CC1101_FREND1   0x21
#define CC1101_FREND0   0x22
#define CC1101_FSCAL3   0x23
#define CC1101_FSCAL2   0x24
#define CC1101_FSCAL1   0x25
#define CC1101_FSCAL0   0x26
#define CC1101_FSTEST   0x29
#define CC1101_TEST2    0x2C
#define CC1101_TEST1    0x2D
#define CC1101_TEST0    0x2E
#define CC1101_PATABLE  0x3E
#define CC1101_TXFIFO   0x3F
#define CC1101_RXFIFO   0x3F

/* ---- Status registers (need burst bit 0xC0) ---- */
#define CC1101_PARTNUM   0x30
#define CC1101_VERSION   0x31
#define CC1101_RSSI      0x34
#define CC1101_MARCSTATE 0x35
#define CC1101_PKTSTATUS 0x38
#define CC1101_RXBYTES   0x3B

typedef struct {
    spi_host_device_t host;
    int cs_pin;
    int gdo0_pin;
    int gdo2_pin;
    int sck_pin;
    int miso_pin;
    int mosi_pin;
    spi_device_handle_t spi;
    bool initialized;
    float mhz;          /* current programmed carrier (MHz), pre-correction */
    uint8_t modulation; /* CC1101_MOD_* */
    int pa_dbm;         /* requested output power */
    uint8_t last_pa_band;
} cc1101_t;

/* Fill a device struct with the compile-time default pin map. */
void cc1101_default_config(cc1101_t *dev);

/* Attach to SPI2, reset, load the OOK baseline, and probe presence.
 * Returns true only if a CC1101 answered (VERSION reg nonzero and != 0xFF). */
bool cc1101_init(cc1101_t *dev);
void cc1101_deinit(cc1101_t *dev);

/* Presence re-check (reads VERSION). */
bool cc1101_present(cc1101_t *dev);
uint8_t cc1101_version(cc1101_t *dev);

/* Low-level register access. */
void    cc1101_write_reg(cc1101_t *dev, uint8_t addr, uint8_t value);
uint8_t cc1101_read_reg(cc1101_t *dev, uint8_t addr);
uint8_t cc1101_read_status(cc1101_t *dev, uint8_t addr);
void    cc1101_strobe(cc1101_t *dev, uint8_t strobe);
void    cc1101_write_burst(cc1101_t *dev, uint8_t addr, const uint8_t *buf, uint8_t n);
void    cc1101_read_burst(cc1101_t *dev, uint8_t addr, uint8_t *buf, uint8_t n);

/* Configuration. */
void cc1101_config_base(cc1101_t *dev);                 /* RegConfigSettings baseline */
void cc1101_set_frequency(cc1101_t *dev, float mhz);    /* includes per-band calibrate */
void cc1101_set_modulation(cc1101_t *dev, uint8_t mod);
void cc1101_set_ccmode(cc1101_t *dev, bool packet_mode);/* false = async serial (raw) */
void cc1101_set_rxbw(cc1101_t *dev, float khz);
void cc1101_set_drate(cc1101_t *dev, float kbaud);
void cc1101_set_deviation(cc1101_t *dev, float khz);
void cc1101_set_pa(cc1101_t *dev, int dbm);

/* State control. */
void cc1101_set_rx(cc1101_t *dev);
void cc1101_set_tx(cc1101_t *dev);
void cc1101_set_idle(cc1101_t *dev);
void cc1101_flush_rx(cc1101_t *dev);
void cc1101_flush_tx(cc1101_t *dev);

/* Telemetry. */
int cc1101_get_rssi(cc1101_t *dev);      /* dBm */

#ifdef __cplusplus
}
#endif

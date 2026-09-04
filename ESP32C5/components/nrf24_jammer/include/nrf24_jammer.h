#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    JAM_BLE = 0,   /* BLE advertising channels (2/26/80) */
    JAM_BT,        /* Bluetooth channel list, constant carrier */
    JAM_WIFI,      /* sweep across all WiFi channels, packet spam */
    JAM_DRONE,     /* constant-carrier sweep 0..125 */
    JAM_ALL,       /* full 2.4GHz constant-carrier sweep 0..125 (default) */
} nrf24_jam_band_t;

/* Initialize and probe the single nRF24 module wired to the ESP32-C5.
 * Returns true if the module responds. Safe to call multiple times. */
bool nrf24_jammer_init(void);

/* Start jamming on the given band. Returns false if the module is not
 * initialized/detected, or if a jam is already running. */
bool nrf24_jammer_start(nrf24_jam_band_t band);

/* Stop any active jamming and place the radio in idle. Safe to call when
 * nothing is running. */
void nrf24_jammer_stop(void);

bool nrf24_jammer_is_running(void);

/* Human-readable band name, e.g. for log output. */
const char* nrf24_jammer_band_name(nrf24_jam_band_t band);

/* SPI bus loopback self-test. Sends a known 4-byte pattern full-duplex on the
 * shared radio SPI device and prints what came back on MISO. With MOSI (GPIO7)
 * jumpered directly to MISO (GPIO2) and the nRF24 unplugged, a healthy C5 SPI
 * bus echoes the pattern back verbatim -- isolating a C5/GPIO2 fault from a
 * module/wiring fault. */
void nrf24_jammer_spi_selftest(void);

/* Internal-loopback variant: reroutes the SPI MISO input to sample the MOSI pin
 * via the GPIO matrix, so the peripheral reads back its own output with NO
 * jumper and NO module. Confirms the SPI peripheral + MOSI path are healthy,
 * isolating them from the GPIO2/MISO net. */
void nrf24_jammer_spi_selftest_internal(void);

/* Real-pin loopback at 200 kHz instead of 2 MHz: distinguishes a GPIO2 pad
 * that's too slow/RC-loaded for SPI speed (echoes slow, garbles fast) from a
 * bad jumper contact (garbles at both). */
void nrf24_jammer_spi_selftest_slow(int khz);

/* Self-drive + read one real pad at 2 MHz to classify it clean vs RC-loaded --
 * used to scan for a clean MISO/MOSI pin. Only pass safe, broken-out GPIOs. */
void nrf24_jammer_pad_test(int pin);

/* Re-probe the real nRF24 at a given SPI clock (kHz): full register round-trip +
 * STATUS read, to find the rate the RC-loaded MISO line reads cleanly at. */
void nrf24_jammer_probe_at_khz(int khz);

/* Bit-banged nRF24 register round-trip with a settle delay (us) around each
 * clock edge -- slow enough to read through a heavily capacitively loaded MISO
 * line that hardware SPI (even at its min clock) can't. Tears down hardware SPI. */
void nrf24_jammer_bitbang_probe(int settle_us);

#ifdef __cplusplus
}
#endif

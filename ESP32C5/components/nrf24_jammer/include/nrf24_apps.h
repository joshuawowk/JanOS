/*
 * nRF24L01+ "apps" ported from nRFBox / ESP32-DIV / Bruce / esp32-rf-sword:
 *   - 2.4 GHz channel scanner / spectrum analyzer (RPD carrier-detect sweep)
 *   - Enhanced ShockBurst (ESB) promiscuous sniffer + MouseJack fingerprint
 *   - MouseJack (Logitech Unifying) keystroke injection
 *   - ESB payload replay
 *
 * Single nRF24 on the shared header (CE=GPIO4, CSN=GPIO3, SPI2). The IRQ pin is
 * NOT wired, so every RX path polls STATUS/RPD -- never the IRQ line. All output
 * is byte-exact [NRF_*] tokens the M5MonsterC5-Tab5 app parses. See
 * docs/radio_apps_port.md.
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "nrf24.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Accessors into the jammer's single shared device handle (defined in
 * nrf24_jammer.c) so all nRF24 features drive the same initialized radio. */
nrf24_device_t *nrf24_jammer_device(void);
bool            nrf24_jammer_ready(void);

/* Start the 2.4 GHz scanner/spectrum sweep over channels [lo,hi] (0..125).
 * Emits [NRF_SPECTRUM_START] then one compact [NRF_SPECTRUM] line per pass and
 * a [NRF_SCAN_TOP] peak. Returns false if the radio is absent or busy. */
bool nrf24_apps_scan_start(int lo, int hi);

/* Start the ESB promiscuous sniffer + MouseJack fingerprinter. Emits
 * [NRF_ESB_START], [NRF_ESB] addr=.. ch=.. rate=.. dev=.. for each locked
 * device, and stashes the last raw packet for nrf24_apps_esb_replay(). */
bool nrf24_apps_esb_start(void);

/* Replay the last ESB packet captured by the sniffer (verbatim, 4 bursts).
 * Emits [NRF_ESB_TX] ok=.. or an error token. Returns false if none stored. */
bool nrf24_apps_esb_replay(void);

/* MouseJack Logitech Unifying unencrypted keystroke injection. addr_hex is a
 * 10-hex-digit 5-byte address (MSB first), ch the RF channel, text the ASCII to
 * type. Emits [NRF_MJ_TX] frames=.. Returns false if the radio is absent. */
bool nrf24_apps_mj_inject(const char *addr_hex, int ch, const char *text);

/* Stop any running scanner/sniffer task and idle the radio. Safe any time. */
void nrf24_apps_stop(void);
bool nrf24_apps_is_running(void);

#ifdef __cplusplus
}
#endif

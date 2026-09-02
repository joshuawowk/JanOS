/*
 * SubGHz feature layer for JanOS / ESP32-C5 with a CC1101.
 *
 * Implements the subghz_* console command surface that the M5MonsterC5-Tab5
 * app (ESP32-P4) drives over UART0 at 115200. All command output uses printf()
 * (the console UART the Tab5 parses) with byte-exact [SUBGHZ_*] tokens.
 *
 * Public entry points: call subghz_register_commands() once from the C5's
 * register_commands(), and (optionally) subghz_early_init() from app_main after
 * the SD card is mounted.
 */
#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Register all subghz_* esp_console commands. Call from register_commands(). */
void subghz_register_commands(void);

/* Optional: probe the CC1101 once at boot (safe to skip; commands lazy-init). */
void subghz_early_init(void);

/* Radio arbitration for the shared CC1101/nRF24 header. Only one radio may own
 * SPI2 CS=GPIO3 / GDO0=CE=GPIO4 at a time. When disabled, subghz_ensure_radio()
 * reports the CC1101 absent WITHOUT touching SPI or GPIO4, so the nRF24 backend
 * can own the shared pins. */
void subghz_set_enabled(bool enabled);

/* Fresh probe for a CC1101 on the shared header (reads its VERSION register and
 * removes the SPI device again if absent). Returns true if a CC1101 answered.
 * Used by the boot-time radio auto-detect. */
bool subghz_detect(void);

#ifdef __cplusplus
}
#endif

# SubGHz (CC1101) for JanOS / ESP32-C5

This adds a CC1101 sub-GHz transceiver backend to JanOS so the **M5MonsterC5-Tab5**
app's existing SubGHz screens (Listen, Manage, Jammer, Tesla, Hunter, Scanner,
Weather, RF-Settings) work. The Tab5 (ESP32-P4) is only a UI; all radio work runs
here on the C5 and is driven over the UART0 console (115200) with `subghz_*`
commands, emitting the exact `[SUBGHZ_*]` lines the Tab5 parses.

## Wiring (CC1101 shares the SD card's SPI2 bus)

| CC1101 pin | ESP32-C5 GPIO | Notes |
|---|---|---|
| SCK  | GPIO6  | shared SD SPI2 clock |
| MOSI (SI) | GPIO7  | shared SD SPI2 MOSI |
| MISO (SO) | GPIO2  | shared SD SPI2 MISO |
| CSN  | **GPIO23** | dedicated CC1101 chip-select |
| GDO0 | **GPIO24** | data I/O (RX capture / TX drive), interrupt-capable |
| GDO2 | **GPIO5**  | optional (unused by firmware today) |
| VCC  | 3V3 | 3.3 V only |
| GND  | GND | |

Pins are `#define`s at the top of `components/cc1101/include/cc1101.h`
(`CC1101_PIN_CS/GDO0/GDO2`); change them there if you wire differently.

**Do not use** GPIO15–22 (in-package flash + Quad-PSRAM / MSPI bus), GPIO3/GPIO4
(nRF24 CS/CE), GPIO11/12 (console UART the Tab5 reads), or the strapping pins
2/7/27/28 for anything else. GPIO5/23/24 were chosen because they are free,
non-MSPI, non-strapping. The CC1101 coexists with the SD card and the nRF24 on
SPI2 (each device has its own CS; the IDF driver arbitrates the bus).

## Console commands

| Command | Purpose |
|---|---|
| `subghz_freq <MHz>` | Set carrier (e.g. `subghz_freq 433.92`) |
| `subghz_rx [raw] rssi=<dBm>` | Start receive; decoded or raw. Streams `[SUBGHZ_RSSI]`, `[SUBGHZ_RX]`/`[SUBGHZ_RX_DUP]`/`[SUBGHZ_RAW]` |
| `subghz_stop` | Stop any running operation |
| `subghz_save <idx>` | Save capture `<idx>` to SD (`/sdcard/lab/subghz/*.sub`, Flipper RAW format) |
| `subghz_tx <idx> mem|sd` | Replay a capture from RAM or SD |
| `subghz_tx tesla` | Tesla charge-port opener (uses the current freq; UI sends `subghz_freq 315.00` first) |
| `subghz_list <mem|sd>` | List stored signals |
| `subghz_rename <idx> <name>` / `subghz_delete <idx>` | Manage SD signals |
| `subghz_jam` | Continuous-carrier jam on the current freq |
| `subghz_freq_analyzer <rssi> [hunt timeout=<ms>]` | Strongest-frequency finder / auto-capture hunt |
| `subghz_scanner dwell=<ms> edges=<n> <rssi> [fast]` | Multi-frequency activity scan |
| `subghz_get_freq_correction` / `subghz_set_freq_correction <±MHz>` | Crystal offset (persisted in NVS) |
| `subghz_weather` | Decode weather-sensor telemetry |
| `subghz_status` / `init_cc1101` | Diagnostics |

Presence: the first radio command prints `CC1101 initialized` or
`CC1101 NOT DETECTED` (via the CC1101 VERSION register), which drives the Tab5's
Radio OK/NONE badge.

## Coverage & bench-tuning notes

- **Capture/replay** is protocol-agnostic (Flipper-style RAW via the RMT
  peripheral), so any OOK remote can be captured and replayed even if not decoded.
- **Decoders**: the fixed-code **Princeton / PT2262 / EV1527** family is decoded
  (serial + button). Other OOK remotes are reported as `Unknown` (still saveable
  and replayable). **Rolling-code** remotes (KeeLoq) cannot be cloned — captured
  and reported only.
- **Weather**: the **Nexus-TH** family is decoded; the decoder framework in
  `subghz_decode.c` is structured to add more (Acurite, LaCrosse, etc.).
- All RF timing constants (RX bandwidth, data rate, glitch/idle thresholds,
  calibration bands, TX inter-frame gap) are marked in the source and expect a
  one-time **bench calibration** against real hardware — they are set to sane
  defaults ported from the ELECHOUSE CC1101 driver but were not tuned on a live
  radio here.

## Files

- `components/cc1101/` — native ESP-IDF SPI CC1101 driver.
- `components/subghz/` — feature layer: `subghz.c` (commands + tasks),
  `subghz_capture.c` (RMT RX/TX + edge count), `subghz_decode.c` (OOK + weather),
  `subghz_store.c` (RAM ring + SD `.sub`).
- Wired into `main/main.c` via `subghz_register_commands()` (in
  `register_commands()`), and `main/CMakeLists.txt` REQUIRES.

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
| MOSI (SI) | GPIO8  | shared SD SPI2 MOSI (moved off strapping GPIO7) |
| MISO (SO) | GPIO5  | shared SD SPI2 MISO (moved off strapping GPIO2) |
| CSN  | **GPIO3** | chip-select — shared swappable radio header (== nRF24 CSN) |
| GDO0 | **GPIO4** | data I/O (RX capture / TX drive), interrupt-capable — shared header (== nRF24 CE) |
| GDO2 | disabled (-1) | unused by firmware |
| VCC  | 3V3 | 3.3 V only (add 100nF at VCC-GND, plus ~10µF bulk) |
| GND  | GND | needs a solid common ground with the C5 |

Pins are `#define`s at the top of `components/cc1101/include/cc1101.h`
(`CC1101_PIN_SCK/MOSI/MISO/CS/GDO0/GDO2`); change them there if you wire differently.

The CC1101 shares ONE **swappable radio header** with the nRF24 jammer: CSN/GDO0 sit
on the exact pins the nRF24 uses for CSN/CE (GPIO3/GPIO4), so exactly one of the two
modules is socketed at a time and firmware auto-detects which is present (`radio_detect()`
in `main.c`). Swap the module with power off, then reboot to re-detect.

**Pin history:** MOSI/MISO were moved **GPIO7→GPIO8** and **GPIO2→GPIO5** because GPIO2/7
are ESP32-C5 strapping pins whose boot-stabilizing cap RC-limits them to ~250 kHz (too
slow for SPI). CSN/GDO0 were moved from the old GPIO23/24 onto the shared GPIO3/4 header.
If you wired to an older revision of this table, re-land MOSI on GPIO8, MISO on GPIO5,
CSN on GPIO3, GDO0 on GPIO4.

**Do not use** GPIO15–22 (in-package flash + Quad-PSRAM / MSPI bus), GPIO11/12 (console
UART the Tab5 reads), or the strapping pins 2/7/27/28. GPIO5/6/8 are clean non-strapping
pads; GPIO3 is boot-safe (its only strap role is an unused SDIO bit) and needs a 10k
external pull-up (no internal pull on MTMS); GPIO4 needs a 4.7k external pull-down (to
beat its ~45k internal pull-up). The CC1101 coexists with the SD card on SPI2 (each
device has its own CS; the IDF driver arbitrates the bus).

**Bring-up tip:** if `CC1101 NOT DETECTED`, the VERSION register reads `0xFF` (MISO stuck
high / open) or `0x00` (no data). Reads that vary as `0x66/0x99/0xcc/0x33` are SCK
crosstalk on a floating MISO. A healthy chip reads PARTNUM `0x00` and VERSION `0x14`.

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

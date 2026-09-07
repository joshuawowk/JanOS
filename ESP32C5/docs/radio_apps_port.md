# Radio Apps Port — CC1101 + nRF24 feature expansion

Porting applications/features/functions for the **CC1101 (sub-GHz)** and **nRF24L01+
(2.4 GHz)** from eight reference firmwares into the **JanOS (ESP32-C5)** +
**MonsterC5 (M5Stack Tab5 / ESP32-P4)** pair, within the *swappable radio header*
hardware model documented in the "Swappable Radio Header" guide.

> Scope note: dual-use security-research/education tools (Flipper-class RF multitool
> functions) for **authorized testing and lab/educational use only**. TX features
> (jammers, brute-force, spam) are region-regulated; use on your own equipment/spectrum
> or with authorization.

## Sources reviewed
esp32bruce, bruce-original, ESP32Marauder, ESP32-DIV, nRFBox, ESPnRF24-Jammer,
CC1101_jammer, esp32-rf-sword (all under ~/Repos).

## Hardware model & constraints
- Radios live on the ESP32-C5 (JanOS). MonsterC5/Tab5 is a pure UI host driving the C5
  over one USB console cable, parsing [TOKEN] reply lines.
- One radio at a time, auto-detected at boot by radio_detect(); only the present backend
  owns the shared control pins.
- Shared SPI2 pins (current): SCK=GPIO6 MOSI=GPIO8 MISO=GPIO5 CS/CSN=GPIO3 GDO0/CE=GPIO4.
- Single nRF24 (not the x3/x4 arrays references use) -> wideband becomes channel-hopping.
- nRF24 IRQ NOT connected -> all nRF24 RX by polling STATUS/FIFO, never the IRQ line.
- 3.3 V only.

## Integration contract
JanOS: each feature = an esp_console command (nRF24 in main.c register_commands();
CC1101 in subghz_register_commands()). Long ops run as a FreeRTOS task; emit
line-buffered "[TOKEN] key=val" via printf()+fflush(). `stop`/`subghz_stop` halt them.

MonsterC5: a Pattern B modular screen (main/screens/<feat>_screen.c, added to
main/CMakeLists.txt), a tile in the Radios submenu (show_group_submenu radios branch,
routed in submenu_option_cb), a per-page UART reader task (subghz_host_uart_send/read),
and the portMUX-dirty-flag + lv_timer repaint idiom. Spectrum/waterfall reuse the
lv_canvas primitive from subghz_listen_screen.c (no lv_chart in the project).

## Existing baseline (do not duplicate)
CC1101: subghz_freq/rx/tx/stop/save/list/rename/delete/jam/freq_analyzer/scanner/
weather/status/get|set_freq_correction/init_cc1101; RAW capture + decode + store/replay;
Tesla TX; weather decode.
nRF24: jammer only (start_jammer24 ble|bt|wifi|drone|all) + SPI/pad diagnostics
(spitest, nrf24probe, nrf24bb, gpiotest), init_nrf24, radio.

## Feature port matrix
1  2.4GHz scanner/spectrum        nRF24  nRFBox,rf-sword,DIV  nrf_scan          [NRF_SPECTRUM]/[NRF_SCAN_TOP]  nrf24_scanner(canvas)
2  2.4GHz analyzer (max-hold)     nRF24  nRFBox               nrf_analyze       [NRF_ANALYZER]                reuse scanner
3  jammer bands (ble-adv/full/    nRF24  nRFBox,rf-sword,     start_jammer24    [NRF24]                       jammer(extend)
   wifi<ch>/bt/zigbee/all)               CC1101_jammer,ESPnRF24
4  BLE spam apple/samsung/        nRF24  nRFBox,rf-sword,DIV  nrf_spam          [NRF_SPAM_*]                  nrf24_spam
   google/windows/all
5  ESB/MouseJack scan (polled)    nRF24  DIV,Bruce            nrf_esb_scan      [NRF_ESB_*]                   nrf24_esb
6  MouseJack inject (Logitech)    nRF24  DIV                  nrf_mj_inject     [NRF_MJ_TX]                   esb action
7  ESB replay                     nRF24  Bruce               nrf_esb_replay    [NRF_ESB_TX]                  esb action
8  sub-GHz spectrum/waterfall     CC1101 Bruce,rf-sword      subghz_spectrum   [SUBGHZ_SPECTRUM_*]           subghz spectrum(canvas)
9  sub-GHz de Bruijn brute        CC1101 Bruce,DIV           subghz_brute      [SUBGHZ_BRUTE_*]              subghz brute
10 sub-GHz TX presets (CAME/NICE/ CC1101 Bruce emit.cpp      subghz_tx <preset> [SUBGHZ_TX] preset=          tesla/tx(extend)
   Princeton/Holtek/Chamberlain)
11 sub-GHz jamming detector       CC1101 DIV                 subghz_jamdet     [SUBGHZ_JAMDET]               indicator

Priority (value x independence): 1, 3, 8, 4, 9, 5/6/7, 2, 10, 11.

## Build / flash / test
C5 (JanOS): source ~/.espressif/v6.1/esp-idf/export.sh && idf.py build -> flash the ACM
port whose esptool chip_id reports esp32c5.
P4 (MonsterC5): source ~/esp/esp-idf-v5.4.1/export.sh && idf.py build -> flash the ACM
port reporting esp32p4.
Baseline C5 build verified: projectZero.bin 2.36 MB, 43% app partition free.

## Git
JanOS branch: feat/radio-apps-port (off feat/swappable-cc1101-nrf24).
MonsterC5 branch: feat/radio-apps-port (off feat/tab5-sd-landscape-a164kbd).
Commit only newly added feature files/edits; leave the user's pre-existing uncommitted
changes for them to cut. No pushes to main; no force-push.

## Update: BLE spam is native-BLE (feature #4)
Source review confirmed all reference tools transmit BLE popups with the ESP's own
BLE controller, not the nRF24. JanOS already runs the NimBLE host, so `ble_spam`
(apple|samsung|google|windows|all) broadcasts crafted non-connectable adv PDUs via
NimBLE GAP with a rotating random address. Tokens: `[BLE_SPAM_START] type=`,
`[BLE_SPAM] sent=`, `[BLE_SPAM_STOP] sent=`. Tab5 screen: ble_spam_screen.c.
Tab5 screens added: nrf_scanner, subghz_spectrum, nrf_esb, subghz_brute,
subghz_jamdet, ble_spam (+ radio_waterfall widget).

# Radio Apps — gap-analysis roadmap (remaining portable features)

From an 8-repo gap-analysis (bruce, ESP32-DIV, nRFBox, esp32-rf-sword, ESPnRF24-Jammer,
CC1101_jammer, ESP32Marauder), ranked by value x hardware-feasibility.

## Done in this branch
- [x] CC1101 spectrum/waterfall, de Bruijn/OOK brute, protocol TX presets, jamming detector
- [x] nRF24 RPD scanner/spectrum, ESB promiscuous sniff + MouseJack fingerprint/inject(+last)/replay
- [x] nRF24 jammer bands ble/ble-adv/bt/wifi/zigbee/drone/all
- [x] Native-BLE spam (apple/samsung/google/windows)
- [x] R1: CC1101 band-sweep + keyfob-preset jammer (subghz_jam_sweep / subghz_jam_keyfob)
- [x] R3/R4/R10: native-BLE Detector (AirTag/Find-My/Flock ALPR/Meta/Flipper) -> ble_detect + Tab5 Detectors screen

## Remaining backlog (ranked; not yet ported)
- [ ] R2 (HIGH,M) Extended OOK protocol DECODE registry on RX (CAME/Nice/Holtek/Ansonic/Linear/
      Clemsa/Mastercode/GateTX/PhoenixV2 + RcSwitch_1..12) in subghz_decode.c. C5-only, no UI.
- [ ] R5 (HIGH,L) KeeLoq rolling-code decode + manufacturer keystore (marquee sub-GHz gap). C5-only.
- [ ] R6 (MED,S) nRF24 targeted jam presets: FPV-video {70,75,80}, USB-HID {40,50,60}, single-WiFi-ch. C5 + band button.
- [ ] R7 (MED,S) Selectable nRF24 jam technique (const-carrier vs packet-flood) + sweep pattern (seq/random/coprime).
- [ ] R8 (MED,S) BLE Fox-Hunt RSSI direction-finding (home in on a detected device). native BLE + Tab5.
- [ ] R9 (MED,S) AirTag/Find-My adv-replay spoof (rebroadcast a captured beacon). native BLE.
- [ ] R11 (MED,M) MouseJack expansion: Microsoft/MS-crypt inject + DuckyScript + vuln classification.
- [ ] R12 (LOW,S) 8-slot ESB capture/replay ring buffer.

## Infeasible on this hardware (single radio, no IRQ, shared header)
3-4 radio wideband jammers; dual-radio mirror jam; BLE connect-following; RFID/NFC suite
(no PN532); Classic-BT skimmer detector.

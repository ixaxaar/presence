# presence

A pocket **presence radar** for a Seeed XIAO ESP32-C3 with a round GC9A01 display.
It passively listens to WiFi and Bluetooth LE traffic, tracks signal strength per
device and its rate of change, and draws the result as a radar sweep. Devices whose
signal is steadily rising are flagged as **closing**, and the rim flashes red when
one gets close.

See [docs/radar.txt](docs/radar.txt) for a sketch of the layout.

## What you see

| Element | Meaning |
|---|---|
| Rings | Approximate range: 2, 5, 10, 20 m (log scale) |
| Rotating beam + trail | Cosmetic sweep, 8 s per rotation |
| Green blip | WiFi client |
| Cyan blip | BLE device |
| Red blip with tail | Signal rising (closing) |
| Hollow dim ring | Not heard for 6 s (last known range) |
| Flashing red rim + `! CLOSING x.x m/s` | Closing device inside 8 m, sustained 2.5 s |

**Blip angle is not a bearing.** One antenna cannot measure direction. Each device
gets a stable arbitrary angle; only its distance from the center means anything.

## Hardware

- Seeed XIAO ESP32-C3
- Seeed Round Display for XIAO (GC9A01, 240x240)

Pin map (set in `platformio.ini`): CS=D1(GPIO3), DC=D3(GPIO5), BL=D6(GPIO21),
SCK=D8(GPIO8), MOSI=D10(GPIO10), MISO=D9(GPIO9). SPI at 80 MHz.

## Build and flash

Requires [PlatformIO](https://platformio.org/).

```
pio run -t upload        # builds and flashes /dev/ttyACM0
pio device monitor       # serial log
```

If upload fails with "No serial data received", hold **BOOT**, tap **RESET**, release
BOOT, and upload again.

Serial prints a heartbeat each second (packet counts, fps, worst frame gap) and one
line per tracked device with RSSI, estimated distance, slope in dB/s and state.

## Docs

- [How it works](docs/how-it-works.md): sensing, tracking, alert logic
- [Accuracy and limits](docs/accuracy.md): what it can and cannot see

## Layout

```
platformio.ini     board, libraries, display pin map
src/tracker.h      per-device RSSI smoothing, slope, range, alert logic
src/main.cpp       WiFi sniffer, BLE scanner, radar UI
```

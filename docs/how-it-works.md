# How it works

## Sensing

Everything runs on the ESP32-C3. The host PC is not involved.

**WiFi (promiscuous mode).** The radio hops across channels 1-13, 250 ms each, and
receives management and data frames without associating to any network. Only frames
sent *by stations* are kept:

- probe requests (management subtype 4)
- data frames with the To-DS bit set and From-DS clear

The transmitter address (addr2) and the per-packet RSSI are recorded. Beacons from
access points are deliberately ignored.

**BLE (NimBLE-Arduino).** A continuous passive scan with duplicate reports enabled, so
every advertisement refreshes that device's RSSI.

Both radios share one 2.4 GHz front end; the driver time-slices them.

## Tracking (`src/tracker.h`)

Per device, keyed by source + MAC:

1. **Smoothing:** exponential moving average of RSSI (alpha 0.25) per packet.
2. **History:** the smoothed value is sampled every 500 ms into a ring buffer.
3. **Slope:** least-squares fit over the last 10 s gives dRSSI/dt in dB/s
   (needs at least 6 samples).
4. **Range:** log-distance model `d = 10^((ref - rssi) / (10 n))` with n = 2.4 and
   a 1 m reference of -59 dBm (BLE) or -45 dBm (WiFi). Clamped to 0.3-40 m.
5. **Closing speed:** `v = d * ln(10) / (10 n) * slope`, positive when approaching.
6. **State:** closing if slope > 0.8 dB/s and heard in the last 4 s; receding if
   slope < -0.8 dB/s; otherwise steady.
7. **Alert:** closing continuously for 2.5 s and estimated distance < 8 m.
8. **Lifetime:** shown as a ghost after 6 s unheard, dropped after 90 s.

Thresholds are constants at the top of `tracker.h`.

## Display (`src/main.cpp`)

An 8-bit 240x240 sprite is drawn each frame and pushed to the GC9A01 (about 31 fps).
The sweep angle is derived from `millis()`, so it does not drift with frame time.
USB serial writes use a zero timeout so logging can never stall rendering.

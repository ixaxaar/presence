# Accuracy and limits

## Distance

Distance is derived from RSSI and is only a ballpark. A device sitting still can
vary 4-6 dB, which swings the estimate between, say, 12 and 18 m. Walls, bodies and
multipath dominate. Different devices transmit at different powers, which the single
reference value cannot capture. Read it as near / mid / far.

Calibrating the 1 m reference RSSI per device type improves it.

## Direction

There is none. One antenna measures strength, not bearing. Blip angles are arbitrary.
Rotating the device (even with a compass) would not make them meaningful. Real
bearing needs several nodes or an antenna array.

## What it can and cannot see

- **Access points / routers:** not shown. Beacons are filtered out.
- **WiFi clients:** only when they transmit on the channel currently being listened
  to. Idle phones transmit rarely, so many are missed.
- **5 GHz:** invisible. The ESP32-C3 is 2.4 GHz only.
- **BLE:** only devices that are advertising. Watches, earbuds, trackers and laptops
  are good; many phones advertise rarely.
- **MAC randomization:** phones rotate addresses, so one phone may appear as several
  devices over time, and a rotation resets its history.

Observed in testing: roughly 1 WiFi packet versus 77 BLE packets in 14 s, so in
practice this is mostly a BLE presence detector.

## False alerts

Slope on a stationary device wandered about +/-0.5 dB/s, so the closing threshold is
0.8 dB/s with a 2.5 s hold. A strong signal (above about -70 dBm) that keeps rising
is the most trustworthy indicator.

## Ideas

- Show access points as dim fixed markers
- Accept more WiFi frame types and lock to channels 1/6/11
- Per-device 1 m calibration
- Compass module to rotate the bezel
- Multiple nodes sharing RSSI over ESP-NOW for rough positioning

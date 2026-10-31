#pragma once
#include <Arduino.h>
#include <math.h>

// Per-device RSSI tracking: smoothed level, slope (dB/s) and a rough range.
enum Source : uint8_t { SRC_WIFI = 0, SRC_BLE = 1 };
enum State : uint8_t { ST_STEADY = 0, ST_CLOSING = 1, ST_RECEDING = 2 };

constexpr int MAX_DEV = 32;
constexpr int HIST = 20;                 // samples kept per device
constexpr uint32_t SAMPLE_MS = 500;      // history sample period
constexpr uint32_t WINDOW_MS = 10000;    // slope window
constexpr uint32_t EXPIRE_MS = 90000;    // forget devices unseen this long
constexpr uint32_t STALE_MS = 6000;      // shown as a ghost after this long unseen
constexpr float EMA_ALPHA = 0.25f;
constexpr float SLOPE_CLOSING = 0.8f;    // dB/s
constexpr uint32_t CLOSE_HOLD_MS = 2500; // must keep closing this long before alerting
constexpr float PATH_N = 2.4f;           // path-loss exponent (indoor guess)
constexpr float ALERT_RANGE_M = 8.0f;

struct Dev {
  bool used = false;
  uint64_t key = 0;
  Source src = SRC_WIFI;
  uint8_t mac[6] = {0};
  float ema = 0;
  uint32_t lastSeen = 0, lastSample = 0, firstSeen = 0;
  uint32_t pkts = 0, closeSince = 0;
  uint32_t ht[HIST];
  float hr[HIST];
  uint8_t hn = 0, hhead = 0;
  // derived (filled by update())
  float slope = 0, dist = 0, speed = 0;  // dB/s, m, m/s (+ = closing)
  State state = ST_STEADY;
  bool alert = false, stale = false;
};

class Tracker {
 public:
  Dev dev[MAX_DEV];
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

  static uint64_t makeKey(Source s, const uint8_t *m) {
    uint64_t k = s;
    for (int i = 0; i < 6; i++) k = (k << 8) | m[i];
    return k;
  }

  // Called from radio callbacks (different tasks) — keep it short.
  void seen(Source s, const uint8_t *mac, int rssi) {
    uint64_t key = makeKey(s, mac);
    uint32_t now = millis();
    portENTER_CRITICAL(&mux);
    Dev *d = nullptr, *free_ = nullptr, *oldest = nullptr;
    for (auto &x : dev) {
      if (!x.used) { if (!free_) free_ = &x; continue; }
      if (x.key == key) { d = &x; break; }
      if (!oldest || x.lastSeen < oldest->lastSeen) oldest = &x;
    }
    if (!d) {
      d = free_ ? free_ : oldest;
      *d = Dev();
      d->used = true; d->key = key; d->src = s;
      memcpy(d->mac, mac, 6);
      d->ema = rssi; d->firstSeen = now; d->lastSample = 0;
    } else {
      d->ema += EMA_ALPHA * (rssi - d->ema);
    }
    d->lastSeen = now;
    d->pkts++;
    portEXIT_CRITICAL(&mux);
  }

  // Run from the main loop: sample history, compute slope/range/state, expire.
  void update() {
    uint32_t now = millis();
    portENTER_CRITICAL(&mux);
    for (auto &d : dev) {
      if (!d.used) continue;
      if (now - d.lastSeen > EXPIRE_MS) { d.used = false; continue; }
      if (now - d.lastSample >= SAMPLE_MS) {
        d.lastSample = now;
        d.ht[d.hhead] = now; d.hr[d.hhead] = d.ema;
        d.hhead = (d.hhead + 1) % HIST;
        if (d.hn < HIST) d.hn++;
      }
      computeSlope(d, now);
      float ref = d.src == SRC_BLE ? -59.f : -45.f;  // RSSI at 1 m (rough)
      d.dist = constrain(powf(10.f, (ref - d.ema) / (10.f * PATH_N)), 0.3f, 40.f);
      // dd/dt = -d * ln10/(10n) * dRSSI/dt ; positive speed = approaching
      d.speed = d.dist * 0.2303f / PATH_N * d.slope;
      bool fresh = now - d.lastSeen < 4000;
      d.stale = now - d.lastSeen > STALE_MS;
      if (d.stale) { d.slope = 0; d.speed = 0; }
      if (d.slope > SLOPE_CLOSING && fresh) d.state = ST_CLOSING;
      else if (d.slope < -SLOPE_CLOSING) d.state = ST_RECEDING;
      else d.state = ST_STEADY;
      if (d.state == ST_CLOSING) { if (!d.closeSince) d.closeSince = now; }
      else d.closeSince = 0;
      d.alert = d.state == ST_CLOSING && now - d.closeSince >= CLOSE_HOLD_MS && d.dist < ALERT_RANGE_M;
    }
    portEXIT_CRITICAL(&mux);
  }

  int snapshot(Dev *out) {
    int n = 0;
    portENTER_CRITICAL(&mux);
    for (auto &d : dev) if (d.used) out[n++] = d;
    portEXIT_CRITICAL(&mux);
    return n;
  }

 private:
  static void computeSlope(Dev &d, uint32_t now) {
    // least-squares slope of ema vs time over the window
    double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (int i = 0; i < d.hn; i++) {
      uint32_t t = d.ht[i];
      if (now - t > WINDOW_MS) continue;
      double x = (double)(t - (now - WINDOW_MS)) / 1000.0, y = d.hr[i];
      n++; sx += x; sy += y; sxx += x * x; sxy += x * y;
    }
    double den = n * sxx - sx * sx;
    d.slope = (n >= 6 && den > 1e-6) ? (float)((n * sxy - sx * sy) / den) : 0.f;
  }
};

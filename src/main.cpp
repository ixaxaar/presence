#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <NimBLEDevice.h>
#include <TFT_eSPI.h>
#include "tracker.h"

Tracker tracker;
volatile uint32_t wifiPk = 0, blePk = 0;
TFT_eSPI tft;
TFT_eSprite fb(&tft);
bool haveFb = false;

// ---------- WiFi sniffing ----------
static void wifiCb(void *buf, wifi_promiscuous_pkt_type_t type) {
  auto *p = (wifi_promiscuous_pkt_t *)buf;
  const uint8_t *f = p->payload;
  uint8_t t = f[0] & 0x0C, st = f[0] & 0xF0, fl = f[1];
  const uint8_t *tx = f + 10;  // addr2 = transmitter
  if (tx[0] & 0x01) return;    // group address can't be a transmitter
  bool station = false;
  if (type == WIFI_PKT_MGMT) station = (t == 0x00 && st == 0x40);          // probe request
  else if (type == WIFI_PKT_DATA) station = (t == 0x08 && (fl & 3) == 1);  // to-DS: from a station
  if (station) { wifiPk++; tracker.seen(SRC_WIFI, tx, p->rx_ctrl.rssi); }
}

// ---------- BLE scanning ----------
class BleCb : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice *d) override {
    blePk++;
    tracker.seen(SRC_BLE, d->getAddress().getNative(), d->getRSSI());
  }
};

// ---------- UI: radar ----------
constexpr int CX = 120, CY = 120, R = 117;
constexpr uint32_t SWEEP_MS = 8000;  // one rotation
constexpr float TAU = 6.2831853f;

static uint16_t rgb(float f, uint8_t r, uint8_t g, uint8_t b) {
  f = constrain(f, 0.f, 1.f);
  return tft.color565((uint8_t)(r * f), (uint8_t)(g * f), (uint8_t)(b * f));
}
static float rangeToRadius(float d) {  // log-ish: 0.3m..30m -> 14..R-8 px
  return 14.f + 95.f * logf(1.f + d / 1.2f) / logf(1.f + 30.f / 1.2f);
}
static float slotAngle(uint64_t key) {  // stable per-device angle (NOT a real bearing)
  uint32_t h = (uint32_t)(key ^ (key >> 29)) * 2654435761u;
  return (h % 3600) / 3600.f * TAU;
}
// compass-style: 0 = up, clockwise
static void polar(float a, float r, int &x, int &y) {
  x = CX + (int)lroundf(sinf(a) * r);
  y = CY - (int)lroundf(cosf(a) * r);
}

volatile uint32_t maxGapMs = 0, pushUs = 0, drawUs = 0, frames = 0;
static void draw(Dev *list, int n) {
  uint32_t now = millis();
  bool blink = (now / 250) & 1;
  float sweep = (now % SWEEP_MS) / (float)SWEEP_MS * TAU;
  const Dev *nearest = nullptr;
  const Dev *worst = nullptr;
  bool anyAlert = false;
  for (int i = 0; i < n; i++) {
    if (!nearest || list[i].dist < nearest->dist) nearest = &list[i];
    if (list[i].alert) { anyAlert = true; if (!worst || list[i].dist < worst->dist) worst = &list[i]; }
  }

  fb.fillSprite(TFT_BLACK);

  // sweep trail: fading wedge behind the beam
  const int STEPS = 40;
  const float STEP = 0.028f;  // ~1.6 deg
  for (int i = 0; i < STEPS; i++) {
    float f = 1.f - (float)i / STEPS;
    f = f * f * 0.55f;
    int x1, y1, x2, y2;
    polar(sweep - i * STEP, R, x1, y1);
    polar(sweep - (i + 1) * STEP, R, x2, y2);
    fb.fillTriangle(CX, CY, x1, y1, x2, y2, rgb(f, 0, 255, 70));
  }

  // grid
  uint16_t grid = rgb(1, 0, 90, 28);
  const float ringsM[] = {2, 5, 10, 20};
  for (float m : ringsM) fb.drawCircle(CX, CY, (int)rangeToRadius(m), grid);
  fb.drawFastHLine(CX - R, CY, 2 * R, grid);
  fb.drawFastVLine(CX, CY - R, 2 * R, grid);
  fb.setTextColor(rgb(1, 0, 130, 40));
  fb.setTextDatum(TL_DATUM);
  fb.drawString("2", CX + 2, CY - (int)rangeToRadius(2) + 1, 1);
  fb.drawString("5", CX + 2, CY - (int)rangeToRadius(5) + 1, 1);
  fb.drawString("10", CX + 2, CY - (int)rangeToRadius(10) + 1, 1);

  // rim + bearing ticks
  uint16_t rim = (anyAlert && blink) ? TFT_RED : rgb(1, 0, 200, 60);
  fb.drawCircle(CX, CY, R + 2, rim);
  fb.drawCircle(CX, CY, R + 1, rim);
  for (int deg = 0; deg < 360; deg += 10) {
    float a = deg * TAU / 360.f;
    int x1, y1, x2, y2;
    polar(a, R - (deg % 30 == 0 ? 8 : 4), x1, y1);
    polar(a, R, x2, y2);
    fb.drawLine(x1, y1, x2, y2, rim);
  }

  // beam
  int bx, by;
  polar(sweep, R, bx, by);
  fb.drawLine(CX, CY, bx, by, rgb(1, 120, 255, 150));

  // blips: bright when the beam passes, then decay until next pass
  for (int i = 0; i < n; i++) {
    Dev &d = list[i];
    float a = slotAngle(d.key);
    float behind = fmodf(sweep - a + TAU, TAU) / TAU;  // 0 = just swept
    float f = d.alert ? 1.f : 0.6f + 0.4f * (1.f - behind) * (1.f - behind);  // steady glow, brighter after the beam
    int x, y;
    polar(a, rangeToRadius(d.dist), x, y);
    if (d.stale) {  // not heard lately: dim hollow ghost at last known range
      fb.drawCircle(x, y, 3, rgb(0.5f, 0, 160, 50));
      continue;
    }
    uint16_t col;
    if (d.state == ST_CLOSING) col = rgb(f, 255, 40, 20);
    else if (d.src == SRC_BLE) col = rgb(f, 60, 255, 220);
    else col = rgb(f, 60, 255, 60);
    if (d.state == ST_CLOSING) {  // comet tail on the side it is coming from
      int tx, ty;
      polar(a, rangeToRadius(d.dist) + 6 + fminf(d.speed, 3.f) * 5, tx, ty);
      fb.drawLine(x, y, tx, ty, col);
    }
    int sz = d.alert ? 4 : 2;
    fb.fillCircle(x, y, sz, col);
    if (behind < 0.08f || (d.alert && blink)) fb.drawCircle(x, y, sz + 3, col);
  }
  fb.fillCircle(CX, CY, 2, rgb(1, 200, 255, 200));

  // readouts (top/bottom keep the scope clear)
  fb.setTextDatum(MC_DATUM);
  char s[28];
  if (anyAlert) {
    snprintf(s, sizeof s, "! CLOSING %.1fm/s", fabsf(worst->speed));
    fb.setTextColor(blink ? TFT_RED : TFT_WHITE, TFT_BLACK);
    fb.drawString(s, CX, 34, 2);
  } else {
    fb.setTextColor(rgb(1, 0, 200, 60), TFT_BLACK);
    fb.drawString(n ? "CLEAR" : "SCANNING", CX, 34, 2);
  }
  if (nearest) snprintf(s, sizeof s, "%.1fm  %dd", nearest->dist, n);
  else snprintf(s, sizeof s, "-- m  0d");
  fb.setTextColor(rgb(1, 0, 220, 70), TFT_BLACK);
  fb.drawString(s, CX, 206, 2);

  uint32_t t1 = micros();
  fb.pushSprite(0, 0);
  pushUs = micros() - t1;
}

// ---------- setup / loop ----------
void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);  // never block the render loop when nobody is reading USB serial
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  fb.setColorDepth(8);
  haveFb = fb.createSprite(240, 240) != nullptr;
  Serial.printf("sprite %s, heap %u\n", haveFb ? "ok" : "FAILED", ESP.getFreeHeap());
  if (!haveFb) { tft.drawString("no framebuffer", 40, 110, 2); while (1) delay(1000); }

  NimBLEDevice::init("");
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new BleCb(), true);  // want duplicates: every adv updates RSSI
  scan->setActiveScan(false);
  scan->setInterval(100);
  scan->setWindow(60);
  scan->setMaxResults(0);
  scan->start(0, nullptr, false);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  wifi_promiscuous_filter_t flt = {.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA};
  esp_wifi_set_promiscuous_filter(&flt);
  esp_wifi_set_promiscuous_rx_cb(wifiCb);
  esp_wifi_set_promiscuous(true);
}

void loop() {
  static uint32_t lastHop = 0, lastDraw = 0, lastLog = 0;
  static uint8_t ch = 1;
  uint32_t now = millis();
  if (now - lastHop > 250) {
    lastHop = now;
    ch = ch % 13 + 1;
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  }
  tracker.update();
  {
    static Dev snap[MAX_DEV];
    static int n = 0;
    static uint32_t lastSnap = 0;
    if (now - lastSnap >= 100) { lastSnap = now; n = tracker.snapshot(snap); }
    static uint32_t lastFrame = 0;
    if (lastFrame && now - lastFrame > maxGapMs) maxGapMs = now - lastFrame;
    lastFrame = now;
    uint32_t t0 = micros();
    draw(snap, n);
    drawUs = micros() - t0;
    frames++;
    if (now - lastLog > 1000) {
      lastDraw = now;
      lastLog = now;
      Serial.printf("hb wifi_pk=%u ble_pk=%u devs=%d heap=%u fps=%u draw=%ums push=%ums maxgap=%ums\n", (unsigned)wifiPk, (unsigned)blePk, n, ESP.getFreeHeap(), (unsigned)frames, (unsigned)(drawUs/1000), (unsigned)(pushUs/1000), (unsigned)maxGapMs); frames = 0; maxGapMs = 0;
      for (int i = 0; i < n; i++) {
        Dev &d = snap[i];
        Serial.printf("%s %02x:%02x:%02x:%02x:%02x:%02x rssi=%.0f d=%.1fm slope=%+.2fdB/s v=%+.2fm/s %s%s\n",
                      d.src == SRC_BLE ? "BLE " : "WIFI", d.mac[0], d.mac[1], d.mac[2], d.mac[3], d.mac[4], d.mac[5],
                      d.ema, d.dist, d.slope, d.speed,
                      d.state == ST_CLOSING ? "closing" : d.state == ST_RECEDING ? "receding" : "steady",
                      d.alert ? " ALERT" : "");
      }
    }
  }
}

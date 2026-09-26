// LDS07RR probe for ESP32: measure signal wires and forward them to the PC.
//
// Commands (USB serial, 921600 baud, terminate with Enter):
//   scan [ms] [pu]              measure all probe pins at once: idle level, edges, baud rate
//                               (pu = with pull-up, for open-collector outputs)
//   bridge <rxpin> <baud> [tx] [od]  forward the lidar's UART 1:1 to the PC
//                               (with tx: also PC -> lidar; od=1: tx open-drain; exit = reset)
//   set <pin> 0|1|z             pin low / high / floating (high impedance)
//   pwm <pin> <freq> <duty%>    PWM on a pin, e.g. for a motor enable
//   lidar <rx> <baud> <motorpin> <rpm>
//                               like bridge, plus speed control: reads the rpm from the
//                               Neato packets (0xFA) and adjusts the motor PWM (20 kHz)
//   pins                        show the probe pins
//   wifi <ssid> <password>      store wifi credentials (in NVS, not in the code) and connect
//   wifi off                    erase wifi credentials
//   info                        uptime, reset reason, motor, wifi/IP
//
// On boot it starts by itself: lidar on GPIO32 (115200), motor control on GPIO26 at 300 rpm,
// and (if wifi is configured) it streams the raw lidar data over TCP port 2323 (lds.local).
#include <WiFi.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include "soc/gpio_reg.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

#if CONFIG_IDF_TARGET_ESP32
const int PROBE[] = {25, 26, 27, 32};
#else  // S2 / S3 / C3 / C6
const int PROBE[] = {4, 5, 6, 7};
#endif
const int NPROBE = sizeof(PROBE) / sizeof(PROBE[0]);
const char *LABEL[] = {"brown", "yellow", "white", "orange"};
const uint32_t STD_BAUDS[] = {9600, 19200, 38400, 57600, 115200, 128000, 153600, 230400,
                              256000, 460800, 500000, 921600, 1000000, 1500000};

const char *AUTOSTART = "lidar 32 115200 26 300";
const uint16_t TCP_PORT = 2323;
WiFiServer server(TCP_PORT);
WiFiClient client;
Preferences prefs;
bool mdnsUp = false;

bool bridging = false;
uint64_t driven = 0;  // pins driven by set/pwm: scan leaves them alone
int bridgeTx = -1;
String line;

static inline uint64_t readAll() {
#if SOC_GPIO_PIN_COUNT > 32
  return REG_READ(GPIO_IN_REG) | ((uint64_t)REG_READ(GPIO_IN1_REG) << 32);
#else
  return REG_READ(GPIO_IN_REG);
#endif
}

uint32_t snapBaud(float us) {
  if (us <= 0) return 0;
  float raw = 1e6f / us;
  uint32_t best = 0;
  float err = 1e9;
  for (uint32_t b : STD_BAUDS) {
    float e = fabsf(raw - b) / b;
    if (e < err) { err = e; best = b; }
  }
  return err < 0.12f ? best : (uint32_t)raw;
}

void scan(uint32_t ms, bool pu) {
  for (int i = 0; i < NPROBE; i++)
    if (!(driven >> PROBE[i] & 1)) pinMode(PROBE[i], pu ? INPUT_PULLUP : INPUT);
  delay(5);
  uint32_t hi[NPROBE] = {0}, edges[NPROBE] = {0}, minw[NPROBE], lastc[NPROBE] = {0};
  bool prev[NPROBE];
  uint64_t v = readAll();
  for (int i = 0; i < NPROBE; i++) { minw[i] = UINT32_MAX; prev[i] = (v >> PROBE[i]) & 1; }
  uint32_t samples = 0, t0 = millis();
  while (millis() - t0 < ms) {
    for (int n = 0; n < 4096; n++) {
      uint32_t now = ESP.getCycleCount();
      v = readAll();
      for (int i = 0; i < NPROBE; i++) {
        bool b = (v >> PROBE[i]) & 1;
        hi[i] += b;
        if (b != prev[i]) {
          if (edges[i] && now - lastc[i] < minw[i]) minw[i] = now - lastc[i];
          edges[i]++;
          lastc[i] = now;
          prev[i] = b;
        }
      }
      samples++;
    }
  }
  float mhz = getCpuFrequencyMhz();
  Serial.printf("\nscan %lu ms, %lu samples (%.2f us/sample)\n", ms, samples, ms * 1000.0f / samples);
  Serial.println("pin  wire     high%   edges    shortest_pulse  baud(estimate)");
  for (int i = 0; i < NPROBE; i++) {
    float us = minw[i] == UINT32_MAX ? 0 : minw[i] / mhz;
    Serial.printf("%-4d %-8s %5.1f %9lu  %9.2f us  %lu\n", PROBE[i], LABEL[i],
                  100.0f * hi[i] / samples, edges[i], us, edges[i] > 20 ? snapBaud(us) : 0);
  }
  Serial.println("ok");
}

// ---- speed control (Neato packets: FA idx speedL speedH ... chkL chkH, 22 bytes)
int motorPin = -1;
float targetRpm = 300, duty = 600;  // duty 0..1023
float rpmSum = 0;
int rpmCount = 0;
uint32_t lastCtl = 0, lastPkt = 0;
uint8_t pkt[22];
int pktLen = 0;

bool neatoOk(const uint8_t *p) {
  uint32_t chk = 0;
  for (int i = 0; i < 20; i += 2) chk = (chk << 1) + (p[i] | p[i + 1] << 8);
  chk = ((chk & 0x7FFF) + (chk >> 15)) & 0x7FFF;
  return chk == (uint32_t)(p[20] | p[21] << 8);
}

void feedParser(const uint8_t *buf, size_t n) {
  for (size_t i = 0; i < n; i++) {
    uint8_t b = buf[i];
    if (pktLen == 0 && b != 0xFA) continue;
    if (pktLen == 1 && (b < 0xA0 || b > 0xF9)) { pktLen = b == 0xFA; continue; }
    pkt[pktLen++] = b;
    if (pktLen == 22) {
      if (neatoOk(pkt)) {
        rpmSum += (pkt[2] | pkt[3] << 8) / 64.0f;
        rpmCount++;
        lastPkt = millis();
      }
      pktLen = 0;
    }
  }
}

void motorControl() {
  if (motorPin < 0 || millis() - lastCtl < 100) return;
  lastCtl = millis();
  if (millis() - lastPkt > 500) {
    duty += 20;  // soft start: no measurement packets -> ramp up power slowly (no current spike)
  } else if (rpmCount) {
    float rpm = rpmSum / rpmCount;
    duty += 0.3f * (targetRpm - rpm);  // integral control, ~1 s settling time
  }
  duty = constrain(duty, 0.0f, 1023.0f);
  ledcWrite(motorPin, (uint32_t)duty);
  rpmSum = 0;
  rpmCount = 0;
}

void startBridge(int rx, uint32_t baud, int tx, bool od) {
  Serial1.end();
  Serial1.setRxBufferSize(8192);
  Serial1.begin(baud, SERIAL_8N1, rx, tx);  // tx = -1: listen only
  gpio_pullup_en((gpio_num_t)rx);  // weak pull-up: a floating (sleeping) TX gives no noise bytes
#if CONFIG_IDF_TARGET_ESP32
  // open-drain: only pull low, the high level comes from the pull-up in the module
  if (tx >= 0 && od) GPIO.pin[tx].pad_driver = 1;
#endif
  bridging = true;
  bridgeTx = tx;
}

void handle(String cmd) {
  cmd.trim();
  if (!cmd.length()) return;
  char c[128];
  cmd.toCharArray(c, sizeof(c));
  int a, b, d;
  char z;
  unsigned long ms;
  if (cmd.startsWith("scan")) {
    scan(sscanf(c, "scan %lu", &ms) == 1 ? ms : 1500, cmd.endsWith("pu"));
  } else if (sscanf(c, "bridge %d %d", &a, &b) == 2) {
    int od = 0;
    if (sscanf(c, "bridge %d %d %d %d", &a, &b, &d, &od) < 3) d = -1;
    Serial.printf("bridge rx=%d baud=%d tx=%d od=%d\n", a, b, d, od);
    Serial.flush();
    startBridge(a, b, d, od);
  } else if (sscanf(c, "set %d %c", &a, &z) == 2) {
    if (a == motorPin) motorPin = -1;  // manual takeover: control off
    ledcDetach(a);
    if (z == 'z') { pinMode(a, INPUT); driven &= ~(1ULL << a); }
    else { pinMode(a, OUTPUT); digitalWrite(a, z == '1'); driven |= 1ULL << a; }
    Serial.printf("pin %d = %c\nok\n", a, z);
  } else if (sscanf(c, "pwm %d %d %d", &a, &b, &d) == 3) {
    if (a == motorPin) motorPin = -1;
    ledcDetach(a);
    int bits = 10;  // lower resolution at high frequencies (80 MHz / 2^bits >= freq)
    while (bits > 2 && (80000000UL >> bits) < (unsigned long)b) bits--;
    ledcAttach(a, b, bits);
    ledcWrite(a, d * ((1 << bits) - 1) / 100);
    driven |= 1ULL << a;
    Serial.printf("pwm pin %d %d Hz %d%%\nok\n", a, b, d);
  } else if (sscanf(c, "lidar %d %d %d %lu", &a, &b, &d, &ms) == 4) {
    Serial.printf("bridge rx=%d baud=%d motor=%d rpm=%lu\n", a, b, d, ms);
    Serial.flush();
    targetRpm = ms;
    if (motorPin != d) {  // control already running on this pin: don't restart the motor
      ledcDetach(d);
      ledcAttach(d, 20000, 10);
      motorPin = d;
      duty = 350;  // ~35%: start low, motor on VIN (5 V) + soft start
      ledcWrite(d, (uint32_t)duty);
      driven |= 1ULL << d;
      lastPkt = 0;
    }
    startBridge(a, b, -1, false);
  } else if (cmd == "wifi off") {
    prefs.putString("ssid", "");
    prefs.putString("pass", "");
    WiFi.disconnect(true);
    Serial.println("wifi erased\nok");
  } else if (cmd.startsWith("wifi ")) {
    // last word = password, everything in between = network name (may contain spaces)
    String rest = cmd.substring(5);
    int sp = rest.lastIndexOf(' ');
    if (sp < 1) {
      Serial.println("usage: wifi <network name> <password>\nok");
      return;
    }
    prefs.putString("ssid", rest.substring(0, sp));
    prefs.putString("pass", rest.substring(sp + 1));
    WiFi.disconnect();
    WiFi.begin(rest.substring(0, sp).c_str(), rest.substring(sp + 1).c_str());
    Serial.printf("wifi saved, connecting to '%s'...\nok\n", rest.substring(0, sp).c_str());
  } else if (cmd == "info") {
    const char *why[] = {"unknown", "power-on", "external", "software", "panic", "int-wdt", "task-wdt",
                         "wdt", "deepsleep", "BROWNOUT", "sdio"};
    int r = esp_reset_reason();
    Serial.printf("uptime %lu s, last reset: %s, motor pin %d, duty %.0f, target %.0f rpm\n",
                  millis() / 1000, r < 11 ? why[r] : "?", motorPin, duty, targetRpm);
    if (WiFi.status() == WL_CONNECTED)
      Serial.printf("wifi: connected to '%s', IP %s, signal %d dBm, TCP port %u, client %s\nok\n",
                    WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(), WiFi.RSSI(), TCP_PORT,
                    client.connected() ? "connected" : "none");
    else
      Serial.printf("wifi: %s\nok\n", prefs.getString("ssid", "").length() ? "not connected" : "not configured");
  } else if (cmd == "pins") {
    for (int i = 0; i < NPROBE; i++) Serial.printf("GPIO%d = %s\n", PROBE[i], LABEL[i]);
    Serial.println("ok");
  } else {
    Serial.println("? commands: scan [ms] | bridge <rx> <baud> [tx] | set <pin> 0|1|z | pwm <pin> <hz> <duty%> | pins");
  }
}

void setup() {
  Serial.setRxBufferSize(1024);
  Serial.begin(921600);
  for (int i = 0; i < NPROBE; i++) pinMode(PROBE[i], INPUT);
  delay(200);
  Serial.println("\nLDS probe ready. Type: info");
  prefs.begin("lds", false);
  String ssid = prefs.getString("ssid", "");
  if (ssid.length()) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);  // low latency for the data stream
    WiFi.begin(ssid.c_str(), prefs.getString("pass", "").c_str());
  }
  handle(AUTOSTART);  // measure + control the motor on its own, also without a PC (battery pack)
}

void network() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (!mdnsUp) {  // only start once the network is up (earlier crashes lwIP)
    server.begin();
    server.setNoDelay(true);
    if (MDNS.begin("lds")) MDNS.addService("lds", "tcp", TCP_PORT);
    mdnsUp = true;
  }
  if (server.hasClient()) {  // a new connection replaces the old one
    if (client) client.stop();
    client = server.accept();
    client.setNoDelay(true);
  }
}

void loop() {
  if (bridging) {
    uint8_t buf[512];
    size_t n = Serial1.available();
    if (n) {
      n = Serial1.read(buf, min(n, sizeof(buf)));
      Serial.write(buf, n);
      if (client && client.connected()) client.write(buf, n);
      if (motorPin >= 0) feedParser(buf, n);
    }
    motorControl();
    network();
    if (bridgeTx >= 0) {
      n = Serial.available();
      if (n) Serial1.write(buf, Serial.read(buf, min(n, sizeof(buf))));
      return;
    }
  }
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { handle(line); line = ""; }
    else if (line.length() < 120) line += ch;
  }
}

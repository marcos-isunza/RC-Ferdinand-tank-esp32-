/*
 * RC Tank — CL 9030 brushed ESC (2× single-channel PWM)
 * Target: ESP32 Dev Module (30-pin, Classic BT for PS3) · ESP32 core 3.x
 *
 * ═══════════════════════════════════════════════════════════════════════
 * ARDUINO IDE
 * ═══════════════════════════════════════════════════════════════════════
 *   Board:              ESP32 Dev Module (WROOM, 30-pin classic)
 *   Flash Size:         4MB
 *   Partition Scheme:   Minimal SPIFFS (1.9MB APP)
 *
 * ═══════════════════════════════════════════════════════════════════════
 * ESC WIRING (3-wire receiver lead per CL 9030)
 * ═══════════════════════════════════════════════════════════════════════
 *   White  → GPIO signal (left GPIO5, right GPIO4)
 *   Black  → GND (required)
 *   Red    → 5V BEC optional (leave off if ESP powered by USB)
 *
 *   After upload, Serial shows a 2.5 s neutral hold — connect ESC battery then.
 *
 * ═══════════════════════════════════════════════════════════════════════
 * DRIVING — PS3 (DualShock 3) only
 * ═══════════════════════════════════════════════════════════════════════
 *   Left stick  (up/down) = left track
 *   Right stick (up/down) = right track
 *   PS button             = all stop
 *   Pair with SixaxisPairTool / sixaxispairer to BT MAC from Serial (btmac).
 *   Patched PS3 library: esp/libraries/PS3_Controller_Host/ (ESP32 core 3.x).
 *   Controller disconnect = all stop.
 *
 * ═══════════════════════════════════════════════════════════════════════
 * WiFi AP — configuration only (no driving)
 * ═══════════════════════════════════════════════════════════════════════
 *   SSID:     TankRC
 *   Password: tank1234
 *   IP:       192.168.4.1
 *
 *   GET /                  config page
 *   GET /status            JSON state
 *   GET /config?top=19.0..100   set max ESC throttle % at full stick (saved to flash)
 *   GET /allstop
 *
 * Serial (115200):  l <pct>  |  r <pct>  |  top <pct>  |  stop  |  status  |  btmac
 */

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <esp_system.h>
#include <esp_chip_info.h>

#include <Ps3Controller.h>
#if __has_include("esp32-hal-alloc-bt-classic-mem.h")
#include "esp32-hal-alloc-bt-classic-mem.h"
#endif

// ─── WiFi ───────────────────────────────────────────────────────────────────

const char* ap_ssid     = "TankRC";
const char* ap_password = "tank1234";

WebServer server(80);
DNSServer dnsServer;
Preferences prefs;

// ─── ESC pins (white wire) ──────────────────────────────────────────────────

// Physical left track is wired to GPIO5, right track to GPIO4 (bench-verified with PS3).
#define PIN_ESC_LEFT   5
#define PIN_ESC_RIGHT  4

// One track is wired/mechanically reversed — flip here (not both). Follows the physical left track.
#define INVERT_LEFT_TRACK   true
#define INVERT_RIGHT_TRACK  false

#define ESC_FREQ_HZ        50
#define ESC_PERIOD_US      20000
// 50 Hz must use 14-bit LEDC on ESP32 (16-bit math produces ~4× too long pulses → ESC fault beep)
#define ESC_LEDC_RES       14
#define ESC_LEDC_MAX_DUTY  ((1u << ESC_LEDC_RES) - 1)

#define NEUTRAL_HOLD_MS    2500   // hold 1500 µs before WiFi; connect ESC battery during this window

#define ESC_PULSE_MIN_US   800
#define ESC_PULSE_NEUT_US  1500
#define ESC_PULSE_MAX_US   2200
#define ESC_US_PER_PCT     7      // 1% ESC throw = 7 µs

#define SLEW_STEP_US       25
#define SLEW_INTERVAL_MS   20

// Bench-measured on this gearbox: motion begins ~19% ESC throw; 21% is max desired speed.
#define THROTTLE_CRAWL_PCT 19   // minimum |ESC| % before pulse leaves neutral band
#define THROTTLE_TOP_DEFAULT_TENTHS 210  // default max |ESC| % at full stick (21.0%); adjustable on config page

// Max throttle is stored in tenths of a percent (190 = 19.0%), range crawl..100%.
#define THROTTLE_TOP_MIN_TENTHS (THROTTLE_CRAWL_PCT * 10)
#define THROTTLE_TOP_MAX_TENTHS 1000

#define PS3_POLL_MS        20
#define STICK_DEADZONE     18   // on ±100 stick scale after mapStickAxis

// ─── State ──────────────────────────────────────────────────────────────────

int16_t targetLeftPct  = 0;   // -100 .. +100
int16_t targetRightPct = 0;
uint16_t currentLeftUs  = ESC_PULSE_NEUT_US;
uint16_t currentRightUs = ESC_PULSE_NEUT_US;
uint32_t lastSlewMs     = 0;
uint16_t throttleTopTenths = THROTTLE_TOP_DEFAULT_TENTHS;  // max ESC throw at full stick, 0.1% units

bool     ps3Ready       = false;
bool     ps3Connected   = false;
char     ps3BtMac[18]   = "";
uint32_t lastPs3PollMs  = 0;

// ─── Max throttle (web config) ──────────────────────────────────────────────

void setThrottleTop(int tenths, bool save) {
  throttleTopTenths = (uint16_t)constrain(tenths, THROTTLE_TOP_MIN_TENTHS, THROTTLE_TOP_MAX_TENTHS);
  if (save) {
    prefs.putUShort("top", throttleTopTenths);
  }
}

// ─── ESC PWM ────────────────────────────────────────────────────────────────

// User -100..100 → signed pulse offset from neutral in µs.
// Below crawl = 0; crawl..100 maps to crawl..configured max throttle.
int16_t userThrottleToOffsetUs(int16_t userPct) {
  if (userPct == 0) {
    return 0;
  }
  int sign = userPct > 0 ? 1 : -1;
  int mag = abs(userPct);
  if (mag < THROTTLE_CRAWL_PCT) {
    return 0;
  }
  mag = constrain(mag, THROTTLE_CRAWL_PCT, 100);
  int32_t crawlUs = THROTTLE_CRAWL_PCT * ESC_US_PER_PCT;
  int32_t topUs   = (int32_t)throttleTopTenths * ESC_US_PER_PCT / 10;
  int32_t offsetUs = crawlUs + (int32_t)(mag - THROTTLE_CRAWL_PCT) *
                               (topUs - crawlUs) / (100 - THROTTLE_CRAWL_PCT);
  return (int16_t)(sign * offsetUs);
}

int16_t userPctForTrack(uint8_t trackIndex, int16_t userPct) {
  if (trackIndex == 0 && INVERT_LEFT_TRACK) {
    return (int16_t)(-userPct);
  }
  if (trackIndex == 1 && INVERT_RIGHT_TRACK) {
    return (int16_t)(-userPct);
  }
  return userPct;
}

uint16_t trackPulseUs(uint8_t trackIndex, int16_t userPct) {
  int16_t offsetUs = userThrottleToOffsetUs(userPctForTrack(trackIndex, userPct));
  return (uint16_t)(ESC_PULSE_NEUT_US + offsetUs);
}

void escApplyPulseUs(uint8_t pin, uint16_t pulseUs) {
  pulseUs = constrain(pulseUs, ESC_PULSE_MIN_US, ESC_PULSE_MAX_US);
  uint32_t duty = (uint32_t)pulseUs * ESC_LEDC_MAX_DUTY / ESC_PERIOD_US;
  ledcWrite(pin, duty);
}

bool escAttachChannel(uint8_t pin) {
  if (!ledcAttach(pin, ESC_FREQ_HZ, ESC_LEDC_RES)) {
    Serial.print(F("[TankRC] ERROR: ledcAttach failed on GPIO "));
    Serial.println(pin);
    return false;
  }
  escApplyPulseUs(pin, ESC_PULSE_NEUT_US);
  return true;
}

void initEscOutputsFirst() {
  bool okL = escAttachChannel(PIN_ESC_LEFT);
  bool okR = escAttachChannel(PIN_ESC_RIGHT);
  currentLeftUs  = ESC_PULSE_NEUT_US;
  currentRightUs = ESC_PULSE_NEUT_US;
  targetLeftPct  = 0;
  targetRightPct = 0;

  if (okL) {
    uint32_t dutyL = (uint32_t)ESC_PULSE_NEUT_US * ESC_LEDC_MAX_DUTY / ESC_PERIOD_US;
    Serial.print(F("[TankRC] Left GPIO"));
    Serial.print(PIN_ESC_LEFT);
    Serial.print(F(" neutral duty="));
    Serial.print(dutyL);
    Serial.print(F("/"));
    Serial.println(ESC_LEDC_MAX_DUTY);
  }
  if (okR) {
    Serial.print(F("[TankRC] Right GPIO"));
    Serial.println(PIN_ESC_RIGHT);
  }

  Serial.print(F("[TankRC] Hold neutral "));
  Serial.print(NEUTRAL_HOLD_MS / 1000);
  Serial.println(F(" s — plug ESC battery now if not already."));
  uint32_t holdStart = millis();
  while (millis() - holdStart < NEUTRAL_HOLD_MS) {
    escApplyPulseUs(PIN_ESC_LEFT, ESC_PULSE_NEUT_US);
    escApplyPulseUs(PIN_ESC_RIGHT, ESC_PULSE_NEUT_US);
    delay(20);
  }
}

void setTargets(int16_t leftPct, int16_t rightPct) {
  targetLeftPct  = constrain(leftPct, -100, 100);
  targetRightPct = constrain(rightPct, -100, 100);
}

void allStop() {
  setTargets(0, 0);
  currentLeftUs  = ESC_PULSE_NEUT_US;
  currentRightUs = ESC_PULSE_NEUT_US;
  escApplyPulseUs(PIN_ESC_LEFT, currentLeftUs);
  escApplyPulseUs(PIN_ESC_RIGHT, currentRightUs);
}

void tickEscSlew() {
  uint32_t now = millis();
  if (now - lastSlewMs < SLEW_INTERVAL_MS) {
    return;
  }
  lastSlewMs = now;

  uint16_t goalL = trackPulseUs(0, targetLeftPct);
  uint16_t goalR = trackPulseUs(1, targetRightPct);

  auto stepUs = [](uint16_t cur, uint16_t goal) -> uint16_t {
    if (cur == goal) {
      return cur;
    }
    if (cur < goal) {
      uint16_t next = cur + SLEW_STEP_US;
      return next > goal ? goal : next;
    }
    uint16_t next = (cur > SLEW_STEP_US) ? (cur - SLEW_STEP_US) : ESC_PULSE_MIN_US;
    return next < goal ? goal : next;
  };

  currentLeftUs  = stepUs(currentLeftUs, goalL);
  currentRightUs = stepUs(currentRightUs, goalR);
  escApplyPulseUs(PIN_ESC_LEFT, currentLeftUs);
  escApplyPulseUs(PIN_ESC_RIGHT, currentRightUs);
}

// ─── PS3 (dual-stick tank drive) ────────────────────────────────────────────

int mapStickAxis(int raw) {
  int v = raw * 100 / 127;
  if (abs(v) <= STICK_DEADZONE) {
    return 0;
  }
  return constrain(v, -100, 100);
}

void pollPs3Input() {
  if (!ps3Ready || !ps3Connected) {
    return;
  }
  if (Ps3.data.button.ps) {
    allStop();
    return;
  }
  // PS3 stick Y: up = negative → negate so forward = +
  int left  = -mapStickAxis(Ps3.data.analog.stick.ly);
  int right = -mapStickAxis(Ps3.data.analog.stick.ry);
  setTargets(left, right);
}

void onPs3Connect() {
  ps3Connected = true;
  Ps3.setPlayer(1);
  Ps3.setRumble(75.0f, 350);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  Serial.println(F("[TankRC] PS3 connected."));
}

void onPs3Disconnect() {
  ps3Connected = false;
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  allStop();
  Serial.println(F("[TankRC] PS3 disconnected — stopped."));
}

void onPs3Data() {
  // Stick input polled from loop() at PS3_POLL_MS
}

void attachPs3Callbacks() {
  Ps3.attach(onPs3Data);
  Ps3.attachOnConnect(onPs3Connect);
  Ps3.attachOnDisconnect(onPs3Disconnect);
}

bool initPs3() {
  ps3Ready = Ps3.begin();
  if (ps3Ready) {
    attachPs3Callbacks();
    String mac = Ps3.getAddress();
    mac.toCharArray(ps3BtMac, sizeof(ps3BtMac));
    Serial.println(F("[TankRC] PS3 stack listening (WiFi AP starts next)."));
    Serial.print(F("[TankRC] Pair PS3 to BT MAC: "));
    Serial.println(ps3BtMac);
    Serial.println(F("[TankRC] SixaxisPairTool → write MAC → unplug USB → press PS."));
  } else {
    ps3BtMac[0] = '\0';
    Serial.println(F("[TankRC] PS3 / BT init failed — no driving input!"));
    Serial.println(F("[TankRC] Use Partition: Minimal SPIFFS (1.9MB APP)."));
  }
  return ps3Ready;
}

void tickPs3WaitingHint() {
  static uint32_t lastHintMs = 0;
  if (!ps3Ready || ps3Connected || Ps3.isConnected()) {
    return;
  }
  uint32_t now = millis();
  if (now - lastHintMs < 15000UL) {
    return;
  }
  lastHintMs = now;
  Serial.print(F("[TankRC] Waiting for PS3… pair to MAC "));
  Serial.print(ps3BtMac);
  Serial.println(F(" then press PS (USB unplugged)."));
}

void tickPs3Connection() {
  if (!ps3Ready) {
    return;
  }
  bool link = Ps3.isConnected();
  if (!link && ps3Connected) {
    onPs3Disconnect();
  }
}

// ─── Serial bench helpers ───────────────────────────────────────────────────

void printRuntimeChipLine() {
  esp_chip_info_t info;
  esp_chip_info(&info);
  Serial.print(F("[TankRC] Silicon: "));
  if (info.model == CHIP_ESP32) {
    Serial.print(F("ESP32"));
    Serial.print(F(", cores="));
    Serial.print(info.cores);
    Serial.print(F(", rev="));
    Serial.println(info.revision);
  } else {
    Serial.println(F("NOT classic ESP32 — PS3 will not work on this chip"));
  }
}

void printPs3PairingHelp() {
  if (ps3Ready) {
    Serial.print(F("[TankRC] Pair PS3 to BT MAC: "));
    Serial.println(ps3BtMac);
    Serial.println(F("[TankRC] SixaxisPairTool (USB) → set MAC → unplug → press PS."));
  } else {
    Serial.println(F("[TankRC] PS3 / BT init failed."));
    Serial.println(F("[TankRC] Try Partition: Minimal SPIFFS (1.9MB APP)."));
  }
}

void printStatusSerial() {
  Serial.print(F("[TankRC] L="));
  Serial.print(targetLeftPct);
  Serial.print(F("% ("));
  Serial.print(currentLeftUs);
  Serial.print(F(" us)  R="));
  Serial.print(targetRightPct);
  Serial.print(F("% ("));
  Serial.print(currentRightUs);
  Serial.print(F(" us)  top="));
  Serial.print(throttleTopTenths / 10.0f, 1);
  Serial.println(F("%"));
}

void pollSerialCommands() {
  if (!Serial.available()) {
    return;
  }
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) {
    return;
  }
  if (line.equalsIgnoreCase(F("stop")) || line.equalsIgnoreCase(F("s"))) {
    allStop();
    Serial.println(F("[TankRC] stop"));
    return;
  }
  if (line.equalsIgnoreCase(F("status"))) {
    printStatusSerial();
    return;
  }
  if (line.equalsIgnoreCase(F("btmac")) || line.equalsIgnoreCase(F("mac"))) {
    printPs3PairingHelp();
    return;
  }
  if (line.startsWith(F("top "))) {
    setThrottleTop((int)(line.substring(4).toFloat() * 10.0f + 0.5f), true);
    printStatusSerial();
    return;
  }
  if (line.startsWith(F("l ")) || line.startsWith(F("L "))) {
    int pct = line.substring(2).toInt();
    setTargets(pct, targetRightPct);
    printStatusSerial();
    return;
  }
  if (line.startsWith(F("r ")) || line.startsWith(F("R "))) {
    int pct = line.substring(2).toInt();
    setTargets(targetLeftPct, pct);
    printStatusSerial();
    return;
  }
  Serial.println(F("[TankRC] Commands: l/r <pct>, top <pct>, stop, status, btmac"));
}

// ─── WiFi AP ────────────────────────────────────────────────────────────────

void initWiFiAP() {
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(true);

  bool apStarted = false;
  for (int attempt = 0; attempt < 3; attempt++) {
    if (WiFi.softAP(ap_ssid, ap_password)) {
      apStarted = true;
      break;
    }
    delay(500);
  }
  if (!apStarted) {
    Serial.println(F("[TankRC] ERROR: WiFi AP failed."));
    return;
  }

  dnsServer.start(53, "*", WiFi.softAPIP());
}

// ─── HTTP handlers ──────────────────────────────────────────────────────────

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>RC Tank Config</title>
<style>
:root{--bg:#0f172a;--card:#1e293b;--text:#f1f5f9;--muted:#94a3b8;--border:#334155;--acc:#38bdf8}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,sans-serif;background:var(--bg);color:var(--text);padding:14px;max-width:420px;margin:0 auto}
h1{font-size:1.1rem;margin-bottom:8px}
.sub{font-size:.75rem;color:var(--muted);margin-bottom:12px}
.estop{width:100%;min-height:52px;border:none;border-radius:14px;background:linear-gradient(180deg,#dc2626,#991b1b);color:#fff;font-weight:800;font-size:1rem;margin-bottom:12px}
.card{background:var(--card);border:1px solid var(--border);border-radius:14px;padding:12px;margin-bottom:10px}
.card h2{font-size:.85rem;margin-bottom:8px}
.row{display:flex;gap:8px;align-items:center;margin:8px 0}
.row input[type=range]{flex:1;accent-color:var(--acc)}
.val{font-size:.9rem;width:48px;text-align:right}
.save{width:100%;min-height:44px;border:none;border-radius:10px;background:var(--acc);color:#0f172a;font-weight:700}
.status{font-size:.78rem;color:var(--muted);background:var(--card);border:1px solid var(--border);border-radius:10px;padding:10px;margin-bottom:10px}
</style>
</head>
<body>
<h1>RC Tank — Config</h1>
<p class="sub">Drive with the PS3 controller: left stick = left track, right stick = right track.</p>
<div class="status" id="status">Connecting…</div>

<div class="card">
<h2>Max throttle (full stick)</h2>
<div class="row"><input type="range" id="lim" min="19" max="100" step="0.5" value="21" oninput="document.getElementById('limv').textContent=this.value+'%'"><span class="val" id="limv">21%</span></div>
<button class="save" onclick="saveLimit()">Save</button>
<p class="sub" style="margin-top:8px" id="msg">ESC throttle sent at full stick. Motors start moving at 19%. Saved across reboots.</p>
</div>

<button class="estop" onclick="fetch('/allstop')">ALL STOP</button>

<script>
var editing=false;
function saveLimit(){
  var v=document.getElementById('lim').value;
  fetch('/config?top='+v).then(function(r){return r.json();}).then(function(s){
    editing=false;
    document.getElementById('msg').textContent='Saved: '+s.top+'%';
    applyStatus(s,true);
  }).catch(function(){document.getElementById('msg').textContent='Save failed';});
}
function applyStatus(s,force){
  document.getElementById('status').textContent=(s.ps3?'PS3 connected':'PS3 not connected')+' · L '+s.left+'% · R '+s.right+'% · max throttle '+s.top+'%';
  if(force||!editing){
    document.getElementById('lim').value=s.top;
    document.getElementById('limv').textContent=s.top+'%';
  }
}
document.getElementById('lim').addEventListener('input',function(){editing=true;});
function pollStatus(){fetch('/status').then(function(r){return r.json();}).then(function(s){applyStatus(s,false);}).catch(function(){});}
document.addEventListener('DOMContentLoaded',function(){pollStatus();setInterval(pollStatus,1000);});
</script>
</body>
</html>
)rawliteral";

void sendStatusJson() {
  char buf[200];
  snprintf(buf, sizeof(buf),
           "{\"left\":%d,\"right\":%d,\"leftUs\":%u,\"rightUs\":%u,"
           "\"top\":%u.%u,\"uptime\":%lu,\"ps3\":%s}",
           (int)targetLeftPct, (int)targetRightPct,
           (unsigned)currentLeftUs, (unsigned)currentRightUs,
           (unsigned)(throttleTopTenths / 10), (unsigned)(throttleTopTenths % 10),
           (unsigned long)millis(),
           ps3Connected ? "true" : "false");
  server.send(200, "application/json", buf);
}

void setupHttpRoutes() {
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", INDEX_HTML);
  });

  server.on("/status", HTTP_GET, []() {
    sendStatusJson();
  });

  server.on("/config", HTTP_GET, []() {
    if (!server.hasArg("top")) {
      server.send(400, "text/plain", "top=19.0..100");
      return;
    }
    setThrottleTop((int)(server.arg("top").toFloat() * 10.0f + 0.5f), true);
    Serial.print(F("[TankRC] /config top="));
    Serial.println(throttleTopTenths / 10.0f, 1);
    sendStatusJson();
  });

  server.on("/allstop", HTTP_GET, []() {
    allStop();
    server.send(200, "text/plain", "ok");
  });

  server.onNotFound([]() {
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });

  server.begin();
}

// ─── Setup & loop ───────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(300);

  prefs.begin("tankrc", false);
  setThrottleTop(prefs.getUShort("top", THROTTLE_TOP_DEFAULT_TENTHS), false);

  // Neutral PWM before WiFi — ESC sees valid midpoint when battery is connected
  initEscOutputsFirst();
  Serial.println(F("\n[TankRC] ESC outputs at 1500 us (neutral)."));

  if (esp_reset_reason() == ESP_RST_BROWNOUT) {
    Serial.println(F("[TankRC] WARNING: last reset was brownout."));
  }

  initPs3();
  initWiFiAP();
  setupHttpRoutes();

  Serial.println(F("[TankRC] WiFi AP: TankRC / tank1234 (config only)"));
  Serial.print(F("[TankRC] Open http://"));
  Serial.println(WiFi.softAPIP());
  Serial.print(F("[TankRC] Max throttle: "));
  Serial.print(throttleTopTenths / 10.0f, 1);
  Serial.println(F("%"));
  Serial.println(F("[TankRC] Serial: l <pct>, r <pct>, top <pct>, stop, status, btmac"));
  Serial.println(F("[TankRC] Bench: USB power ESP → verify neutral → then ESC battery."));
  printRuntimeChipLine();
  printPs3PairingHelp();
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();
  tickEscSlew();
  pollSerialCommands();
  tickPs3Connection();
  tickPs3WaitingHint();
  if (ps3Ready) {
    Ps3.tickRearm();
  }
  if (ps3Ready && ps3Connected) {
    uint32_t now = millis();
    if (now - lastPs3PollMs >= PS3_POLL_MS) {
      lastPs3PollMs = now;
      pollPs3Input();
    }
  }
}

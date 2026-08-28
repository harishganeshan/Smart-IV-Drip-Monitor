/*
  =====================================================================
   SMART IV DRIP MONITORING SYSTEM  (PlatformIO version)
  =====================================================================
  Components:
   - ESP32 Dev Board
   - MAX30102 Pulse Oximeter & Heart Rate Sensor  (I2C)
   - 128x64 OLED Display (SSD1306, I2C)
   - Non-Contact Capacitive Liquid Level Sensor (SEMLAB CLS-24BP / NPN-PNP
     open-collector digital output, clamped onto the IV bottle/tube)
   - Active Buzzer
   - Built-in ESP32 Wi-Fi web server (hospital patient monitor dashboard)

  Wiring:
   MAX30102        -> ESP32
     VIN            -> 3.3V
     GND            -> GND
     SDA            -> GPIO 21
     SCL            -> GPIO 22

   OLED 128x64 (I2C)-> ESP32
     VCC            -> 3.3V
     GND            -> GND
     SDA            -> GPIO 21   (same bus as MAX30102)
     SCL            -> GPIO 22   (same bus as MAX30102)

   Liquid Level Sensor -> ESP32
     VCC (5-24V)    -> External 5V / 12V supply as per sensor rating
     GND            -> Common GND with ESP32 (must share ground)
     OUT (NPN/PNP)  -> GPIO 26  (through a voltage divider, see note below)

   Buzzer -> ESP32
     +              -> GPIO 25
     -              -> GND

   SOS Button -> ESP32   (see SOS_BUTTON_MODE below)
     MODE 1 (default): digital touch module (TTP223) or push-button
       module VCC -> 3.3V,  GND -> GND,  OUT/SIG -> GPIO 27
     MODE 0: a bare wire / copper pad straight on GPIO 27 (capacitive TOUCH7)

  IMPORTANT NOTE ON THE LIQUID SENSOR OUTPUT VOLTAGE:
   ESP32 GPIOs are NOT 5V/12V/24V tolerant. If you run the sensor at 5V
   in NPN mode, use a 10k/20k voltage divider from OUT to GPIO26 to bring
   5V down to ~3.3V. If using 12V/24V or PNP mode, use a level shifter
   or optocoupler instead.

  SALINE (WATER) LEVEL SENSING:
   The capacitive sensor is clamped on the IV bottle at the ~30% mark.
   While saline covers the sensor  -> "liquid present"  -> level  > 30%  (NORMAL)
   Once saline drops below it      -> "liquid absent"   -> level <= 30%  (LOW)
   The LOW condition raises the buzzer alarm and the web dashboard banner.
  =====================================================================
*/

#include <Arduino.h>
#include <string.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

// ---------------- WI-FI / WEB SERVER CONFIG ----------------
// Set USE_AP_MODE = 1 -> ESP32 creates its own Wi-Fi hotspot (no router needed).
// Set USE_AP_MODE = 0 -> ESP32 joins the hospital Wi-Fi below.
#define USE_AP_MODE     1

#define WIFI_SSID       "Dhyana Engineering Solutions"
#define WIFI_PASSWORD   "DhyanaLab@0912"

#define AP_SSID         "SmartIV-Monitor"
#define AP_PASSWORD     "ivmonitor123"   // must be at least 8 characters

WebServer server(80);
String gIpAddress = "0.0.0.0";

// ---------------- OLED CONFIG ----------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDRESS  0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------------- MAX30102 CONFIG ----------------
MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255
uint32_t irBuffer[100];         // sliding window, oldest at [0]
uint32_t redBuffer[100];
int32_t bufferLength = 100;
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;

uint16_t sampleCount  = 0;      // 0..100, how full the window is
uint16_t newSinceCalc = 0;      // fresh samples since the last SpO2 recompute
unsigned long lastSampleMs = 0;

// Finger is "present" when the IR DC level is above this. Tune with the
// serial monitor: with no finger IR reads a few thousand; a good contact
// reads > 80000. 50000 is a safe middle ground.
#define IR_FINGER_THRESHOLD 50000

// ---- Real-time heart-beat detector (heartRate.h) ----
const byte RATE_SIZE = 8;
byte  rates[RATE_SIZE];
byte  rateSpot = 0;
unsigned long lastBeatMs = 0;
int   beatAvgBpm = 0;

// ================= SOFTWARE CALIBRATION =================
// Compare the readings against a reference pulse oximeter, then nudge these.
//   offset : add a fixed correction   (e.g. SPO2_CAL_OFFSET = +2 if it reads 2% low)
//   gain   : multiply, for slope errors (keep 1.0 unless a 2-point check needs it)
float SPO2_CAL_OFFSET = 0.0f;   // %
float SPO2_CAL_GAIN   = 1.0f;
float HR_CAL_OFFSET   = 0.0f;   // bpm
float HR_CAL_GAIN     = 1.0f;

// Plausibility gates - readings outside these are rejected, not shown.
const int SPO2_MIN_VALID = 80;
const int SPO2_MAX_VALID = 100;
const int HR_MIN_VALID   = 40;
const int HR_MAX_VALID   = 180;

// Perfusion index (%) = AC/DC of the IR signal. Real fingers sit around
// 0.4-5%. Below this the pulsatile signal is too weak to trust.
const float MIN_PERFUSION_INDEX = 0.15f;

// Ignore the first few seconds after a finger is placed (signal settling).
const unsigned long WARMUP_MS = 3000;

// Output smoothing: median-of-5 (kills spikes) then an exponential average.
const float SPO2_EMA_ALPHA = 0.30f;
const float HR_EMA_ALPHA   = 0.35f;

int   spo2Hist[5]; int spo2HistN = 0, spo2HistIdx = 0; float spo2Ema = 0; bool spo2Has = false;
int   hrHist[5];   int hrHistN   = 0, hrHistIdx   = 0; float hrEma   = 0; bool hrHas   = false;

float gPerfusionIndex = 0.0f;
// 0 = no finger, 1 = calibrating, 2 = weak signal, 3 = good signal
volatile int gQualityCode = 0;
unsigned long fingerSinceMs = 0;
bool prevFingerPresent = false;

// ---------------- LIQUID LEVEL SENSOR CONFIG ----------------
#define LIQUID_SENSOR_PIN 26

// Set this after testing your sensor:
// If sensor output goes LOW when liquid IS present, use LOW.
// If it goes HIGH when liquid IS present, use HIGH.
#define LIQUID_DETECTED_LEVEL LOW

// Saline is judged relative to the sensor clamp position on the bottle.
#define SALINE_SENSOR_MARK_PERCENT 30

// ---------------- SOS TOUCH BUTTON CONFIG ----------------
#define SOS_TOUCH_PIN        27

// How the SOS button is wired:
//   1 = DIGITAL module / push-button (TTP223 touch board, or a normal switch).
//       The module outputs a clean HIGH/LOW - read with digitalRead().
//       This is the default; most sensor kits ship the TTP223 board.
//   0 = BARE capacitive pad straight on GPIO27, read with the ESP32's
//       built-in touchRead().  (needs the 2.0.x core - see platformio.ini)
#define SOS_BUTTON_MODE      0

// --- settings for SOS_BUTTON_MODE 1 (digital) ---
// TTP223 idles LOW and goes HIGH on touch -> HIGH. If your module/wiring is
// inverted (or a button to GND with a pull-up), set this to LOW.
#define SOS_DIGITAL_ACTIVE   HIGH

// --- settings for SOS_BUTTON_MODE 0 (bare capacitive pad) ---
// Idle reading auto-measured at boot; SOS fires when the live reading moves
// this many % away from it. Watch the "[sos]" line on the serial monitor.
#define SOS_TOUCH_DROP_PCT   30

// The button only RAISES the SOS alert (a quick tap latches it on). Clearing
// it is done from the web dashboard - the "RESET SOS ALERT" button on the
// red SOS screen, which calls POST /sos/clear.

// ---------------- BUZZER CONFIG ----------------
#define BUZZER_PIN 25

// ---------------- TIMING ----------------
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL = 500;

int lastValidBPM = 0;
int lastValidSpO2 = 0;

// ---------------- SHARED STATE (for the web dashboard) ----------------
volatile bool gFingerDetected = false;
volatile bool gSalinePresent  = true;   // true  -> saline above the 30% mark
volatile bool gSensorOk       = false;  // MAX30102 present & initialised
volatile bool gSosActive      = false;  // patient SOS touch button latched
int gTouchBaseline = 60;                // idle touchRead(), measured at boot (mode 0)
int gSosRaw = 0;                        // last raw button reading (for debug)

// Alarm priority: SOS outranks the saline alarm.
enum AlarmMode : uint8_t { ALARM_NONE = 0, ALARM_SALINE, ALARM_SOS };

void webServerTask(void *param);
void setupWiFi();
void handleRoot();
void handleData();
void handleSosClear();
void handleNotFound();
String buildJson();

void configureSensor();
void pollSensor();
void recomputeVitals();
void checkLiquidLevel();
void checkSosButton();
void updateAlarms();
void updateAlarm(AlarmMode mode);
bool isLiquidPresent();
void updateDisplay(bool fingerDetected);

// ---------------- WEB DASHBOARD (served from flash) ----------------
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Smart IV Drip Monitor</title>
<style>
  :root{
    --bg:#0b1020; --panel:#131a2e; --line:#243049;
    --ok:#22c55e; --spo2:#38bdf8; --bpm:#34d399; --warn:#f43f5e; --muted:#8ea0c0;
  }
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:var(--bg);color:#e8eefc;font-family:-apple-system,Segoe UI,Roboto,Helvetica,Arial,sans-serif;
       min-height:100vh;padding:18px}
  header{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-bottom:16px}
  h1{font-size:18px;letter-spacing:.5px;font-weight:600}
  .status{display:flex;align-items:center;gap:8px;font-size:13px;color:var(--muted)}
  .dot{width:9px;height:9px;border-radius:50%;background:var(--warn)}
  .dot.live{background:var(--ok);box-shadow:0 0 10px var(--ok)}
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:14px}
  .card{background:var(--panel);border:1px solid var(--line);border-radius:16px;padding:18px 20px}
  .label{font-size:12px;text-transform:uppercase;letter-spacing:1.5px;color:var(--muted)}
  .value{font-variant-numeric:tabular-nums;font-weight:700;line-height:1;margin-top:10px}
  .value .num{font-size:64px}
  .value .unit{font-size:20px;color:var(--muted);margin-left:6px}
  .spo2 .num{color:var(--spo2)}
  .bpm .num{color:var(--bpm)}
  .sub{margin-top:10px;font-size:13px;color:var(--muted)}
  .saline{display:flex;gap:18px;align-items:stretch}
  .tube{width:46px;min-height:150px;border:2px solid var(--line);border-radius:10px;position:relative;overflow:hidden;background:#0c1222}
  .fill{position:absolute;left:0;right:0;bottom:0;background:linear-gradient(180deg,#60a5fa,#2563eb);transition:height .6s ease,background .3s}
  .fill.low{background:linear-gradient(180deg,#fb7185,#e11d48)}
  .saline .meta{flex:1;display:flex;flex-direction:column;justify-content:center}
  .pill{display:inline-block;margin-top:10px;padding:5px 12px;border-radius:999px;font-size:13px;font-weight:600}
  .pill.ok{background:rgba(34,197,94,.15);color:var(--ok)}
  .pill.low{background:rgba(244,63,94,.15);color:var(--warn)}
  .banner{margin-bottom:14px;padding:14px 18px;border-radius:14px;background:var(--warn);
          color:#fff;font-weight:700;letter-spacing:.5px;display:none;animation:blink 1s steps(2) infinite}
  .banner.show{display:block}
  @keyframes blink{50%{opacity:.55}}
  footer{margin-top:16px;font-size:12px;color:var(--muted);text-align:center}

  /* SOS: patient panic button - whole screen turns red, everything else hidden */
  #sos{position:fixed;inset:0;z-index:1000;display:none;
       flex-direction:column;align-items:center;justify-content:center;text-align:center;
       padding:24px;background:#d11a1a;color:#fff;animation:sospulse .7s steps(2) infinite}
  #sos.show{display:flex}
  #sos .big{font-size:clamp(72px,26vw,190px);font-weight:900;letter-spacing:10px;line-height:.95}
  #sos .msg{margin-top:14px;font-size:clamp(18px,5vw,34px);font-weight:800;letter-spacing:2px}
  #sos .clr{margin-top:30px;padding:16px 34px;font:inherit;font-size:clamp(15px,4vw,20px);
            font-weight:800;letter-spacing:1px;color:#b91c1c;background:#fff;border:3px solid #fff;
            border-radius:12px;cursor:pointer}
  #sos .clr:active{transform:scale(.97)}
  #sos .hint{margin-top:16px;font-size:clamp(12px,3vw,16px);font-weight:600;opacity:.85}
  @keyframes sospulse{50%{background:#8f0d0d}}
</style>
</head>
<body>
  <header>
    <h1>SMART IV DRIP &mdash; PATIENT MONITOR</h1>
    <div class="status"><span id="dot" class="dot"></span><span id="conn">connecting&hellip;</span></div>
  </header>

  <div id="banner" class="banner">&#9888; SALINE LOW (&le;30%) &mdash; REPLACE IV BOTTLE</div>

  <div id="sos">
    <div class="big">SOS</div>
    <div class="msg">PATIENT EMERGENCY &mdash; RESPOND NOW</div>
    <button id="sosClear" class="clr" type="button">RESET SOS ALERT</button>
    <div class="hint">raised from the bedside button &middot; clear it here after attending</div>
  </div>

  <div class="grid">
    <div class="card spo2">
      <div class="label">SpO&#8322; &mdash; Oxygen Saturation</div>
      <div class="value"><span id="spo2" class="num">--</span><span class="unit">%</span></div>
      <div id="spo2sub" class="sub">place finger on sensor</div>
    </div>

    <div class="card bpm">
      <div class="label">Heart Rate</div>
      <div class="value"><span id="bpm" class="num">--</span><span class="unit">BPM</span></div>
      <div id="bpmsub" class="sub">place finger on sensor</div>
    </div>

    <div class="card">
      <div class="label">Saline Level</div>
      <div class="saline">
        <div class="tube"><div id="fill" class="fill" style="height:70%"></div></div>
        <div class="meta">
          <div id="salineTxt" class="value" style="font-size:26px">--</div>
          <div><span id="salinePill" class="pill">&hellip;</span></div>
          <div class="sub">sensor clamp at 30% mark</div>
        </div>
      </div>
    </div>
  </div>

  <footer>Auto-refresh every second &middot; uptime <span id="uptime">0</span> s</footer>

<script>
function fmtUptime(ms){return Math.floor(ms/1000);}
async function poll(){
  try{
    const r = await fetch('/data',{cache:'no-store'});
    const d = await r.json();

    document.getElementById('dot').classList.add('live');
    document.getElementById('conn').textContent = 'live';

    // SOS panic button - full red takeover, overrides everything else
    const sos = document.getElementById('sos');
    if(d.sos){
      sos.classList.add('show');
      document.title = '🆘 SOS - PATIENT EMERGENCY';
    }else{
      sos.classList.remove('show');
      if(document.title.indexOf('SOS') !== -1) document.title = 'Smart IV Drip Monitor';
    }

    // quality: 0 no finger, 1 calibrating, 2 weak signal, 3 good
    const q = d.quality|0;
    const pit = 'perfusion ' + (d.pi!==undefined ? d.pi : '--') + '%';
    if(!d.sensorOk){
      document.getElementById('spo2').textContent = '--';
      document.getElementById('bpm').textContent  = '--';
      document.getElementById('spo2sub').textContent = 'sensor not connected';
      document.getElementById('bpmsub').textContent  = 'sensor not connected';
    }else if(q>=3 && d.finger){
      document.getElementById('spo2').textContent = d.spo2;
      document.getElementById('bpm').textContent  = d.bpm;
      document.getElementById('spo2sub').textContent = (d.spo2>=95?'normal · '+pit:(d.spo2>0?'below normal · '+pit:pit));
      document.getElementById('bpmsub').textContent  = pit;
    }else if(q===2 && d.finger){
      document.getElementById('spo2').textContent = d.spo2>0?d.spo2:'--';
      document.getElementById('bpm').textContent  = d.bpm>0?d.bpm:'--';
      document.getElementById('spo2sub').textContent = 'weak signal — press finger gently, hold still';
      document.getElementById('bpmsub').textContent  = 'weak signal — ' + pit;
    }else if(q===1){
      document.getElementById('spo2').textContent = '--';
      document.getElementById('bpm').textContent  = '--';
      document.getElementById('spo2sub').textContent = 'calibrating… keep finger still';
      document.getElementById('bpmsub').textContent  = 'calibrating…';
    }else{
      document.getElementById('spo2').textContent = '--';
      document.getElementById('bpm').textContent  = '--';
      document.getElementById('spo2sub').textContent = 'place finger on sensor';
      document.getElementById('bpmsub').textContent  = 'place finger on sensor';
    }

    const fill = document.getElementById('fill');
    const pill = document.getElementById('salinePill');
    const banner = document.getElementById('banner');
    if(d.salineOk){
      fill.classList.remove('low'); fill.style.height='70%';
      document.getElementById('salineTxt').textContent = '> 30%';
      pill.className = 'pill ok'; pill.textContent = 'NORMAL';
      banner.classList.remove('show');
    }else{
      fill.classList.add('low'); fill.style.height='18%';
      document.getElementById('salineTxt').textContent = '≤ 30%';
      pill.className = 'pill low'; pill.textContent = 'LOW';
      banner.classList.add('show');
    }

    document.getElementById('uptime').textContent = fmtUptime(d.uptime);
  }catch(e){
    document.getElementById('dot').classList.remove('live');
    document.getElementById('conn').textContent = 'no signal';
  }
}

document.getElementById('sosClear').addEventListener('click', async function(){
  this.textContent = 'CLEARING...';
  try{
    await fetch('/sos/clear',{method:'POST'});
    document.getElementById('sos').classList.remove('show');   // hide right away
  }catch(e){}
  this.textContent = 'RESET SOS ALERT';
  poll();
});

poll();
setInterval(poll, 1000);
</script>
</body>
</html>
)HTML";

void setup() {
  Serial.begin(115200);

  pinMode(LIQUID_SENSOR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin(21, 22); // SDA, SCL

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("OLED not found. Check wiring/address.");
    while (true) { delay(10); }
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Smart IV Drip");
  display.println("Initializing...");
  display.display();

  setupWiFi();

#if SOS_BUTTON_MODE == 1
  // Digital touch module / push-button: pull the line to the inactive rail so
  // a disconnected wire can't float and false-trigger.
  pinMode(SOS_TOUCH_PIN, (SOS_DIGITAL_ACTIVE == HIGH) ? INPUT_PULLDOWN : INPUT_PULLUP);
  Serial.printf("SOS button: DIGITAL on GPIO%d, active %s\n",
                SOS_TOUCH_PIN, (SOS_DIGITAL_ACTIVE == HIGH) ? "HIGH" : "LOW");
#else
  // Bare capacitive pad: auto-calibrate its idle level (keep hands off it now).
  {
    long sum = 0; int n = 0;
    for (int i = 0; i < 20; i++) {
      int v = touchRead(SOS_TOUCH_PIN);
      if (v > 0) { sum += v; n++; }
      delay(15);
    }
    if (n > 0) gTouchBaseline = sum / n;
    if (gTouchBaseline < 5) gTouchBaseline = 60;   // fallback if pad reads 0
    Serial.printf("SOS button: CAPACITIVE on GPIO%d, idle baseline = %d\n",
                  SOS_TOUCH_PIN, gTouchBaseline);
  }
#endif

  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/sos/clear", handleSosClear);   // any method - clears the SOS latch
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.printf("Web server started -> http://%s/\n", gIpAddress.c_str());

  // Run the web server in its own task (core 0) so a slow / missing
  // MAX30102 acquisition on the main loop can never starve it.
  xTaskCreatePinnedToCore(webServerTask, "webServer", 8192, NULL, 1, NULL, 0);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    // Non-fatal: keep serving the dashboard even without the vitals sensor.
    Serial.println("MAX30102 not found. Check wiring. (dashboard still running)");
    gSensorOk = false;
  } else {
    configureSensor();
    gSensorOk = true;
    lastSampleMs = millis();

    // Prime the sliding window so the first SpO2 result is ready quickly.
    unsigned long t0 = millis();
    while (sampleCount < 100 && millis() - t0 < 6000) {
      pollSensor();
      delay(1);                 // yield to Wi-Fi / watchdog
    }
    Serial.printf("Sensor primed: %u samples\n", sampleCount);
  }

  delay(200);
}

void loop() {
  checkLiquidLevel();
  checkSosButton();
  updateAlarms();

  // Heartbeat state print - works even with the MAX30102 unplugged.
  static unsigned long lastDbg = 0;
  if (millis() - lastDbg > 1000) {
    lastDbg = millis();
    Serial.printf("[state] salinePresent=%d  sos=%d  sosRaw=%d  alarm=%s\n",
                  gSalinePresent, gSosActive, gSosRaw,
                  gSosActive ? "SOS" : (!gSalinePresent ? "SALINE" : "none"));
  }

  if (gSensorOk) {
    pollSensor();
  } else {
    gFingerDetected = false;
    gQualityCode = 0;
    static unsigned long lastTry = 0;
    if (millis() - lastTry > 1500) {          // retry a missing sensor, non-blocking
      lastTry = millis();
      if (particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        configureSensor();
        lastSampleMs = millis();
        gSensorOk = true;
        Serial.println("MAX30102 recovered.");
      }
    }
  }

  // Refresh faster while the SOS screen is up so its blink looks urgent.
  unsigned long interval = gSosActive ? 250 : DISPLAY_INTERVAL;
  if (millis() - lastDisplayUpdate >= interval) {
    lastDisplayUpdate = millis();
    updateDisplay(gFingerDetected);
  }
}

// Dedicated web-server task. Owns server.handleClient() exclusively.
void webServerTask(void *param) {
  for (;;) {
    server.handleClient();
    delay(2);
  }
}

// ---------------- WI-FI ----------------
void setupWiFi() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Smart IV Drip");

#if USE_AP_MODE
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  delay(100);
  bool apOk = WiFi.softAP(AP_SSID, AP_PASSWORD);
  delay(300);                       // let the AP + DHCP server come up
  gIpAddress = WiFi.softAPIP().toString();
  Serial.printf("SoftAP \"%s\" %s  IP: %s\n",
                AP_SSID, apOk ? "up" : "FAILED", gIpAddress.c_str());

  display.println("Wi-Fi hotspot:");
  display.println(AP_SSID);
  display.print("IP ");
  display.println(gIpAddress);
#else
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  display.println("Connecting WiFi..");
  display.display();

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(400);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    gIpAddress = WiFi.localIP().toString();
    Serial.printf("\nConnected. IP: %s\n", gIpAddress.c_str());
    display.println("WiFi connected");
    display.print("IP ");
    display.println(gIpAddress);
  } else {
    // Fall back to hotspot so the dashboard is always reachable.
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    gIpAddress = WiFi.softAPIP().toString();
    Serial.printf("\nWi-Fi failed. SoftAP \"%s\" IP: %s\n", AP_SSID, gIpAddress.c_str());
    display.println("WiFi failed - AP:");
    display.println(AP_SSID);
    display.print("IP ");
    display.println(gIpAddress);
  }
#endif

  display.display();
  delay(2500);
}

// ---------------- WEB HANDLERS ----------------
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

String buildJson() {
  String s = "{";
  s += "\"finger\":";  s += (gFingerDetected ? "true" : "false");
  s += ",\"spo2\":";   s += lastValidSpO2;
  s += ",\"bpm\":";    s += lastValidBPM;
  s += ",\"pi\":";     s += String(gPerfusionIndex, 2);
  s += ",\"quality\":"; s += gQualityCode;   // 0 none 1 calibrating 2 weak 3 good
  s += ",\"sensorOk\":"; s += (gSensorOk ? "true" : "false");
  s += ",\"sos\":";     s += (gSosActive ? "true" : "false");
  s += ",\"salineOk\":"; s += (gSalinePresent ? "true" : "false");
  s += ",\"alarm\":";   s += (gSalinePresent ? "false" : "true");
  s += ",\"salineMark\":"; s += SALINE_SENSOR_MARK_PERCENT;
  s += ",\"uptime\":";  s += millis();
  s += "}";
  return s;
}

void handleData() {
  server.send(200, "application/json", buildJson());
}

// Clears the latched SOS alert. Called by the "RESET SOS ALERT" button on the
// web dashboard's red SOS screen.
void handleSosClear() {
  gSosActive = false;
  Serial.println("SOS cleared from web dashboard");
  server.send(200, "text/plain", "cleared");
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// ---------------- VITALS ----------------

// MAX30102 acquisition setup - the proven SparkFun SpO2 configuration:
// 100 Hz / 4x average = 25 Hz effective, which is exactly what the SpO2
// algorithm (spo2_algorithm.cpp, FS = 25) is written for.
void configureSensor() {
  byte ledBrightness = 60;     // 0 = off .. 255 = 50 mA
  byte sampleAverage = 4;      // 1, 2, 4, 8, 16, 32
  byte ledMode       = 2;      // 2 = Red + IR (needed for SpO2)
  int  sampleRate    = 100;    // Hz
  int  pulseWidth    = 411;    // us
  int  adcRange      = 4096;

  // setup() already sets Red and IR amplitude to ledBrightness; leave them.
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate,
                       pulseWidth, adcRange);

  sampleCount = 0;
  newSinceCalc = 0;
  spo2HistN = spo2HistIdx = 0; spo2Has = false;
  hrHistN = hrHistIdx = 0; hrHas = false;
  for (byte x = 0; x < RATE_SIZE; x++) rates[x] = 0;
  beatAvgBpm = 0;
  prevFingerPresent = false;
}

// median-of-5 on a tiny ring buffer (insertion sort, n <= 5)
static int medianPush(int *hist, int &n, int &idx, int v) {
  hist[idx] = v;
  idx = (idx + 1) % 5;
  if (n < 5) n++;
  int t[5];
  for (int i = 0; i < n; i++) t[i] = hist[i];
  for (int i = 1; i < n; i++) {
    int k = t[i], j = i - 1;
    while (j >= 0 && t[j] > k) { t[j + 1] = t[j]; j--; }
    t[j + 1] = k;
  }
  return t[n / 2];
}

// Called once per loop() pass. Pulls at most one fresh sample, feeds the
// real-time beat detector, keeps the sliding window current, and triggers a
// SpO2 recompute every ~25 fresh samples (about once a second).
void pollSensor() {
  particleSensor.check();
  if (!particleSensor.available()) {
    return;                       // no new FIFO sample yet - just wait
  }
  lastSampleMs = millis();

  uint32_t red = particleSensor.getRed();
  uint32_t ir  = particleSensor.getIR();
  particleSensor.nextSample();

  if (sampleCount < 100) {
    irBuffer[sampleCount]  = ir;
    redBuffer[sampleCount] = red;
    sampleCount++;
  } else {
    memmove(irBuffer,  irBuffer  + 1, 99 * sizeof(uint32_t));
    memmove(redBuffer, redBuffer + 1, 99 * sizeof(uint32_t));
    irBuffer[99]  = ir;
    redBuffer[99] = red;
  }
  if (newSinceCalc < 1000) newSinceCalc++;

  // Real-time heart beat: needs true wall-clock spacing between beats.
  if (checkForBeat(ir)) {
    unsigned long now = millis();
    unsigned long delta = now - lastBeatMs;
    lastBeatMs = now;
    if (delta > 300 && delta < 2000) {          // 30..200 bpm plausible
      int bpm = (int)(60000.0 / delta);
      rates[rateSpot++] = (byte)bpm;
      rateSpot %= RATE_SIZE;
      unsigned int tot = 0; byte cnt = 0;
      for (byte x = 0; x < RATE_SIZE; x++) if (rates[x]) { tot += rates[x]; cnt++; }
      if (cnt >= 3) beatAvgBpm = tot / cnt;
    }
  }

  checkLiquidLevel();
  checkSosButton();
  updateAlarms();

  if (sampleCount >= 100 && newSinceCalc >= 25) {
    newSinceCalc = 0;
    recomputeVitals();
  }
}

// Runs the SpO2 algorithm over the window, judges signal quality, then
// filters + calibrates the accepted numbers.
void recomputeVitals() {
  uint32_t irMin = irBuffer[0], irMax = irBuffer[0];
  double irSum = 0;
  for (int i = 0; i < 100; i++) {
    uint32_t v = irBuffer[i];
    if (v < irMin) irMin = v;
    if (v > irMax) irMax = v;
    irSum += v;
  }
  double irMean = irSum / 100.0;
  gPerfusionIndex = (irMean > 0) ? (float)((irMax - irMin) / irMean * 100.0) : 0.0f;

  bool fingerPresent = (irMean > IR_FINGER_THRESHOLD);

  // Diagnostics - open the serial monitor at 115200 to see live values.
  Serial.printf("IRmean=%.0f  PI=%.2f%%  finger=%d  spo2=%ld(v%d)  hrAlg=%ld(v%d)  beat=%d\n",
                irMean, gPerfusionIndex, fingerPresent,
                (long)spo2, validSPO2, (long)heartRate, validHeartRate, beatAvgBpm);

  if (fingerPresent && !prevFingerPresent) {          // finger just placed
    fingerSinceMs = millis();
    spo2HistN = spo2HistIdx = 0; spo2Has = false;
    hrHistN = hrHistIdx = 0; hrHas = false;
    for (byte x = 0; x < RATE_SIZE; x++) rates[x] = 0;
    beatAvgBpm = 0;
  }
  prevFingerPresent = fingerPresent;

  if (!fingerPresent) {
    gFingerDetected = false;
    gQualityCode = 0;
    return;
  }

  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer,
                                         &spo2, &validSPO2, &heartRate,
                                         &validHeartRate);

  bool warming = (millis() - fingerSinceMs < WARMUP_MS);
  bool weak    = (gPerfusionIndex < MIN_PERFUSION_INDEX);

  gFingerDetected = true;
  gQualityCode = warming ? 1 : (weak ? 2 : 3);
  if (warming) return;                                // let the signal settle first

  // ---- SpO2: gate -> median -> EMA -> calibrate ----
  if (validSPO2 && spo2 >= SPO2_MIN_VALID && spo2 <= SPO2_MAX_VALID) {
    int med = medianPush(spo2Hist, spo2HistN, spo2HistIdx, (int)spo2);
    spo2Ema = spo2Has ? (SPO2_EMA_ALPHA * med + (1 - SPO2_EMA_ALPHA) * spo2Ema) : med;
    spo2Has = true;
    int out = (int)lround(spo2Ema * SPO2_CAL_GAIN + SPO2_CAL_OFFSET);
    lastValidSpO2 = constrain(out, 0, 100);
  }

  // ---- Heart rate: prefer the real-time beat detector, fall back to algo ----
  int hrCandidate = -1;
  bool recentBeat = (millis() - lastBeatMs < 4000);
  if (recentBeat && beatAvgBpm >= HR_MIN_VALID && beatAvgBpm <= HR_MAX_VALID) {
    hrCandidate = beatAvgBpm;
  } else if (validHeartRate && heartRate >= HR_MIN_VALID && heartRate <= HR_MAX_VALID) {
    hrCandidate = (int)heartRate;
  }
  if (hrCandidate > 0) {
    int med = medianPush(hrHist, hrHistN, hrHistIdx, hrCandidate);
    hrEma = hrHas ? (HR_EMA_ALPHA * med + (1 - HR_EMA_ALPHA) * hrEma) : med;
    hrHas = true;
    int out = (int)lround(hrEma * HR_CAL_GAIN + HR_CAL_OFFSET);
    lastValidBPM = constrain(out, 0, 250);
  }
}

void updateDisplay(bool fingerDetected) {
  display.clearDisplay();

  // ---------- SOS CRITICAL: highest priority, flashing full-screen alert ------
  if (gSosActive) {
    static bool flip = false;
    flip = !flip;
    uint16_t bg = flip ? SSD1306_WHITE : SSD1306_BLACK;
    uint16_t fg = flip ? SSD1306_BLACK : SSD1306_WHITE;

    display.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, bg);
    display.setTextColor(fg);
    display.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, fg);
    display.drawRect(2, 2, SCREEN_WIDTH - 4, SCREEN_HEIGHT - 4, fg);

    display.setTextSize(3);
    display.setCursor(37, 6);              // "SOS" ~ 54 px wide, centred
    display.print("SOS");

    display.setTextSize(1);
    display.setCursor(9, 36);
    display.print("PATIENT NEEDS HELP");
    display.setCursor(30, 48);
    display.print("CALL NURSE");

    display.setTextColor(SSD1306_WHITE);
    display.display();
    return;
  }

  // ---------- SALINE EMERGENCY: full white screen, black text only ----------
  if (!gSalinePresent) {
    display.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);

    display.setTextSize(2);
    display.setCursor(8, 4);
    display.print("EMERGENCY");

    display.drawLine(0, 22, SCREEN_WIDTH, 22, SSD1306_BLACK);

    display.setTextSize(1);
    display.setCursor(4, 28);
    display.print("SALINE LEVEL <= 30%");
    display.setCursor(4, 40);
    display.print("REPLACE IV BOTTLE");
    display.setCursor(4, 52);
    display.print("!! CHECK PATIENT !!");

    display.setTextColor(SSD1306_WHITE);
    display.display();
    return;
  }

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Smart IV Drip Monitor");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  bool calibrating = (gQualityCode == 1);
  bool showNumbers = fingerDetected && !calibrating;

  display.setTextSize(2);
  display.setCursor(0, 16);
  if (showNumbers) {
    display.print("SpO2-");
    display.print(lastValidSpO2);
    display.println("%");
  } else {
    display.println("SpO2- --");
  }

  display.setCursor(0, 40);
  if (showNumbers) {
    display.print("BPM-");
    display.println(lastValidBPM);
  } else {
    display.println("BPM- --");
  }

  display.display();
}

bool isLiquidPresent() {
  int reading = digitalRead(LIQUID_SENSOR_PIN);
  return (reading == LIQUID_DETECTED_LEVEL);
}

// Non-blocking buzzer driver. Uses tone() so it works with BOTH a passive
// piezo and an active buzzer. Each alarm has its own sound signature:
//   SALINE : 2 kHz  beep-beep-beep .... pause   (repeating)
//   SOS    : hi/lo two-tone wail, never silent  (repeating)
void updateAlarm(AlarmMode mode) {
  static AlarmMode prev = ALARM_NONE;
  static unsigned long phaseStart = 0;
  static int lastFreq = -1;

  if (mode != prev) { prev = mode; phaseStart = millis(); lastFreq = -1; }

  unsigned int freq = 0;                      // 0 = silent
  unsigned long t = millis() - phaseStart;

  if (mode == ALARM_SALINE) {
    unsigned long c = t % 1600;               // 1.6 s repeating cycle
    if      (c < 150) freq = 2000;
    else if (c < 270) freq = 0;
    else if (c < 420) freq = 2000;
    else if (c < 540) freq = 0;
    else if (c < 690) freq = 2000;
    else              freq = 0;               // long rest
  } else if (mode == ALARM_SOS) {
    unsigned long c = t % 700;                // fast wail, always sounding
    freq = (c < 350) ? 2700 : 1600;
  }

  if ((int)freq != lastFreq) {
    lastFreq = freq;
    if (freq == 0) { noTone(BUZZER_PIN); digitalWrite(BUZZER_PIN, LOW); }
    else           { tone(BUZZER_PIN, freq); }
  }
}

// Central alarm arbiter - SOS wins over the saline alarm.
void updateAlarms() {
  AlarmMode mode = ALARM_NONE;
  if (gSosActive)           mode = ALARM_SOS;
  else if (!gSalinePresent) mode = ALARM_SALINE;
  updateAlarm(mode);
}

void checkLiquidLevel() {
  gSalinePresent = isLiquidPresent();
}

// Reads the SOS button (digital module or bare capacitive pad, per
// SOS_BUTTON_MODE) and debounces it. A quick tap LATCHES gSosActive on.
// The button never clears it - that is done from the web dashboard
// (POST /sos/clear, the "RESET SOS ALERT" button on the red SOS screen).
void checkSosButton() {
  static unsigned long lastRead = 0;
  static unsigned long firstMs = 0;
  static uint8_t  cnt = 0;                 // debounce accumulator (0..6)
  static bool wasTouching = false;
  static unsigned long lastDbg = 0;

  if (millis() - lastRead < 25) return;    // sample ~40x/sec
  lastRead = millis();
  if (firstMs == 0) firstMs = millis();

  // Brief settle window after boot so nothing false-fires during power-on.
  bool settling = (millis() - firstMs < 1500);
  bool pressed  = false;

#if SOS_BUTTON_MODE == 1
  // ---- digital touch module (TTP223) or push-button ----
  int raw = digitalRead(SOS_TOUCH_PIN);
  gSosRaw = raw;
  pressed = (raw == SOS_DIGITAL_ACTIVE);
  if (millis() - lastDbg > 500) {
    lastDbg = millis();
    Serial.printf("[sos] pin=%d pressed=%d cnt=%d sos=%d%s\n",
                  raw, pressed, cnt, gSosActive, settling ? " (settling)" : "");
  }
#else
  // ---- bare capacitive pad on the ESP32 touch peripheral ----
  int raw = touchRead(SOS_TOUCH_PIN);
  gSosRaw = raw;
  int need = gTouchBaseline * SOS_TOUCH_DROP_PCT / 100;
  if (need < 3) need = 3;
  int dev = abs(raw - gTouchBaseline);
  pressed = (raw > 0 && dev >= need);
  if (settling) {
    if (raw > 0) gTouchBaseline += (raw - gTouchBaseline) / 4;   // lock onto idle
  } else if (cnt == 0 && raw > 0) {
    gTouchBaseline += (raw - gTouchBaseline) / 16;               // self-heal drift
  }
  if (millis() - lastDbg > 500) {
    lastDbg = millis();
    Serial.printf("[sos] raw=%d base=%d dev=%d need=%d cnt=%d sos=%d%s\n",
                  raw, gTouchBaseline, dev, need, cnt, gSosActive,
                  settling ? " (settling)" : "");
  }
#endif

  if (settling) { cnt = 0; wasTouching = false; return; }

  // ---- debounce, then latch on the press edge only ----
  if (pressed) { if (cnt < 6) cnt++; }
  else         { if (cnt > 0) cnt--; }
  bool touching = (cnt >= 4);

  if (touching && !wasTouching && !gSosActive) {
    gSosActive = true;
    Serial.println("SOS BUTTON PRESSED - patient emergency");
  }
  wasTouching = touching;
}

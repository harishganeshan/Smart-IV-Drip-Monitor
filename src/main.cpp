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
     OUT (NPN/PNP)  -> GPIO 4   (through a voltage divider, see note below)

   Buzzer -> ESP32
     +              -> GPIO 25
     -              -> GND

  IMPORTANT NOTE ON THE LIQUID SENSOR OUTPUT VOLTAGE:
   ESP32 GPIOs are NOT 5V/12V/24V tolerant. If you run the sensor at 5V
   in NPN mode, use a 10k/20k voltage divider from OUT to GPIO4 to bring
   5V down to ~3.3V. If using 12V/24V or PNP mode, use a level shifter
   or optocoupler instead.
  =====================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

// ---------------- OLED CONFIG ----------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDRESS  0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------------- MAX30102 CONFIG ----------------
MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255
uint32_t irBuffer[100];
uint32_t redBuffer[100];
int32_t bufferLength = 100;
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;

#define IR_FINGER_THRESHOLD 50000

// ---------------- LIQUID LEVEL SENSOR CONFIG ----------------
#define LIQUID_SENSOR_PIN 4

// Set this after testing your sensor:
// If sensor output goes LOW when liquid IS present, keep this as LOW.
// If it goes HIGH when liquid IS present, change to HIGH.
#define LIQUID_DETECTED_LEVEL LOW

// ---------------- BUZZER CONFIG ----------------
#define BUZZER_PIN 25

// ---------------- TIMING ----------------
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL = 1000;

int lastValidBPM = 0;
int lastValidSpO2 = 0;

void checkLiquidLevel();
bool isLiquidPresent();
void readAndCalculateVitals();
void updateDisplay(bool fingerDetected);

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

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found. Check wiring.");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("MAX30102 ERROR");
    display.display();
    while (true) { delay(10); }
  }

  byte ledBrightness = 60;
  byte sampleAverage = 4;
  byte ledMode = 2;
  int sampleRate = 100;
  int pulseWidth = 411;
  int adcRange = 4096;

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate,
                        pulseWidth, adcRange);

  delay(500);
}

void loop() {
  checkLiquidLevel();
  readAndCalculateVitals();
}

void readAndCalculateVitals() {
  for (byte i = 0; i < bufferLength; i++) {
    while (particleSensor.available() == false) {
      particleSensor.check();
    }
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();

    checkLiquidLevel();
  }

  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer,
                                          &spo2, &validSPO2, &heartRate,
                                          &validHeartRate);

  bool fingerDetected = (irBuffer[bufferLength - 1] > IR_FINGER_THRESHOLD);

  if (fingerDetected && validSPO2 && validHeartRate) {
    lastValidSpO2 = spo2;
    lastValidBPM = heartRate;
  }

  if (millis() - lastDisplayUpdate >= DISPLAY_INTERVAL) {
    lastDisplayUpdate = millis();
    updateDisplay(fingerDetected);
  }
}

void updateDisplay(bool fingerDetected) {
  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Smart IV Drip Monitor");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(0, 16);
  if (fingerDetected) {
    display.print("SpO2-");
    display.print(lastValidSpO2);
    display.println("%");
  } else {
    display.println("SpO2- --");
  }

  display.setCursor(0, 40);
  if (fingerDetected) {
    display.print("BPM-");
    display.println(lastValidBPM);
  } else {
    display.println("BPM- --");
  }

  bool liquidPresent = isLiquidPresent();
  if (!liquidPresent) {
    display.fillRect(0, 56, 128, 8, SSD1306_WHITE);
    display.setTextSize(1);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(2, 57);
    display.print("!! NO FLUID / CHECK IV !!");
    display.setTextColor(SSD1306_WHITE);
  }

  display.display();
}

bool isLiquidPresent() {
  int reading = digitalRead(LIQUID_SENSOR_PIN);
  return (reading == LIQUID_DETECTED_LEVEL);
}

void checkLiquidLevel() {
  if (!isLiquidPresent()) {
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
}
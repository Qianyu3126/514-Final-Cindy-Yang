#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#define SDA_PIN 6
#define SCL_PIN 7

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

float zOffset = 0.0;
const int CALIB_SAMPLES = 200;

const float MOTION_THRESHOLD = 3.0;  // m/s² delta above 1G to count as intentional movement
const int   CONFIRM_SAMPLES  = 3;    // consecutive samples that must exceed threshold
int   triggerCount = 0;
bool  goalPrinted  = false;          // prevents spam printing

void calibrateZ();
float readRawZ();

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!accel.begin()) {
    Serial.println("[ERROR] ADXL345 not found! Check wiring:");
    Serial.println("  SDA -> D4 (GPIO6)");
    Serial.println("  SCL -> D5 (GPIO7)");
    Serial.println("  VCC -> 3.3V | GND -> GND");
    while (1) delay(500);
  }

  accel.setRange(ADXL345_RANGE_2_G);
  Serial.println("[OK] ADXL345 initialized, range: +-2G");

  calibrateZ();

  Serial.println("========================================");
  Serial.println("Monitoring Z-axis for intentional movement...");
  Serial.print("Trigger threshold: +-");
  Serial.print(MOTION_THRESHOLD, 1);
  Serial.println(" m/s² from 1G");
  Serial.println("========================================");
}

void loop() {
  float rawZ   = readRawZ();
  float calibZ = rawZ - zOffset;             
  float delta  = abs(calibZ - 9.80665);       // deviation from resting 1G

  if (delta > MOTION_THRESHOLD) {
    triggerCount++;
    goalPrinted = false;                       // reset so it can fire again next gesture
  } else {
    if (triggerCount >= CONFIRM_SAMPLES && !goalPrinted) {
      Serial.println("Move goal completed!");
      goalPrinted = true;
    }
    triggerCount = 0;
  }

  delay(20); // 50 Hz polling
}

void calibrateZ() {
  Serial.println("[Calibration] Keep sensor still, lying flat...");
  Serial.println("[Calibration] Starting in 3 seconds...");
  delay(3000);

  Serial.print("[Calibration] Sampling");
  double sum = 0.0;
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    sensors_event_t event;
    accel.getEvent(&event);
    sum += event.acceleration.z;
    if (i % 20 == 0) Serial.print(".");
    delay(10);
  }

  float rawMean = sum / CALIB_SAMPLES;
  zOffset = rawMean - 9.80665;

  Serial.println(" Done!");
  Serial.print("[Calibration] Raw mean: ");
  Serial.print(rawMean, 4);
  Serial.println(" m/s2");
  Serial.print("[Calibration] Offset applied: ");
  Serial.print(zOffset, 4);
  Serial.println(" m/s2\n");
}

float readRawZ() {
  sensors_event_t event;
  accel.getEvent(&event);
  return event.acceleration.z;
}
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <math.h>

#define SDA_PIN 6
#define SCL_PIN 7

#define SERVICE_UUID        "c5878e5f-86f5-4949-8569-d3a541ea920a"
#define CHARACTERISTIC_UUID "71450645-7a45-491f-878c-2485e3f48d15"

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
float yOffset = 0.0;
const int CALIB_SAMPLES = 200;

// ── DSP 1: Moving Average Filter ──────────────────────────
const int MA_WINDOW = 5;
float maBuffer[MA_WINDOW] = {0};
int   maIndex = 0;
bool  maFull  = false;

float movingAverage(float newVal) {
  maBuffer[maIndex] = newVal;
  maIndex = (maIndex + 1) % MA_WINDOW;
  if (maIndex == 0) maFull = true;
  int count = maFull ? MA_WINDOW : maIndex;
  float sum = 0;
  for (int i = 0; i < count; i++) sum += maBuffer[i];
  return sum / count;
}

// ── DSP 2: Z-score Anomaly Detection ─────────────────────
const int   ZSCORE_WINDOW   = 50;
const float Z_THRESHOLD     = 3.0;
const float MIN_DELTA       = 1.5;  // m/s², minimum actual movement
const int   CONFIRM_SAMPLES = 3;

float zBuffer[ZSCORE_WINDOW] = {0};
int   zIndex = 0;
bool  zFull  = false;

float computeZScore(float newVal) {
  zBuffer[zIndex] = newVal;
  zIndex = (zIndex + 1) % ZSCORE_WINDOW;
  if (zIndex == 0) zFull = true;

  // Don't compute until buffer is fully warmed up
  if (!zFull) return 0.0;

  float sum = 0;
  for (int i = 0; i < ZSCORE_WINDOW; i++) sum += zBuffer[i];
  float mean = sum / ZSCORE_WINDOW;

  float variance = 0;
  for (int i = 0; i < ZSCORE_WINDOW; i++) {
    float diff = zBuffer[i] - mean;
    variance += diff * diff;
  }
  float stdDev = sqrt(variance / ZSCORE_WINDOW);

  if (stdDev < 0.1) return 0.0;
  return abs(newVal - mean) / stdDev;
}

// ── Stand detection state ─────────────────────────────────
int  triggerCount = 0;
bool goalSent     = false;

// ── BLE ───────────────────────────────────────────────────
BLEServer*         pServer         = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected    = false;
bool oldDeviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("[BLE] Display device connected!");
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("[BLE] Display device disconnected");
  }
};

void calibrateY() {
  Serial.println("[Calibration] Keep still for 3 seconds...");
  delay(3000);
  double sum = 0.0;
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    sensors_event_t event;
    accel.getEvent(&event);
    sum += event.acceleration.y;
    delay(10);
  }
  yOffset = sum / CALIB_SAMPLES;
  Serial.print("[Calibration] Y offset: ");
  Serial.println(yOffset);
}

void setup() {
  Serial.begin(115200);
  Serial.println("=== Sensing Device (Server) Starting ===");

  Wire.begin(SDA_PIN, SCL_PIN);
  if (!accel.begin()) {
    Serial.println("[ERROR] ADXL345 not found!");
    while (1);
  }
  Serial.println("[OK] ADXL345 found");
  accel.setRange(ADXL345_RANGE_2_G);
  calibrateY();

  BLEDevice::init("MurfCin_Sensing");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setValue("Waiting...");
  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("[BLE] Advertising as MurfCin_Sensing...");
}

void loop() {

  sensors_event_t event;
  accel.getEvent(&event);
  float rawY = event.acceleration.y - yOffset;

  // DSP Step 1: smooth the signal
  float smoothedY = movingAverage(rawY);

  // DSP Step 2: z-score anomaly detection
  float zScore = computeZScore(smoothedY);

  static int printCounter = 0;
  if (++printCounter >= 50) {
    Serial.print("[Y] raw=");
    Serial.print(rawY, 2);
    Serial.print("  smoothed=");
    Serial.print(smoothedY, 2);
    Serial.print("  z=");
    Serial.print(zScore, 2);
    Serial.print("  triggerCount=");
    Serial.print(triggerCount);
    Serial.print("  connected=");
    Serial.println(deviceConnected);
    printCounter = 0;
  }

  // Both z-score AND minimum movement must be satisfied
  if (zScore > Z_THRESHOLD && abs(smoothedY) > MIN_DELTA) {
    triggerCount++;
    goalSent = false;
    Serial.print("[Motion] z=");
    Serial.print(zScore, 2);
    Serial.print(" smoothedY=");
    Serial.print(smoothedY, 2);
    Serial.print(" count=");
    Serial.println(triggerCount);
  } else {
    if (triggerCount >= CONFIRM_SAMPLES && !goalSent && deviceConnected) {
      Serial.println("[BLE] Sending STAND_GOAL via notify...");
      pCharacteristic->setValue("STAND_GOAL");
      pCharacteristic->notify();
      goalSent = true;
      Serial.println("[BLE] Sent!");
    }
    triggerCount = 0;
  }

  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("[BLE] Restarting advertising...");
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  delay(20);
}
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#define I2C_SDA     6
#define I2C_SCL     7
#define NEO_PIN     10
#define NEO_COUNT   1
#define MOTOR_PIN1  2
#define MOTOR_PIN2  1
#define MOTOR_PIN3  3
#define MOTOR_PIN4  4

#define SERVICE_UUID        "c5878e5f-86f5-4949-8569-d3a541ea920a"
#define CHARACTERISTIC_UUID "71450645-7a45-491f-878c-2485e3f48d15"

Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_NeoPixel pixel(NEO_COUNT, NEO_PIN, NEO_RGB + NEO_KHZ800);

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan    = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice*     myDevice;

volatile bool goalReceived   = false;
unsigned long seatTimerStart = 0;
unsigned long lastScanTime   = 0;
int  lastSecs    = -1;
bool motorMoved  = false;
bool alertMode   = false;

// 
const int STEP_SEQ[8][4] = {
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1}
};
int currentStep = 0;
const int MOTOR_STEPS = 45 * 8;

void applyStep() {
  digitalWrite(MOTOR_PIN1, STEP_SEQ[currentStep][0]);
  digitalWrite(MOTOR_PIN2, STEP_SEQ[currentStep][1]);
  digitalWrite(MOTOR_PIN3, STEP_SEQ[currentStep][2]);
  digitalWrite(MOTOR_PIN4, STEP_SEQ[currentStep][3]);
  delay(5);
}

void releaseMotor() {
  digitalWrite(MOTOR_PIN1, LOW);
  digitalWrite(MOTOR_PIN2, LOW);
  digitalWrite(MOTOR_PIN3, LOW);
  digitalWrite(MOTOR_PIN4, LOW);
}

void rotateCW() {
  for (int i = 0; i < MOTOR_STEPS; i++) {
    currentStep = (currentStep + 1) % 8;
    applyStep();
  }
  delay(100);
  releaseMotor();
}

void rotateCCW() {
  for (int i = 0; i < MOTOR_STEPS; i++) {
    currentStep = (currentStep - 1 + 8) % 8;
    applyStep();
  }
  delay(100);
  releaseMotor();
}

// ── NeoPixel helpers
void flashBlue(int times) {
  for (int i = 0; i < times; i++) {
    pixel.setPixelColor(0, pixel.Color(0, 0, 255));
    pixel.show();
    delay(200);
    pixel.setPixelColor(0, pixel.Color(0, 0, 0));
    pixel.show();
    delay(200);
  }
  pixel.setPixelColor(0, pixel.Color(0, 0, 255));
  pixel.show();
}

// ── Display helpers
void showTime(int secs) {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(28, 20);
  display.print("Seating time:");
  display.setTextSize(2);
  display.setCursor(40, 38);
  display.print(secs);
  display.print(" sec");
  display.display();
}

void showAlert() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(8, 20);
  display.print("TIME TO");
  display.setCursor(16, 42);
  display.print("STAND!");
  display.display();
}

void showGoalCompleted() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(8, 8);
  display.print("STAND");
  display.setCursor(8, 28);
  display.print("GOAL");
  display.setCursor(8, 48);
  display.print("DONE!");
  display.display();
}

// 
void resetToIdle() {
  seatTimerStart = millis();
  lastSecs       = -1;
  alertMode      = false;
  goalReceived   = false;
  motorMoved     = false;  
  pixel.setPixelColor(0, pixel.Color(0, 0, 255));
  pixel.show();
  showTime(0);
}

// ── BLE callbacks
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    String value = "";
    for (int i = 0; i < length; i++) value += (char)pData[i];
    Serial.print("[BLE] Received: ");
    Serial.println(value);
    if (value == "STAND_GOAL") goalReceived = true;
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {}
  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("[BLE] Disconnected");
  }
};

bool connectToServer() {
  Serial.println("[BLE] Connecting...");
  BLEClient* pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());
  pClient->connect(myDevice);
  pClient->setMTU(517);

  BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
  if (pRemoteService == nullptr) {
    pClient->disconnect();
    return false;
  }

  pRemoteCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID);
  if (pRemoteCharacteristic == nullptr) {
    pClient->disconnect();
    return false;
  }

  if (pRemoteCharacteristic->canNotify())
    pRemoteCharacteristic->registerForNotify(notifyCallback);

  connected = true;
  Serial.println("[BLE] Connected!");
  flashBlue(3);
  return true;
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.haveServiceUUID() &&
        advertisedDevice.isAdvertisingService(BLEUUID(SERVICE_UUID))) {
      BLEDevice::getScan()->stop();
      myDevice  = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan    = true;
    }
  }
};

// ══════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("=== Display Device Starting ===");

  pixel.begin();
  pixel.setBrightness(200);
  pixel.setPixelColor(0, pixel.Color(0, 0, 255));
  pixel.show();

  Wire.begin(I2C_SDA, I2C_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  pinMode(MOTOR_PIN3, OUTPUT);
  pinMode(MOTOR_PIN4, OUTPUT);
  releaseMotor();

  BLEDevice::init("");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);

  seatTimerStart = millis();
  showTime(0);
}

// ══════════════════════════════════════════════════════════
void loop() {

  if (doConnect) {
    connectToServer();
    doConnect = false;
  }

  if (!connected && doScan) {
    unsigned long now = millis();
    if (now - lastScanTime > 10000) {
      lastScanTime = now;
      Serial.println("[BLE] Rescanning...");
      BLEDevice::getScan()->start(2, false);
    }
  }

  // ── Goal received ─────────────────────────────────────
  if (goalReceived) {
    showGoalCompleted();
    pixel.setPixelColor(0, pixel.Color(100, 255, 0));
    pixel.show();
    if (motorMoved) {
      rotateCCW();
      motorMoved = false;  
    }
    delay(2000);
    resetToIdle();
    return;
  }

  int elapsed = (int)((millis() - seatTimerStart) / 1000);

  // ── 20s alert ─────────────────────────────────────────
  if (elapsed >= 20) {
    if (!alertMode) {
      alertMode = true;
      showAlert();
    }
    pixel.setPixelColor(0, pixel.Color(255, 0, 0));
    pixel.show();
    delay(400);
    pixel.setPixelColor(0, pixel.Color(0, 0, 0));
    pixel.show();
    delay(400);
    return;
  }

  // ── 10s: ─────────────────────────
  if (elapsed >= 10 && !motorMoved) {
    motorMoved = true;  
    rotateCW();
  }

  // ── Timer display ─────────────────────────────────────
  if (elapsed != lastSecs) {
    lastSecs = elapsed;
    showTime(elapsed);
  }
}
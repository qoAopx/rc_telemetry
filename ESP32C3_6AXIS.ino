/*
  ESP32-C3 + BMI160 6-axis IMU
  --------------------------------
  旧GY-61(ADXL335)版からBMI160へ変更。

  変更しない部分:
    - BLE Service / Characteristic UUID
    - BLEデバイス名
    - BLE接続/切断処理
    - OLED表示
    - SWITCH_PIN
    - BLE送信フォーマットの基本構造

  BMI160:
    I2C SDA = GPIO5
    I2C SCL = GPIO6
    SA0/SDO = VDDIO -> I2C address 0x69
    CSB     = VDDIO -> I2C mode

  BMI160 data:
    accel X/Y/Z : 16-bit signed, ±2g, 16384 LSB/g
    gyro  X/Y/Z : 16-bit signed, ±2000 dps, 16.4 LSB/dps

  BMI160の生データからRoll/Pitchを加速度で計算します。
  ジャイロ値も読み取り、BLEには従来のX/Y/Z + Roll/Pitchに加えて
  GX/GY/GZを追加しています。
*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <U8g2lib.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// --- ピン割り当て設定 ---
#define SWITCH_PIN 3
#define SDA_PIN 5
#define SCL_PIN 6

// --- BLE設定（従来のまま） ---
#define BLE_NAME "RC_CAR_TELEMETRY"
#define SERVICE_UUID "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcdefab-1234-5678-1234-abcdefabcdef"

// --- BMI160 I2C ---
#define BMI160_ADDR 0x69   // 回路図では SA0/SDO が VDD にプルアップ
#define BMI160_CHIP_ID 0xD1

// BMI160 registers
#define REG_CHIP_ID   0x00
#define REG_GYR_X_L   0x0C
#define REG_ACC_X_L   0x12
#define REG_ACC_CONF  0x40
#define REG_ACC_RANGE 0x41
#define REG_GYR_CONF  0x42
#define REG_GYR_RANGE 0x43
#define REG_CMD       0x7E

// BMI160 commands
#define CMD_ACC_NORMAL 0x11
#define CMD_GYR_NORMAL 0x15

// --- グローバル変数 ---
BLECharacteristic *pCharacteristic;
BLEServer *pServer = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

float fX = 0.0f, fY = 0.0f, fZ = 1.0f;
float fGX = 0.0f, fGY = 0.0f, fGZ = 0.0f;
const float filterAlpha = 0.5f;

U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2(
  U8G2_R0, /* reset=*/U8X8_PIN_NONE, SCL_PIN, SDA_PIN
);

// -----------------------------------------------------------------------------
// BMI160 low-level I2C
// -----------------------------------------------------------------------------

bool bmi160WriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool bmi160ReadRegs(uint8_t reg, uint8_t *data, size_t len) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  size_t received = Wire.requestFrom((int)BMI160_ADDR, (int)len);
  if (received != len) return false;

  for (size_t i = 0; i < len; i++) {
    data[i] = Wire.read();
  }
  return true;
}

uint8_t bmi160ReadReg(uint8_t reg) {
  uint8_t value = 0;
  bmi160ReadRegs(reg, &value, 1);
  return value;
}

int16_t makeInt16(uint8_t lo, uint8_t hi) {
  return (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
}

bool bmi160Init() {
  delay(50);

  uint8_t chipId = bmi160ReadReg(REG_CHIP_ID);
  Serial.printf("BMI160 CHIP_ID: 0x%02X\n", chipId);

  if (chipId != BMI160_CHIP_ID) {
    Serial.println("ERROR: BMI160 not detected.");
    Serial.println("Check SDA/SCL, VDD/VDDIO, CSB and SA0.");
    return false;
  }

  // Accelerometer: normal mode
  if (!bmi160WriteReg(REG_CMD, CMD_ACC_NORMAL)) return false;
  delay(5);

  // Gyroscope: normal mode
  if (!bmi160WriteReg(REG_CMD, CMD_GYR_NORMAL)) return false;
  delay(80);

  // ACC_CONF:
  // ODR = 100 Hz (0x08), normal bandwidth setting (0x02)
  // 0x28 is a commonly used 100 Hz / normal bandwidth setting.
  if (!bmi160WriteReg(REG_ACC_CONF, 0x28)) return false;

  // ACC_RANGE = ±2g
  if (!bmi160WriteReg(REG_ACC_RANGE, 0x03)) return false;

  // GYR_CONF:
  // ODR = 100 Hz (0x08), normal bandwidth setting (0x02)
  if (!bmi160WriteReg(REG_GYR_CONF, 0x28)) return false;

  // GYR_RANGE = ±2000 deg/s
  if (!bmi160WriteReg(REG_GYR_RANGE, 0x00)) return false;

  delay(20);
  Serial.println("BMI160 initialization complete.");
  return true;
}

bool bmi160Read(float &ax, float &ay, float &az,
                float &gx, float &gy, float &gz,
                int16_t &axRaw, int16_t &ayRaw, int16_t &azRaw,
                int16_t &gxRaw, int16_t &gyRaw, int16_t &gzRaw) {
  uint8_t acc[6];
  uint8_t gyro[6];

  if (!bmi160ReadRegs(REG_ACC_X_L, acc, 6)) return false;
  if (!bmi160ReadRegs(REG_GYR_X_L, gyro, 6)) return false;

  axRaw = makeInt16(acc[0], acc[1]);
  ayRaw = makeInt16(acc[2], acc[3]);
  azRaw = makeInt16(acc[4], acc[5]);

  gxRaw = makeInt16(gyro[0], gyro[1]);
  gyRaw = makeInt16(gyro[2], gyro[3]);
  gzRaw = makeInt16(gyro[4], gyro[5]);

  // ±2g: 16384 LSB/g
  ax = (float)axRaw / 16384.0f;
  ay = (float)ayRaw / 16384.0f;
  az = (float)azRaw / 16384.0f;

  // ±2000 deg/s: 16.4 LSB/(deg/s)
  gx = (float)gxRaw / 16.4f;
  gy = (float)gyRaw / 16.4f;
  gz = (float)gzRaw / 16.4f;

  return true;
}

// -----------------------------------------------------------------------------
// BLE
// -----------------------------------------------------------------------------

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();
    if (value.length() > 0) {
      Serial.print("Received BLE: ");
      for (int i = 0; i < value.length(); i++) Serial.print(value[i]);
      Serial.println();
    }
  }
};

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("BLE device connected");
  }

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("BLE device disconnected");
  }
};

void BLESetup() {
  BLEDevice::init(BLE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  pAdvertising->start();
}

void BLEWrite(String msg) {
  pCharacteristic->setValue(msg.c_str());
  pCharacteristic->notify();
  Serial.println(msg);
}

// -----------------------------------------------------------------------------
// OLED
// -----------------------------------------------------------------------------

void drawMovingTrapezoid(float roll, float pitch) {
  const float cx = 35.0f, cy = 20.0f;
  float baseW = 15.0f, h_half = 8.0f;

  float w_top = constrain(baseW + (pitch * 0.3f), 2.0f, 25.0f);
  float rx[4] = { -w_top, w_top, baseW, -baseW };
  float ry[4] = { -h_half, -h_half, h_half, h_half };

  float rad = roll * M_PI / 180.0f;
  float s = sin(rad);
  float c = cos(rad);

  int px[4], py[4];

  for (int i = 0; i < 4; i++) {
    px[i] = (int)(cx + (rx[i] * c - ry[i] * s));
    py[i] = (int)(cy + (rx[i] * s + ry[i] * c));
  }

  u8g2.drawLine(px[0], py[0], px[1], py[1]);
  u8g2.drawLine(px[1], py[1], px[2], py[2]);
  u8g2.drawLine(px[2], py[2], px[3], py[3]);
  u8g2.drawLine(px[3], py[3], px[0], py[0]);
}

// -----------------------------------------------------------------------------
// setup / loop
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);

  pinMode(SWITCH_PIN, INPUT_PULLUP);

  // BMI160 I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  u8g2.clearBuffer();
  u8g2.drawStr(0, 10, "BMI160 Init...");
  u8g2.sendBuffer();

  bool sensorOK = bmi160Init();

  BLESetup();

  if (sensorOK) {
    Serial.println("BMI160 ESP32-C3 Setup Complete. Measuring...");
  } else {
    Serial.println("BMI160 initialization FAILED.");
  }
}

void loop() {
  u8g2.clearBuffer();

  char buf[180];

  int16_t axRaw, ayRaw, azRaw;
  int16_t gxRaw, gyRaw, gzRaw;
  float ax, ay, az;
  float gx, gy, gz;

  bool sensorOK = bmi160Read(
    ax, ay, az, gx, gy, gz,
    axRaw, ayRaw, azRaw,
    gxRaw, gyRaw, gzRaw
  );

  float roll = 0.0f;
  float pitch = 0.0f;

  if (sensorOK) {
    // Simple low-pass filter for accelerometer
    fX = (ax * filterAlpha) + (fX * (1.0f - filterAlpha));
    fY = (ay * filterAlpha) + (fY * (1.0f - filterAlpha));
    fZ = (az * filterAlpha) + (fZ * (1.0f - filterAlpha));

    // Simple low-pass filter for gyro
    fGX = (gx * filterAlpha) + (fGX * (1.0f - filterAlpha));
    fGY = (gy * filterAlpha) + (fGY * (1.0f - filterAlpha));
    fGZ = (gz * filterAlpha) + (fGZ * (1.0f - filterAlpha));

    // Roll/Pitch from gravity vector.
    // This preserves the same basic angle convention as the old sketch.
    roll = atan2(fY, fZ) * 180.0f / PI;
    pitch = atan2(-fX, sqrt(fY * fY + fZ * fZ)) * 180.0f / PI;

    // Keep old X/Y/Z/Roll/Pitch fields and append gyro fields.
    snprintf(
      buf, sizeof(buf),
      "X:%d,Y:%d,Z:%d,Roll:%6.2f,Pitch:%6.2f,GX:%6.1f,GY:%6.1f,GZ:%6.1f",
      axRaw, ayRaw, azRaw,
      roll, pitch,
      fGX, fGY, fGZ
    );
    Serial.println(buf);
    if (deviceConnected) {
      BLEWrite(String(buf));
      u8g2.drawStr(0, 10, "BLE Connect");
    } else {
      u8g2.drawStr(0, 10, "BLE Ready");
    }

    drawMovingTrapezoid(roll, pitch);

  } else {
    u8g2.drawStr(0, 10, "BMI160 ERROR");
  }

  // SWITCH_PIN is retained so existing hardware wiring does not need to change.
  // The old min/max ADC calibration is intentionally removed because BMI160
  // produces signed digital acceleration values and does not use ADC endpoints.

  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();

    u8g2.clearBuffer();
    u8g2.drawStr(0, 10, "BLE disConn");
    u8g2.sendBuffer();

    Serial.println("Start advertising...");
  }

  oldDeviceConnected = deviceConnected;

  u8g2.sendBuffer();

  // 200 ms = 5 BLE notifications/sec, same as the original sketch.
  delay(200);
}

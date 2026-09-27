/*
  ESP32-C3とBMI160を使用する6軸センサーテレメトリー。
  加速度からRoll/Pitchを計算し、加速度・角度・角速度をBLEで送信します。

  BMI160の配線:
    I2C SDA = GPIO5、SCL = GPIO6
    SA0/SDOをVDDIOへ接続するとI2Cアドレスは0x69になります。
    CSBをVDDIOへ接続してI2Cモードにします。

  センサーの設定と換算:
    加速度 X/Y/Z: 符号付き16 bit、±2 g、16384 LSB/g
    角速度 X/Y/Z: 符号付き16 bit、±2000 deg/s、16.4 LSB/(deg/s)

  Roll/Pitchは重力方向を示す加速度から計算します。
  BLEには生の加速度、計算した角度、平滑化した角速度を送信します。
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
#define SDA_PIN 5
#define SCL_PIN 6

// --- BLE設定 ---
// Web画面と接続するためのデバイス名、サービスUUID、特性UUIDです。
#define BLE_NAME "RC_CAR_TELEMETRY"
#define SERVICE_UUID "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcdefab-1234-5678-1234-abcdefabcdef"

// --- BMI160 I2C ---
#define BMI160_ADDR 0x69   // 回路図では SA0/SDO が VDD にプルアップ
#define BMI160_CHIP_ID 0xD1

// BMI160で使用するレジスターアドレスです。
#define REG_CHIP_ID   0x00
#define REG_GYR_X_L   0x0C
#define REG_ACC_X_L   0x12
#define REG_ACC_CONF  0x40
#define REG_ACC_RANGE 0x41
#define REG_GYR_CONF  0x42
#define REG_GYR_RANGE 0x43
#define REG_CMD       0x7E

// BMI160の加速度計とジャイロを通常動作モードにするコマンドです。
#define CMD_ACC_NORMAL 0x11
#define CMD_GYR_NORMAL 0x15

// BLEサーバー、通知用Characteristic、および接続状態を保持します。
BLECharacteristic *pCharacteristic;
BLEServer *pServer = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// センサー値のローパスフィルター出力。初期状態ではZ方向に重力があると仮定します。
float fX = 0.0f, fY = 0.0f, fZ = 1.0f;
float fGX = 0.0f, fGY = 0.0f, fGZ = 0.0f;
// 新しい測定値をどの程度フィルター出力へ反映するかを決めます。
const float filterAlpha = 0.5f;

// 72x40 OLED用のU8g2ドライバー。I2CのSCL/SDAピンを明示しています。
U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2(
  U8G2_R0, /* reset=*/U8X8_PIN_NONE, SCL_PIN, SDA_PIN
);

// -----------------------------------------------------------------------------
// BMI160 low-level I2C
// -----------------------------------------------------------------------------

// BMI160の指定レジスターへ1バイトを書き込みます。
bool bmi160WriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

// 指定レジスターからlenバイトを読み込みます。
// レジスター指定後にリピートスタートを発行し、読み取った値をdataへ格納します。
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

// BMI160のレジスターを1バイト読み込みます。
uint8_t bmi160ReadReg(uint8_t reg) {
  uint8_t value = 0;
  bmi160ReadRegs(reg, &value, 1);
  return value;
}

// リトルエンディアンの2バイトを符号付き16 bit値へ組み立てます。
int16_t makeInt16(uint8_t lo, uint8_t hi) {
  return (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
}

// チップIDを確認し、加速度計・ジャイロを通常モードと測定範囲に設定します。
// いずれかの通信または設定に失敗した場合はfalseを返します。
bool bmi160Init() {
  delay(50);

  uint8_t chipId = bmi160ReadReg(REG_CHIP_ID);
  Serial.printf("BMI160 CHIP_ID: 0x%02X\n", chipId);

  if (chipId != BMI160_CHIP_ID) {
    Serial.println("ERROR: BMI160 not detected.");
    Serial.println("Check SDA/SCL, VDD/VDDIO, CSB and SA0.");
    return false;
  }

  // 加速度計を通常動作モードにします。
  if (!bmi160WriteReg(REG_CMD, CMD_ACC_NORMAL)) return false;
  delay(5);

  // ジャイロを通常動作モードにします。起動完了まで待ってから設定を続けます。
  if (!bmi160WriteReg(REG_CMD, CMD_GYR_NORMAL)) return false;
  delay(80);

  // 加速度計を100 Hz、通常帯域幅に設定します。
  if (!bmi160WriteReg(REG_ACC_CONF, 0x28)) return false;

  // 加速度の測定範囲を±2 gに設定します。
  if (!bmi160WriteReg(REG_ACC_RANGE, 0x03)) return false;

  // ジャイロを100 Hz、通常帯域幅に設定します。
  if (!bmi160WriteReg(REG_GYR_CONF, 0x28)) return false;

  // ジャイロの測定範囲を±2000 deg/sに設定します。
  if (!bmi160WriteReg(REG_GYR_RANGE, 0x00)) return false;

  delay(20);
  Serial.println("BMI160 initialization complete.");
  return true;
}

// 加速度と角速度を読み出し、生データと物理単位へ換算した値を返します。
// 通信に失敗した場合はfalseを返します。
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

  // 加速度の生データをg単位へ換算します（±2 g設定では16384 LSB/g）。
  ax = (float)axRaw / 16384.0f;
  ay = (float)ayRaw / 16384.0f;
  az = (float)azRaw / 16384.0f;

  // 角速度の生データをdeg/sへ換算します（±2000 deg/s設定では16.4 LSB/(deg/s)）。
  gx = (float)gxRaw / 16.4f;
  gy = (float)gyRaw / 16.4f;
  gz = (float)gzRaw / 16.4f;

  return true;
}

// -----------------------------------------------------------------------------
// BLE
// -----------------------------------------------------------------------------

// クライアントからCharacteristicへ書き込みがあったとき、受信内容をシリアルへ表示します。
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

// BLE接続・切断イベントに応じて接続状態を更新します。
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

// BLEデバイス名、サービス、Characteristic、通知設定を作成してアドバタイズを開始します。
void BLESetup() {
  BLEDevice::init(BLE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // センサーデータの読取り・書込み・通知を許可します。
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  // 接続先がサービスを発見できるよう、サービスUUIDを含めて広告します。
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  pAdvertising->start();
}

// Characteristicの値を更新し、接続中のクライアントへ通知します。
void BLEWrite(String msg) {
  pCharacteristic->setValue(msg.c_str());
  pCharacteristic->notify();
  Serial.println(msg);
}

// -----------------------------------------------------------------------------
// OLED
// -----------------------------------------------------------------------------

// Rollで台形を回転し、Pitchに応じて上辺の幅を変えてOLEDに姿勢を表示します。
void drawMovingTrapezoid(float roll, float pitch) {
  const float cx = 35.0f, cy = 20.0f;
  float baseW = 15.0f, h_half = 8.0f;

  float w_top = constrain(baseW + (pitch * 0.3f), 2.0f, 25.0f);
  float rx[4] = { -w_top, w_top, baseW, -baseW };
  float ry[4] = { -h_half, -h_half, h_half, h_half };

  // Rollをラジアンに変換し、各頂点を中心(cx, cy)の周りで回転します。
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

// 起動時にシリアル、OLED、I2C、BMI160、BLEの順に初期化します。
void setup() {
  Serial.begin(115200);

  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);

  // BMI160とOLEDで共有するI2Cバスを開始し、通信速度を400 kHzに設定します。
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  u8g2.clearBuffer();
  u8g2.drawStr(0, 10, "BMI160 Init...");
  u8g2.sendBuffer();

  // センサーの初期化結果にかかわらずBLEを開始し、状態をシリアルへ表示します。
  bool sensorOK = bmi160Init();

  BLESetup();

  if (sensorOK) {
    Serial.println("BMI160 ESP32-C3 Setup Complete. Measuring...");
  } else {
    Serial.println("BMI160 initialization FAILED.");
  }
}

// センサーを読み取り、表示・BLE通知を更新します。
void loop() {
  u8g2.clearBuffer();

  char buf[180];

  int16_t axRaw, ayRaw, azRaw;
  int16_t gxRaw, gyRaw, gzRaw;
  float ax, ay, az;
  float gx, gy, gz;

  // 生データと単位換算済みデータを同時に受け取ります。
  bool sensorOK = bmi160Read(
    ax, ay, az, gx, gy, gz,
    axRaw, ayRaw, azRaw,
    gxRaw, gyRaw, gzRaw
  );

  float roll = 0.0f;
  float pitch = 0.0f;

  if (sensorOK) {
    // 加速度の急な変化を抑え、姿勢計算に使う値を滑らかにします。
    fX = (ax * filterAlpha) + (fX * (1.0f - filterAlpha));
    fY = (ay * filterAlpha) + (fY * (1.0f - filterAlpha));
    fZ = (az * filterAlpha) + (fZ * (1.0f - filterAlpha));

    // 角速度にも同じローパスフィルターを適用します。
    fGX = (gx * filterAlpha) + (fGX * (1.0f - filterAlpha));
    fGY = (gy * filterAlpha) + (fGY * (1.0f - filterAlpha));
    fGZ = (gz * filterAlpha) + (fGZ * (1.0f - filterAlpha));

    // 重力ベクトルからRoll/Pitchを度単位で計算します。
    roll = atan2(fY, fZ) * 180.0f / PI;
    pitch = atan2(-fX, sqrt(fY * fY + fZ * fZ)) * 180.0f / PI;

    // BLE送信文字列。X/Y/Zは生加速度、Roll/Pitchは度、GX/GY/GZはdeg/sです。
    snprintf(
      buf, sizeof(buf),
      "X:%d,Y:%d,Z:%d,Roll:%6.2f,Pitch:%6.2f,GX:%6.1f,GY:%6.1f,GZ:%6.1f",
      axRaw, ayRaw, azRaw,
      roll, pitch,
      fGX, fGY, fGZ
    );
    Serial.println(buf);
    if (deviceConnected) {
      // 接続中は測定値を通知し、OLEDに接続状態を表示します。
      BLEWrite(String(buf));
      u8g2.drawStr(0, 10, "BLE Connect");
    } else {
      // 未接続時も測定とOLED表示は続け、BLEの接続待ち状態を示します。
      u8g2.drawStr(0, 10, "BLE Ready");
    }

    drawMovingTrapezoid(roll, pitch);

  } else {
    // センサー読取りに失敗した周期はエラー表示にします。
    u8g2.drawStr(0, 10, "BMI160 ERROR");
  }

  if (!deviceConnected && oldDeviceConnected) {
    // 切断を検出したら少し待ってから再度アドバタイズを開始します。
    delay(500);
    pServer->startAdvertising();

    u8g2.clearBuffer();
    u8g2.drawStr(0, 10, "BLE disConn");
    u8g2.sendBuffer();

    Serial.println("Start advertising...");
  }

  oldDeviceConnected = deviceConnected;

  u8g2.sendBuffer();

  delay(10);
}

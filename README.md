# ESP32-C3 6軸センサーテレメトリー

ESP32-C3とBMI160で取得した加速度・角速度・姿勢角をBLEでブラウザへ送り、RCカーの姿勢を3D表示するプロジェクトです。ブラウザ画面では受信データの記録と再生もできます。

## 構成

- `ESP32C3_6AXIS.ino`: BMI160の初期化と読み取り、BLE通知、OLED表示を行うArduinoスケッチ。
- `docs/index.html`: BLE接続、記録・再生ボタン、描画画面を含むWebページ。
- `docs/sketch.js`: Web Bluetooth通信、受信データ処理、p5.jsによる3Dモデルとグラフの描画。
- `docs/buggy.obj`: 姿勢表示に使うRCバギーの3Dモデル。

## ハードウェア

| 信号 | ESP32-C3 GPIO / 設定 |
| --- | --- |
| BMI160 SDA | GPIO5 |
| BMI160 SCL | GPIO6 |
| スイッチ | GPIO3（`INPUT_PULLUP`。現在のスケッチでは読み取り未使用） |
| BMI160 I2Cアドレス | `0x69`（SA0/SDOをVDDIOへ接続） |

BMI160はI2Cモードで使用します。CSBをVDDIOへ接続し、SA0/SDOもVDDIOへ接続してアドレスを`0x69`にします。センサーとOLEDはGPIO5/6のI2Cバスを共有する設定です。電源電圧や各モジュールのロジックレベルは、使用するボード／ブレークアウトの仕様に合わせてください。

BMI160は加速度±2 g、ジャイロ±2000 deg/s、出力データレート100 Hzで設定されています。Roll/Pitchはローパス処理した加速度から算出します。角速度にもローパス処理を適用します。

## ファームウェアの書き込み

1. Arduino IDEにESP32ボードサポートを導入し、使用するESP32-C3ボードとシリアルポートを選択します。
2. `ESP32C3_6AXIS.ino`を開き、U8g2ライブラリをインストールします。`Wire`、`math`、BLEライブラリはArduino／ESP32環境から提供されます。
3. コンパイルしてボードへ書き込みます。
4. シリアルモニターを115200 baudで開き、`BMI160 initialization complete.` が出ることを確認します。

センサーが見つからない場合は、SDA/SCL、電源、CSB、SA0/SDO、およびI2Cアドレスを確認してください。

## Web画面の利用

Web Bluetoothに対応したブラウザ（ChromeまたはEdgeなど）で`docs/index.html`を開きます。Web Bluetoothはセキュアコンテキストを必要とするため、公開時はHTTPSでホストしてください。ローカルで試す場合は、ブラウザがセキュアコンテキストとして扱うlocalhost上のWebサーバーから配信します。`file://`で直接開く方法ではBLE接続できない場合があります。

1. ESP32-C3の電源を入れ、BMI160の初期化が成功したことを確認します。
2. Web画面で`CONNECT`を押し、デバイス一覧から`RC_CAR_TELEMETRY`を選択して接続を許可します。
3. 3D表示と受信データを確認します。`DISCONNECT`で切断できます。予期しない切断時は、ページが再接続を試みます。
4. `REC`で記録を開始し、`STOP`で終了します。記録データはブラウザ内に保持され、停止時にクリップボードへのコピーを試みます。
5. 記録済みデータがある状態で`PLAY`を押すと、記録データを繰り返し再生します。再生中に`STOP`で終了します。

記録はページを閉じると失われます。ファイルへの自動保存機能はありません。

## BLEデータ形式

デバイス名は`RC_CAR_TELEMETRY`です。Service UUIDとCharacteristic UUIDは次のとおりです。

- Service: `12345678-1234-1234-1234-1234567890ab`
- Characteristic: `abcdefab-1234-5678-1234-abcdefabcdef`

CharacteristicはRead、Write、Notifyに対応します。スケッチはおよそ200 ms間隔（約5通知/秒）で、次のCSV風テキストを通知します。

```text
X:<raw>,Y:<raw>,Z:<raw>,Roll:<degrees>,Pitch:<degrees>,GX:<dps>,GY:<dps>,GZ:<dps>
```

`X/Y/Z`はBMI160の符号付き加速度生データ（±2 g設定、16384 LSB/g）、`Roll/Pitch`は度、`GX/GY/GZ`は平滑化後の角速度（deg/s）です。Web画面の受信データ欄にはこの通知文字列が表示されます。

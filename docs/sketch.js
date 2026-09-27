// --- BLE設定 ---
// BLE通信で使用するServiceとCharacteristicのUUID
const SERVICE_UUID = "12345678-1234-1234-1234-1234567890ab";
const CHARACTERISTIC_UUID = "abcdefab-1234-5678-1234-abcdefabcdef";

// BLEデバイスと特徴量(Characteristic)のインスタンスを保持する変数
let device = null;
let characteristic = null;

// センサーから取得した加速度と姿勢(ロール・ピッチ)の値を保持する変数
let accelX = 0;
let accelY = 0;
let accelZ = 1.0;
let rollVal = 0;
let pitchVal = 0;
let GXVal = 0.0;
let GYVal = 0.0;
let GZVal = 0.0;

// 描画用のロール・ピッチ角度と、初回のデータ受信かどうかを判定するフラグ
let displayRoll = 0;
let displayPitch = 0;
let isFirstData = true;

// ユーザーの意図的な切断かどうかを判定するフラグ
let isIntentionalDisconnect = false;
// 現在受信している生データの文字列を保持する変数
let currentValString = "-";

// --- REC/PLAY設定 ---
// 録画中・再生中を判定するフラグ
let isRecording = false;
let isPlaying = false;
// 録画されたデータを保持する配列 (形式: [{time: ミリ秒, raw: "生データ文字列"}])
let recordedData = [];
// 再生時の現在のデータインデックス
let playIndex = 0;
// 録画および再生の開始時間を記録する変数
let recordingStartTime = 0;
let playStartTime = 0;

// Z軸の加速度の履歴を保持する配列 (グラフ描画用)
let historyZ = [];
// 履歴として保持する最大データ数
const MAX_HISTORY = 60;

// 3Dモデルデータを保持する変数
let carModel;

/**
 * リソースの事前読み込みを行います。
 * p5.jsの標準関数で、setup()の前に自動的に呼び出されます。
 */
function preload() {
  // 変換したobjファイル（車の3Dモデル）を読み込む
  carModel = loadModel("buggy.obj", true);
}

/**
 * BLEデバイスに接続を試みます。
 * 非同期処理(async/await)を用いて、ユーザーにデバイス選択を促し、接続を確立します。
 */
async function connectBLE() {
  const status = document.getElementById("status");
  isIntentionalDisconnect = false;
  try {
    status.innerText = "SELECTING...";
    // デバイスの検索と選択ダイアログの表示
    device = await navigator.bluetooth.requestDevice({
      filters: [{ services: [SERVICE_UUID] }],
    });

    // 切断イベントのハンドラを登録（予期せぬ切断時の再接続用）
    device.addEventListener("gattserverdisconnected", onDisconnected);

    status.innerText = "CONNECTING...";

    // 重要：GATTサーバーへの接続が完了するのを待つ
    const server = await device.gatt.connect();
    // 接続安定化のため、500msほど待機する
    await new Promise((resolve) => setTimeout(resolve, 500));

    // 接続状態を二重チェック（GATT Serverが存在し、接続されているか）
    if (!server || !server.connected) {
      throw new Error("GATT Server connection failed.");
    }

    status.innerText = "GETTING SERVICE...";
    // 対象のサービスを取得
    const service = await server.getPrimaryService(SERVICE_UUID);

    status.innerText = "GETTING CHARACTERISTIC...";
    // 対象のキャラクタリスティックを取得
    characteristic = await service.getCharacteristic(CHARACTERISTIC_UUID);

    // デバイスからのデータ通知(Notify)を開始
    await characteristic.startNotifications();
    // データ受信時のイベントハンドラを登録
    characteristic.addEventListener("characteristicvaluechanged", handleNotify);

    status.innerText = "CONNECTED";
  } catch (e) {
    status.innerText = "ERROR: " + e.message;
    console.error(e);
  }
}

/**
 * デバイスが切断された際に呼び出されるイベントハンドラ。
 * 意図せぬ切断の場合は自動再接続を試みます。
 *
 * @param {Event} event - 切断イベントオブジェクト
 */
function onDisconnected(event) {
  const status = document.getElementById("status");
  characteristic = null;
  console.log("onDisconnected", event);

  if (isIntentionalDisconnect) {
    // 意図的な切断の場合
    status.innerText = "DISCONNECTED";
  } else {
    // 予期せぬ切断の場合は再接続処理へ
    status.innerText = "LOST CONNECTION. RECONNECTING...";
    setTimeout(reconnectBLE, 2000); // 2秒後に再接続スタート
  }
}

/**
 * BLEデバイスへの自動再接続を実行します。
 */
async function reconnectBLE() {
  // デバイス情報がない場合、または意図的な切断の場合は処理しない
  if (!device || isIntentionalDisconnect) return;

  const status = document.getElementById("status");
  try {
    status.innerText = "RECONNECTING...";
    // 再度GATTサーバーへ接続
    const server = await device.gatt.connect();
    await new Promise((resolve) => setTimeout(resolve, 500));

    // サービスとキャラクタリスティックを再取得
    const service = await server.getPrimaryService(SERVICE_UUID);
    characteristic = await service.getCharacteristic(CHARACTERISTIC_UUID);

    // 通知の再開とイベントリスナの再登録
    await characteristic.startNotifications();
    characteristic.addEventListener("characteristicvaluechanged", handleNotify);

    status.innerText = "CONNECTED (RECOVERED)";
    console.log("Auto-reconnect successful.");
  } catch (e) {
    console.error("Auto-reconnect failed.", e);
    status.innerText = "RETRYING CONNECTION...";
    setTimeout(reconnectBLE, 3000); // 失敗したら3秒後に再試行
  }
}

/**
 * BLEデバイスから意図的に切断します。
 */
function disconnectBLE() {
  isIntentionalDisconnect = true;
  if (device) device.gatt.disconnect();
}

/**
 * 録画(REC)ボタンのトグル処理を行います。
 * 録画開始時は変数を初期化し、停止時はデータをクリップボードにコピーします。
 */
function toggleRec() {
  const btnRec = document.getElementById("btn-rec");
  const hiddenData = document.getElementById("hidden-data");

  if (!isRecording) {
    // 録画開始処理
    isRecording = true;
    if (isPlaying) togglePlay(); // 再生中であれば停止させる

    btnRec.innerText = "STOP";
    btnRec.style.background = "#6c757d"; // ボタン色をグレーに変更

    // 録画用バッファとデータ配列を初期化
    hiddenData.value = "";
    recordedData = [];
    recordingStartTime = performance.now(); // 録画開始時のタイムスタンプを記録
  } else {
    // 録画停止処理
    isRecording = false;
    btnRec.innerText = "REC";
    btnRec.style.background = "#dc3545"; // ボタン色を元の赤色に戻す

    // クリップボードに録画データをコピー
    // p5.jsエディタ(iframe)などの環境でクリップボードAPIがブロックされる場合のフォールバック処理を考慮
    if (navigator.clipboard && window.isSecureContext) {
      navigator.clipboard.writeText(hiddenData.value).then(() => {
        console.log("Recorded data copied to clipboard.");
      }).catch(err => {
        console.warn("Clipboard API failed, trying fallback.", err);
        fallbackCopyTextToClipboard(hiddenData.value);
      });
    } else {
      fallbackCopyTextToClipboard(hiddenData.value);
    }
  }
}

/**
 * クリップボードコピーのフォールバック処理。
 * navigator.clipboard が使えない環境（非HTTPSやiframe内等）向けに、
 * 一時的なテキストエリアを作成してコピーコマンドを実行します。
 *
 * @param {string} text - コピーする文字列
 */
function fallbackCopyTextToClipboard(text) {
  const textArea = document.createElement("textarea");
  textArea.value = text;

  // 画面スクロールを防ぐため、画面外ではなく左上に固定配置
  textArea.style.top = "0";
  textArea.style.left = "0";
  textArea.style.position = "fixed";

  document.body.appendChild(textArea);
  textArea.focus();
  textArea.select();

  try {
    const successful = document.execCommand("copy");
    if (successful) {
      console.log("Fallback: Copying text command was successful.");
    } else {
      console.error("Fallback: Copying text command was unsuccessful.");
    }
  } catch (err) {
    console.error("Fallback: Oops, unable to copy", err);
  }

  document.body.removeChild(textArea);
}

/**
 * 再生(PLAY)ボタンのトグル処理を行います。
 * 録画されたデータがある場合、そのデータを元にアニメーションの再生を開始または停止します。
 */
function togglePlay() {
  const btnPlay = document.getElementById("btn-play");

  if (!isPlaying) {
    // 再生開始処理
    if (recordedData.length === 0) {
      alert("記録データがありません");
      return;
    }
    isPlaying = true;
    if (isRecording) toggleRec(); // もし録画中なら録画を停止する

    btnPlay.innerText = "STOP";
    btnPlay.style.background = "#6c757d"; // ボタン色をグレーに変更

    playStartTime = performance.now(); // 再生開始時のタイムスタンプを記録
    playIndex = 0; // 再生インデックスをリセット
  } else {
    // 再生停止処理
    isPlaying = false;
    btnPlay.innerText = "PLAY";
    btnPlay.style.background = "#28a745"; // ボタン色を元の緑色に戻す
  }
}

/**
 * 受信した文字列データを解析し、各変数に適用します。
 *
 * @param {string} val - センサーから送られてきた文字列データ (例: "X:100 Y:200 Z:1000 Roll:10 Pitch:-5")
 */
function parseDataAndApply(val) {
  currentValString = val;

  // 正規表現を用いて各パラメータの数値を抽出
  const mX = val.match(/X:\s*(-?[\d.]+)/);
  const mY = val.match(/Y:\s*(-?[\d.]+)/);
  const mZ = val.match(/Z:\s*(-?[\d.]+)/);
  const mR = val.match(/Roll:\s*(-?[\d.]+)/);
  const mP = val.match(/Pitch:\s*(-?[\d.]+)/);
  const mGX = val.match(/Pitch:\s*(-?[\d.]+)/);
  const mGY = val.match(/Pitch:\s*(-?[\d.]+)/);
  const mGZ = val.match(/Pitch:\s*(-?[\d.]+)/);

  // 抽出できた場合は数値に変換して変数に格納 (加速度は1000で割ってG単位にする想定)
  if (mX) accelX = parseFloat(mX[1]) / 1000;
  if (mY) accelY = parseFloat(mY[1]) / 1000;
  if (mZ) accelZ = parseFloat(mZ[1]) / 1000;
  if (mR) rollVal = parseFloat(mR[1]);
  if (mP) pitchVal = parseFloat(mP[1]);
  if (mGX) GXVal = parseFloat(mGX[1]);
  if (mGY) GYVal = parseFloat(mGY[1]);
  if (mGZ) GZVal = parseFloat(mGZ[1]);

  // グラフレンダリング用にZ軸加速度の履歴を更新
  historyZ.push(GZVal * 100);
  if (historyZ.length > MAX_HISTORY) {
    historyZ.shift(); // 古いデータを削除して最大数を維持
  }
}

/**
 * BLECharacteristicのnotifyイベントで呼び出され、送られてきたデータを受信します。
 *
 * @param {Event} event - notifyイベントオブジェクト
 */
function handleNotify(event) {
  // 受信したバイト列を文字列にデコード
  const val = new TextDecoder().decode(event.target.value);

  // 録画中の場合はデータを保存
  if (isRecording) {
    const elapsed = performance.now() - recordingStartTime;
    const timeStr = `[${elapsed.toFixed(2)}]`;
    const recordStr = `${timeStr} ${val}\n`;

    // 隠しテキストエリアに蓄積（後でクリップボードへコピーするため）
    document.getElementById("hidden-data").value += recordStr;
    // 再生用の配列にも保存
    recordedData.push({ time: elapsed, raw: val });
  }

  // 再生中でなければリアルタイムのデータを反映
  if (!isPlaying) {
    parseDataAndApply(val);
  }
}

// --- 描画レイアウト (3Dのみを中央に配置) ---

/**
 * p5.jsの初期化処理。キャンバスを生成し、WebGLモードに設定します。
 */
function setup() {
  let container = document.getElementById("canvas-container");
  // ウィンドウのサイズに合わせてキャンバスを作成 (高さを少しマイナスしているのはUIの領域確保のため)
  createCanvas(windowWidth, windowHeight - 120, WEBGL).parent(container);
}

/**
 * p5.jsのメイン描画ループ。毎フレーム呼び出されます。
 */
function draw() {
  background(64, 128, 200); // 背景色を青みがかった色で塗りつぶし

  // --- 再生処理 ---
  if (isPlaying && recordedData.length > 0) {
    let currentPlayTime = performance.now() - playStartTime;
    // 経過時間に該当するRecordedDataがある限り、連続して適用（時間が追いつくまで）
    while (playIndex < recordedData.length && recordedData[playIndex].time <= currentPlayTime) {
      parseDataAndApply(recordedData[playIndex].raw);
      playIndex++;
    }
    // 全て再生し終わったら最初からループ再生
    if (playIndex >= recordedData.length) {
      playStartTime = performance.now();
      playIndex = 0;
    }
  }

  let h = height / 2;

  // --- 1. 3D姿勢（上段に描画） ---
  push();
  // 3D空間への移動。上半分に配置
  translate(0, -h + h / 1, 0);
  draw3DPosture();
  pop();

  // --- 2. 振動グラフ（下段に描画） ---
  push();
  // 下半分に配置
  translate(0, h - h / 1, 0);
  // グラフの枠のサイズを定義
  let graphW = width - 60;
  let graphH = h * 0.6;
  drawGraph(graphW, graphH);
  pop();

  // HTML側への現在値のテキスト更新（UI表示用）
  document.getElementById("count").innerText = currentValString;
}

/**
 * ロール、ピッチから3Dモデルの姿勢を描画します。
 */
function draw3DPosture() {
  // 1. ライトの設定（これがないと陰影がつかず立体感が出ません）
  ambientLight(100); // 全体を照らす環境光（弱い光）
  directionalLight(255, 255, 255, 0.5, 1, -1); // 右斜め下方向へ向かう強い白色の平行光源

  push(); // 描画状態を保存

  // iPhoneなどの縦長画面に合わせてモデルのスケールを自動調整
  let scaleFactor = min(width, height) / 300;
  scale(scaleFactor);

  // 初回データ受信時に描画角度を実際の角度に同期させる
  if (isFirstData && (rollVal !== 0 || pitchVal !== 0)) {
    displayRoll = rollVal;
    displayPitch = pitchVal;
    isFirstData = false;
  }

  // --- 角度計算とスムージング処理 ---

  /**
   * 角度の差分を -180 ~ +180 の範囲に正規化するローカル関数
   */
  function normalizeAngleDiff(target, current) {
    let diff = target - current;
    while (diff < -180) diff += 360;
    while (diff > 180) diff -= 360;
    return diff;
  }

  // 目標角度に対する現在の表示角度の差分を計算
  let diffPitch = normalizeAngleDiff(pitchVal, displayPitch);
  let diffRoll = normalizeAngleDiff(rollVal, displayRoll);

  // ジャンプ時の衝撃などで発生する極端なスパイク（ノイズ）を無視・制限する
  // 1フレームでの最大変化角度を規制
  const MAX_DELTA = 15;
  diffPitch = constrain(diffPitch, -MAX_DELTA, MAX_DELTA);
  diffRoll = constrain(diffRoll, -MAX_DELTA, MAX_DELTA);

  // ローパスフィルタをかけて滑らかに追従させる（係数 0.0〜1.0）
  // 値が小さいほど滑らかになるが追従は遅れる
  const LPF_ALPHA = 0.2;
  displayPitch += diffPitch * LPF_ALPHA;
  displayRoll += diffRoll * LPF_ALPHA;

  // 計算後の表示角度を -180 ~ +180 の範囲に保つ
  while (displayPitch <= -180) displayPitch += 360;
  while (displayPitch > 180) displayPitch -= 360;
  while (displayRoll <= -180) displayRoll += 360;
  while (displayRoll > 180) displayRoll -= 360;

  // 3D空間の回転を適用
  // 鳥瞰図風（少し見下ろした視点）にするため、X軸回転にオフセットを加算
  rotateX(radians(displayPitch) + Math.PI - Math.PI / 11);
  rotateZ(-radians(displayRoll));

  // 2. モデルの材質(マテリアル)の設定
  noStroke(); // ポリゴンの網目（ワイヤーフレームライン）を出さない
  fill(180); // モデルの基本色（180は明るいグレー）
  specularMaterial(255); // 光の反射（ツヤ）の色を白に設定
  shininess(50); // 輝き具合。数値を大きくするとハイライトが鋭く金属っぽくなります

  // 3. 3Dモデル(=carModel)の描画
  model(carModel);

  pop(); // 描画状態を元に戻す
}

/**
 * Z軸加速度の履歴をグラフとして描画します。
 *
 * @param {number} gw - グラフ領域の横幅
 * @param {number} gh - グラフ領域の高さ
 */
function drawGraph(gw, gh) {
  // グラフの外枠（現在は描画なしのコメントアウトに近い状態だが、設定は残す）
  noFill();
  stroke(60);

  // 1.0Gの中心線（うっすらとした背景線）
  //strokeWeight(2); // 線の太さ
  //stroke(100, 0, 2150, -1000);
  strokeWeight(1);
  line(-gw / 2, 0, gw / 2, 0); // 必要に応じてコメント解除して使用

  // データの描画
  if (historyZ.length > 1) {
    stroke(50, 255, 100); // グラフの線の色（明るい緑色）
    strokeWeight(2); // 線の太さ
    noFill();

    beginShape(); // 連続した線を引く開始宣言
    for (let i = 0; i < historyZ.length; i++) {
      // X軸：履歴のインデックス(0 〜 MAX_HISTORY-1)をグラフ幅(-gw/2 〜 gw/2)にマッピング
      let x = map(i, 0, MAX_HISTORY - 1, -gw / 2, gw / 2);

      // Y軸：加速度Zに基づく縦位置の計算
      // 現在の実装では、Zが特定の範囲になるように独自のオフセット・スケール計算を行っています
      let y = (historyZ[i])  / 30;
      // 頂点を追加
      vertex(x, y);
    }
    endShape(); // 連続した線を引く終了宣言
  }
}

/**
 * ウィンドウがリサイズされた時に呼び出されるイベントハンドラ。
 * キャンバスのサイズを動的に再調整します。
 */
function windowResized() {
  let container = document.getElementById("canvas-container");
  // コンテナのサイズに合わせてキャンバスをリサイズ
  resizeCanvas(container.offsetWidth, container.offsetHeight);
}

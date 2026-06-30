#include <Wire.h>

#define LED_PIN A0

const uint8_t MT6701_ADDR = 0x06;

// モーターピン(TB6612FNG)
const int pinBIn1 = 8;
const int pinBIn2 = 7;
const int pinBPWM = 6;
const int pinSTBY = 12;

// アームエンコーダピン
const int encB_A = 2; // エンコーダBのA相
const int encB_B = 3; // エンコーダBのB相

volatile long encoderCount = 0;

float Kp_pendulum = 35.0; // 倒れそうな方向にアームを動かす力
float Kd_pendulum = 0.37;  // 振り子の振れを抑えるブレーキ

// アームの位置に対するゲイン(アームが回り続けないようにセンターに戻す力)
float Kp_arm = 0.272;
float Kd_arm = 0.0955;

// 振り上げ(エネルギー制御)用のゲイン
// アームをしゃくる強さ
float K_energy = 0.0000010;

// 制御状態(ステート)の定義
// 0: 振り上げモード(スイングアップ)
// 1: 倒立維持モード(バランス)
int controlMode = 0;

// 制御周期用
unsigned long lastTime = 0;
float lastPendulumAngle = 0;
float pendulumVelocity = 0;
float lastPendulumError = 0;
float lastArmError = 0;

float targetAngle = 1.0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(pinBIn1, OUTPUT);
  pinMode(pinBIn2, OUTPUT);
  pinMode(pinBPWM, OUTPUT);
  pinMode(pinSTBY, OUTPUT);

  pinMode(encB_A, INPUT_PULLUP);
  pinMode(encB_B, INPUT_PULLUP);

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // I2Cの通信速度を高速モード(400kHz)に設定（古田ペンデュラムの必須設定)

  digitalWrite(LED_PIN, HIGH);
  digitalWrite(pinSTBY, HIGH);

  attachInterrupt(digitalPinToInterrupt(encB_A), doEncoderA, CHANGE);

  lastTime = micros();

  delay(3000);
}

void loop() {
  // 微分用の時間(dt)を計算（秒単位）
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;
  if (dt <= 0.0) dt = 0.001;  // ゼロ除算防止
  lastTime = currentTime;

  // 1. MT6701から振り子の角度を読み込む
  float pendulumAngle = getMT6701Angle();
  if (pendulumAngle == 999.0) return; // エラー対策

  // 振り子の角速度(度/秒)を計算
  float rawVelocity = (pendulumAngle - lastPendulumAngle) / dt;
  pendulumVelocity = (rawVelocity * 0.15) + (pendulumVelocity * 0.85);
  lastPendulumAngle = pendulumAngle;

  float motorOutput = 0.0;

  // ==========================================================================
  // 2. ステートマシン(モード切替ロジック)
  // ==========================================================================

  // 振り子が真上(0度)から±20度以内のエリアにふわっと入ってきたら
  if (abs(pendulumAngle) < 20.0) {
    if (controlMode == 0) {
      // 振り上げ中だった場合、ここから倒立維持モードにバトンタッチ
      controlMode = 1;
      // 過去のエラー履歴をリセットしてPIDの急激な変化を防ぐ
      lastPendulumError = targetAngle - pendulumAngle;
      lastArmError = 0.0 - encoderCount;
    }
  } else if (abs(pendulumAngle) > 40.0) {
    // 真上から40度以上脱落したら、諦めて振り上げモード(スイングアップ)に戻る
    controlMode = 0;
  }

  // ==========================================================================
  // 3. 各モードの制御計算
  // ==========================================================================
  if (controlMode == 1) {
    // --- モード1: 倒立維持(これまでのPD制御) ---
    digitalWrite(LED_PIN, LOW); // モード識別用(LED消灯 = 倒立維持)

    float pendulumError = targetAngle - pendulumAngle;  // 目標は0度（真上）
    float pendulumDerivative = (pendulumError - lastPendulumError) / dt;
    lastPendulumError = pendulumError;
    float outputPendulum = (pendulumError * Kp_pendulum) + (pendulumDerivative * Kd_pendulum);

    // 【アーム軸】のPD制御計算（アームがどこまでも流れていかないようにする）
    float armError = 0.0 - encoderCount;  // 目標はエンコーダカウント0(初期位置)
    float armDerivative = (armError - lastArmError) / dt;
    lastArmError = armError;
    float outputArm = (armError * Kp_arm) + (armDerivative * Kd_arm);

    motorOutput = outputPendulum + outputArm;

  } else {
    // --- モード0: 振り上げ(エネルギー制御) ---
    digitalWrite(LED_PIN, HIGH); // モード識別用(LED点灯 = 振り上げ中)

    // 振り子の現在の簡易エネルギー計算(位置＋運動)
    float theta_rad = pendulumAngle * DEG_TO_RAD;
    float E = 0.5 * pendulumVelocity * pendulumVelocity;
    float E0 = 1000.0; // 真上で静止するときの目標エネルギー定数

    // コトノピック法によるアームの加速度指令の計算
    // 振り子の動く向き(pendulumVelocity)とエネルギーの過不足(E - E0)から出力を決定
    motorOutput = K_energy * (E - E0) * pendulumVelocity;

    // 安全対策: アームが回りすぎないよう、橋に行きそうになったら逆側に少し戻すブレーキ
    if (encoderCount >  400 && motorOutput > 0) motorOutput = -50;
    if (encoderCount < -400 && motorOutput < 0) motorOutput =  50;
    
  }

  controlMotor(motorOutput);

  // 定期的にシリアルプロッタなどに状況を確認したい場合
  // Serial.print("Pendulum: "); Serial.print(pendulumAngle);
  // Serial.print(", Arm: "); Serial.println(encoderCount);

  delay(2); // 約500Hzの超高速クローズドループ制御
}

// MT6701から「真上が0度」となる角度を取得する関数
float getMT6701Angle() {
  Wire.beginTransmission(MT6701_ADDR);
  Wire.write(0x03);
  if (Wire.endTransmission(false) != 0) {
    return 999.0;
  }

  // 2. 2バイト分(14ビットのデータ）を要求
  Wire.requestFrom(MT6701_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) {
    uint8_t highByte = Wire.read();
    uint8_t lowByte = Wire.read();
    uint16_t rawAngle = (highByte << 6) | (lowByte >> 2);
    // 4. 度数法(0.0〜359.99度)に変換
    float degAngle = rawAngle * 360.0 / 16384.0;

    float offsetAngle = degAngle - 263.00;  // 264.10

    if (offsetAngle >  180.0) offsetAngle -= 360.0;
    if (offsetAngle < -180.0) offsetAngle += 360.0;

    // 「真上を0度」に変換する処理
    // 補正後、180度（真上）だったものが0度になり、0度（真下）だったものが-180度になる
    float finalAngle = offsetAngle - 180.0;
    if (finalAngle < -180.0) finalAngle += 360;

    return finalAngle;
  }

  return 999.0;
}

void doEncoderA() {
  int statA = digitalRead(encB_A);
  int statB = digitalRead(encB_B);

  if (statA == statB) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void controlMotor(float output) {
  int pwmValue = (int)abs(output);

  // PWM値の上限・下限ガード
  if (pwmValue > 255) pwmValue = 255;
  if (pwmValue < 0)   pwmValue = 0;

  if (output > 0) {
    // 正転
    digitalWrite(pinBIn1, HIGH);
    digitalWrite(pinBIn2, LOW);
    analogWrite(pinBPWM, pwmValue);
  }
  else if (output < 0) {
    // 逆転
    digitalWrite(pinBIn1, LOW);
    digitalWrite(pinBIn2, HIGH);
    analogWrite(pinBPWM, pwmValue);
  }
  else {
    // 出力0のときはブレーキ（またはLOW, LOWのストップでも可）
    digitalWrite(pinBIn1, HIGH);
    digitalWrite(pinBIn2, HIGH);
    analogWrite(pinBPWM, 0);
  }
}

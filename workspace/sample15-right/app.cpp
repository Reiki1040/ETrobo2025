#include <stdio.h>
#include <algorithm>
#include <cmath>

#include "Motor.h"
#include "Clock.h"
#include "ColorSensor.h"
#include "ForceSensor.h"
#include "UltrasonicSensor.h"

using namespace spikeapi;

//-----------------------------------------------------------------------------
// パラメータ定義
//-----------------------------------------------------------------------------
// 基本速度
const int BASE_POWER = 50; //基本のモータ出力
const int SLOW_POWER = 40; //ダブルループに入った時用の出力

//　黒ライントレース時のPIDパラメータ
const float KP_BLACK = 1.60f; //比例ゲイン
const float KI_BLACK = 0.2f;  //積分ゲイン
const float KD_BLACK = 0.0f;  //微分ゲイン
const int TARGET_BLACK = 17;  //目標反射値、この値は白の反射値と黒の反射値の中間が理想

//　青ライントレース時のPIDパラメータ
const float KP_BLUE = 1.55f;
const float KI_BLUE = 0.08f;
const float KD_BLUE = 0.0f;
const int TARGET_BLUE = 21;  //青ライントレース時の反射値目標値
const float INTEGRAL_MAX = 100.0f; //積分値MAX

// 障害物回避用パラメータ
const int OBSTACLE_THRESHOLD_CM = 200; // この距離(mm)より手前に障害物があれば回避
const int AVOIDANCE_TURN_POWER = 48;  // 回避時の旋回速度

// 色判別HSV閾値
const int BLUE_HUE_MIN = 200; //青だと認識する最低H値
const int BLUE_HUE_MAX = 235; //青だと認識する最高H値

const int BLUE_SATURATION_MIN = 70; //青だと認識する最低S値

const int BLUE_VALUE_MIN = 24; //青だと認識する最低V値
const int BLUE_VALUE_MAX = 40; //青だと認識する最高V値

const int BLACK_VALUE_MAX = 20; //黒だと認識する最高V値

// 制御設定
const uint32_t CONTROL_INTERVAL = 10 * 1000;
const int SWITCH_CONFIRM_COUNT = 1;
const int SHARP_TURN_THRESHOLD = 60;
const uint32_t STRAIGHT_DURATION_MS = 500;
const int BLUE_IGNORE_CYCLES = 200;

enum class RobotState {
  TRACE_BLACK,
  TRACE_BLUE
};

extern "C" void main_task(intptr_t exinf) {
  // デバイスの初期化
  Motor leftWheel(EPort::PORT_B, Motor::EDirection::COUNTERCLOCKWISE, true);
  Motor rightWheel(EPort::PORT_A, Motor::EDirection::CLOCKWISE, true);
  ColorSensor colorSensor(EPort::PORT_C);
  Clock clock;
  ForceSensor forceSensor(EPort::PORT_D);
  UltrasonicSensor ultrasonicSensor(EPort::PORT_F); // ★超音波センサーを初期化 (ポートFを仮定)

  // スタート待機
  printf("Press Force Sensor to Start\n");
  while (!forceSensor.isTouched()) {
    clock.sleep(100 * 1000);
  }
  printf("Force Sensor Pressed, Start!\n");
  clock.sleep(500 * 1000);

  // 制御用変数
  RobotState currentState = RobotState::TRACE_BLACK;
  float integral = 0;
  int previousError = TARGET_BLACK - colorSensor.getReflection();
  int lap_count = 0;
  int blue_detect_count = 0;
  int black_detect_count = 0;
  int blue_ignore_timer = 0;

  while (1) {

/*
//-------------------------------------------------------------------------
// ★★★ここから障害物回避ロジックを追加★★★
//-------------------------------------------------------------------------
if (ultrasonicSensor.getDistance() < OBSTACLE_THRESHOLD_CM) {
  printf("Obstacle detected! Starting avoidance maneuver.\n");

  // 1. その場で停止
  leftWheel.stop();
  rightWheel.stop();
  clock.sleep(200 * 1000);

  // 2. 左に1秒間その場で旋回
  printf("左に旋回...\n");
  leftWheel.setPower(-AVOIDANCE_TURN_POWER);
  rightWheel.setPower(AVOIDANCE_TURN_POWER);
  clock.sleep(1500 * 1000);

  // 3. 1秒間前進
  printf("直進...\n");
  leftWheel.setPower(BASE_POWER);
  rightWheel.setPower(BASE_POWER);
  clock.sleep(1800 * 1000);

  // 4. 右に2秒間その場で旋回
  printf("右に旋回...\n");
  leftWheel.setPower(AVOIDANCE_TURN_POWER);
  rightWheel.setPower(-AVOIDANCE_TURN_POWER);
  clock.sleep(1800 * 1000);

  // 5. 黒ラインを検出するまで低速で前進
  printf("黒ラインを検出するまで直進...\n");
  leftWheel.setPower(SLOW_POWER);
  rightWheel.setPower(SLOW_POWER);
  while(colorSensor.getReflection() > TARGET_BLACK) {
      // ラインを見つけるまでループ
      clock.sleep(CONTROL_INTERVAL);
  }

  // ラインを見つけたら停止
  printf("黒ライン検出.\n");
  leftWheel.stop();
  rightWheel.stop();
  clock.sleep(500 * 1000);

  // PID制御の変数をリセットして、次回ループから正常にトレースを再開
  integral = 0;
  previousError = TARGET_BLACK - colorSensor.getReflection();
  
  // 回避動作が完了したので、以降のライントレース処理をスキップしてループの先頭に戻る
  continue;
}
//-------------------------------------------------------------------------
// ★★★障害物回避ロジックここまで★★★
//-------------------------------------------------------------------------
*/

// 現在の状態に応じてパラメータを選択
float Kp, Ki, Kd;
int target;
if (currentState == RobotState::TRACE_BLUE) {
  Kp = KP_BLUE; Ki = KI_BLUE; Kd = KD_BLUE; target = TARGET_BLUE;
} else {
  Kp = KP_BLACK; Ki = KI_BLACK; Kd = KD_BLACK; target = TARGET_BLACK;
}

// PID計算
int32_t reflection = colorSensor.getReflection();
int error = target - reflection;
integral += error * (CONTROL_INTERVAL / 1000000.0f);
integral = std::max(-INTEGRAL_MAX, std::min(INTEGRAL_MAX, integral));
float derivative = (error - previousError) / (CONTROL_INTERVAL / 1000000.0f);
float turn = Kp * error + Ki * integral + Kd * derivative;

// 色情報の判定
ColorSensor::HSV hsv;
colorSensor.getHSV(hsv, true);
bool is_turning_sharply = std::abs(turn) > SHARP_TURN_THRESHOLD;
bool is_blue_candidate = (hsv.h >= BLUE_HUE_MIN && hsv.h <= BLUE_HUE_MAX &&
                          hsv.s >= BLUE_SATURATION_MIN &&
                          hsv.v >= BLUE_VALUE_MIN && hsv.v <= BLUE_VALUE_MAX);
bool is_black = (hsv.v <= BLACK_VALUE_MAX);
bool is_blue = (currentState == RobotState::TRACE_BLACK) ? (is_blue_candidate && !is_turning_sharply) : is_blue_candidate;

// 状態遷移の判定
if (currentState == RobotState::TRACE_BLACK) {
  if (blue_ignore_timer > 0) {
    blue_ignore_timer--;
  } else {
    if (is_blue) {
      blue_detect_count++;
    } else {
      blue_detect_count = 0;
    }
    if (blue_detect_count >= SWITCH_CONFIRM_COUNT) {
      printf("青ライン検出：青ライントレースへ(Lap: %d)\n", lap_count);
      currentState = RobotState::TRACE_BLUE;
      previousError = TARGET_BLUE - colorSensor.getReflection();
      integral = 0;
    }
  }
} else if (currentState == RobotState::TRACE_BLUE) {
  if (is_black && !is_blue_candidate) {
    black_detect_count++;
  } else {
    black_detect_count = 0;
  }
  if (black_detect_count >= SWITCH_CONFIRM_COUNT) {
    if (lap_count <= 3) {
      printf("青ライン離脱：(Lap: %d), 直進.\n", lap_count);
      leftWheel.setPower(BASE_POWER);
      rightWheel.setPower(BASE_POWER);
      clock.sleep(STRAIGHT_DURATION_MS * 1000);
      blue_ignore_timer = BLUE_IGNORE_CYCLES;
    }
    printf("黒ライン検出：黒ラインへ\n");
    currentState = RobotState::TRACE_BLACK;
    lap_count++;
    previousError = TARGET_BLACK - colorSensor.getReflection();
    integral = 0;
  }
}

// モーター出力の決定
int currentBasePower;
if (lap_count >= 1 && lap_count <= 3) {
  currentBasePower = SLOW_POWER;
} else {
  currentBasePower = BASE_POWER;
}

bool trace_on_right_edge = (lap_count == 2 || lap_count == 4); //lap_countが2、4の時だけ右エッジトレース

int leftPower, rightPower;
if (trace_on_right_edge) {
  leftPower = currentBasePower + static_cast<int>(turn);
  rightPower = currentBasePower - static_cast<int>(turn);
} else {
  leftPower = currentBasePower - static_cast<int>(turn);
  rightPower = currentBasePower + static_cast<int>(turn);
}

leftWheel.setPower(std::max(-100, std::min(100, leftPower)));
rightWheel.setPower(std::max(-100, std::min(100, rightPower)));

previousError = error;
clock.sleep(CONTROL_INTERVAL);
  }
}
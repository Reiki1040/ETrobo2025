// 【キャリブレーション反映版】ダブルループ攻略プログラム
#include "app.h"
#include <stdio.h>
#include <algorithm>

#include "Motor.h"
#include "Clock.h"
#include "ColorSensor.h"
#include "ForceSensor.h"

using namespace spikeapi;

// ロボットの動作状態を定義
enum class RobotState {
  TRACE_BLACK,    // 1. 黒ラインをPID制御でトレース
  TRACE_BLUE      // 2. 青ラインを周回数に応じてトレース方法を変える
};

extern "C" void main_task(intptr_t exinf) {
  // --- デバイスの初期化 ---
  Motor leftWheel(EPort::PORT_B, Motor::EDirection::COUNTERCLOCKWISE, true);
  Motor rightWheel(EPort::PORT_A, Motor::EDirection::CLOCKWISE, true);
  ColorSensor colorSensor(EPort::PORT_C);
  Clock clock;
  ForceSensor forceSensor(EPort::PORT_D);

  // --- パラメータ調整 ---
  const uint32_t interval = 10 * 1000;
  const int basePower = 11;

  // 【モード1】黒ライン用の安定したPIDパラメータ
  const float Kp_black = 1.2f;
  const float Ki_black = 0.05f;
  const float Kd_black = 0.8f;
  const int target_black = 17;

  // 【モード2】青ライン用のPIDパラメータ
  const float Kp_blue = 1.0f;
  const float Ki_blue = 0.08f;
  const float Kd_blue = 1.0f;
  const int target_blue = 21;

  // ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼
  // ★★★ 2周目黒ライン用のON/OFF制御パラメータを追加 ★★★
  const float Kp_black_onoff = 5.0f; // Pゲインを極端に大きくする
  const float Ki_black_onoff = 0.0f;
  const float Kd_black_onoff = 0.0f;
  // ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

  // ★★★ 色判別のためのHSV閾値 ★★★
  const int BLUE_HUE_MIN = 215;
  const int BLUE_HUE_MAX = 250;
  const int BLUE_SATURATION_MIN = 40;
  const int BLACK_VALUE_MAX = 15;

  // ★★★ スタート待機処理 ★★★
  printf("Press Force Sensor to Start\n");
  while(!forceSensor.isTouched()) {
    clock.sleep(100 * 1000);
  }
  printf("Force Sensor Pressed, Start!\n");
  clock.sleep(500 * 1000);

  // --- 制御用変数 ---
  RobotState currentState = RobotState::TRACE_BLACK;
  float integral = 0;
  
  // ★★★ 微分キックを防止する修正 ★★★
  int32_t initial_reflection = colorSensor.getReflection();
  int previousError = target_black - initial_reflection;
  
  int lap_count = 0;
  int blue_detect_count = 0;
  int black_detect_count = 0;
  const int SWITCH_CONFIRM_COUNT = 3;

  while (1) {
    // --- 1. 色情報の取得と判定 ---
    ColorSensor::HSV hsv;
    colorSensor.getHSV(hsv, true);
    bool is_blue = (hsv.h >= BLUE_HUE_MIN && hsv.h <= BLUE_HUE_MAX && hsv.s >= BLUE_SATURATION_MIN);
    bool is_black = (hsv.v <= BLACK_VALUE_MAX);

    // --- 2. 状態の遷移判定 ---
    if (currentState == RobotState::TRACE_BLACK) {
      if (is_blue) {
        blue_detect_count++;
      } else {
        blue_detect_count = 0;
      }
      if (blue_detect_count >= SWITCH_CONFIRM_COUNT) {
        printf("青ライン検知！ 青トレースモードに切り替え (周回: %d)\n", lap_count);
        currentState = RobotState::TRACE_BLUE;
        previousError = target_blue - colorSensor.getReflection();
        integral = 0;
      }
    } else if (currentState == RobotState::TRACE_BLUE) {
      if (is_black && !is_blue) {
        black_detect_count++;
      } else {
        black_detect_count = 0;
      }
      if (black_detect_count >= SWITCH_CONFIRM_COUNT) {
        printf("黒ライン復帰！ PID制御を再開します\n");
        currentState = RobotState::TRACE_BLACK;
        lap_count++;
        previousError = target_black - colorSensor.getReflection();
        integral = 0;
      }
    }

    // --- 3. 現在の状態に応じたPIDパラメータを選択 ---
    float Kp, Ki, Kd;
    int target;
    if (currentState == RobotState::TRACE_BLUE) {
      Kp = Kp_blue; Ki = Ki_blue; Kd = Kd_blue; target = target_blue;
    } else { // currentState == RobotState::TRACE_BLACK
      // ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼
      // ★★★ lap_countが2の時だけ、黒ラインをON/OFF制御する ★★★
      if (lap_count == 2) {
        Kp = Kp_black_onoff; Ki = Ki_black_onoff; Kd = Kd_black_onoff; target = target_black;
      } else {
        Kp = Kp_black; Ki = Ki_black; Kd = Kd_black; target = target_black;
      }
      // ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲
    }

    // --- 4. PID計算 ---
    int32_t reflection = colorSensor.getReflection();
    int error = target - reflection;
    integral += error * (interval / 1000000.0f);
    float derivative = (error - previousError) / (interval / 1000000.0f);
    float turn = Kp * error + Ki * integral + Kd * derivative;

    // --- 5. 周回数に応じてモーター出力を決定 ---
    int leftPower, rightPower;
    const char* trace_edge_mode;
    
    bool trace_on_right_edge = (lap_count <= 1) || (lap_count >= 2 && lap_count % 2 != 0);

    if (trace_on_right_edge) {
      // 【右エッジトレース】
      leftPower = basePower + static_cast<int>(turn);
      rightPower = basePower - static_cast<int>(turn);
      trace_edge_mode = "Right";
    } else {
      // 【左エッジトレース】
      leftPower = basePower - static_cast<int>(turn);
      rightPower = basePower + static_cast<int>(turn);
      trace_edge_mode = "Left ";
    }

    leftWheel.setPower(std::max(-100, std::min(100, leftPower)));
    rightWheel.setPower(std::max(-100, std::min(100, rightPower)));

    previousError = error;
    clock.sleep(interval);
  }
}

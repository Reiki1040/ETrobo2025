#include <stdio.h>
#include <algorithm>
#include <cmath> // std::absを使用するために追加

#include "Motor.h"
#include "Clock.h"
#include "ColorSensor.h"
#include "ForceSensor.h"

using namespace spikeapi;

//パラメータ定義
const int BASE_POWER = 70; // 基準となる走行速度
const int SLOW_POWER = 35; // 低速走行時の速度

//黒ライン用PIDパラメータ
const float KP_BLACK = 1.0f;
const float KI_BLACK = 0.0f;
const float KD_BLACK = 0.0f;
const int TARGET_BLACK = 17;

//青ライン用PIDパラメータ
const float KP_BLUE = 1.0f;
const float KI_BLUE = 0.08f;
const float KD_BLUE = 1.0f;
const int TARGET_BLUE = 21;

//積分値上限
const float INTEGRAL_MAX = 100.0f;

//色判別HSV閾値
const int BLUE_HUE_MIN = 210;
const int BLUE_HUE_MAX = 240;
const int BLUE_SATURATION_MIN = 55;
const int BLUE_VALUE_MIN = 150;
const int BLUE_VALUE_MAX = 230;
const int BLACK_VALUE_MAX = 16;

//制御設定
const uint32_t CONTROL_INTERVAL = 10 * 1000; // 制御周期(10ms)
const int SWITCH_CONFIRM_COUNT = 1; // 状態遷移に必要な色検知の連続回数
const int SHARP_TURN_THRESHOLD = 60; // 急旋回と判断するturn値の閾値
const uint32_t STRAIGHT_DURATION_MS = 150; // 青ライン通過後の直進時間
const int BLUE_IGNORE_CYCLES = 200; // 青検知を無視するサイクル数 (200 * 10ms = 2s)

enum class RobotState {
  TRACE_BLACK,
  TRACE_BLUE
};

extern "C" void main_task(intptr_t exinf) {
  //デバイスの初期化
  Motor leftWheel(EPort::PORT_B, Motor::EDirection::COUNTERCLOCKWISE, true);
  Motor rightWheel(EPort::PORT_A, Motor::EDirection::CLOCKWISE, true);
  ColorSensor colorSensor(EPort::PORT_C);
  Clock clock;
  ForceSensor forceSensor(EPort::PORT_D);

  //スタート待機
  printf("Press Force Sensor to Start\n");
  while (!forceSensor.isTouched()) {
    clock.sleep(100 * 1000);
  }
  printf("Force Sensor Pressed, Start!\n");
  clock.sleep(500 * 1000);

  //制御用変数
  RobotState currentState = RobotState::TRACE_BLACK;
  float integral = 0;
  int previousError = TARGET_BLACK - colorSensor.getReflection(); // 微分キック防止
  int lap_count = 0;
  int blue_detect_count = 0;
  int black_detect_count = 0;
  int blue_ignore_timer = 0;

  while (1) {
    //現在の状態に応じてパラメータを選択
    float Kp, Ki, Kd;
    int target;
    if (currentState == RobotState::TRACE_BLUE) {
      Kp = KP_BLUE; Ki = KI_BLUE; Kd = KD_BLUE; target = TARGET_BLUE;
    } else {
      Kp = KP_BLACK; Ki = KI_BLACK; Kd = KD_BLACK; target = TARGET_BLACK;
    }

    //PID計算
    int32_t reflection = colorSensor.getReflection();
    int error = target - reflection;
    integral += error * (CONTROL_INTERVAL / 1000000.0f);
    
    //積分ワインドアップ対策
    integral = std::max(-INTEGRAL_MAX, std::min(INTEGRAL_MAX, integral));

    float derivative = (error - previousError) / (CONTROL_INTERVAL / 1000000.0f);
    float turn = Kp * error + Ki * integral + Kd * derivative;

    //色情報の判定
    ColorSensor::HSV hsv;
    colorSensor.getHSV(hsv, true);
    
    //急旋回中はラインの縁の誤認識が起きやすいため、青検知の条件を厳しく
    bool is_turning_sharply = std::abs(turn) > SHARP_TURN_THRESHOLD;
    
    bool is_blue_candidate = (hsv.h >= BLUE_HUE_MIN && hsv.h <= BLUE_HUE_MAX &&
                              hsv.s >= BLUE_SATURATION_MIN &&
                              hsv.v >= BLUE_VALUE_MIN && hsv.v <= BLUE_VALUE_MAX);
    bool is_black = (hsv.v <= BLACK_VALUE_MAX);
    
    //黒トレース中、急旋回中の青検知を無効化
    bool is_blue = (currentState == RobotState::TRACE_BLACK) ? (is_blue_candidate && !is_turning_sharply) : is_blue_candidate;

    //状態遷移の判定
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
          printf("Blue line detected: Switching to TRACE_BLUE (Lap: %d)\n", lap_count);
          currentState = RobotState::TRACE_BLUE;
          previousError = TARGET_BLUE - colorSensor.getReflection();
          integral = 0; //状態遷移時に積分値をリセット
        }
      }
    } else if (currentState == RobotState::TRACE_BLUE) {
      if (is_black && !is_blue_candidate) {
        black_detect_count++;
      } else {
        black_detect_count = 0;
      }
      if (black_detect_count >= SWITCH_CONFIRM_COUNT) {
        //青ラインを抜け、黒ラインに復帰したと判断
        if (lap_count <= 3) {
          printf("Exiting blue line (Lap: %d), proceeding straight.\n", lap_count);
          leftWheel.setPower(BASE_POWER);
          rightWheel.setPower(BASE_POWER);
          clock.sleep(STRAIGHT_DURATION_MS * 1000);
          blue_ignore_timer = BLUE_IGNORE_CYCLES; //復帰直後は青を検知しない
        }
        
        printf("Black line detected: Switching back to TRACE_BLACK\n");
        currentState = RobotState::TRACE_BLACK;
        lap_count++;
        previousError = TARGET_BLACK - colorSensor.getReflection();
        integral = 0; //状態遷移時に積分値をリセット
      }
    }

    int currentBasePower;
      // 1周目から3周目(lap_countが1, 2, 3)の間は低速走行
      if (lap_count >= 1 && lap_count <= 3) {
        currentBasePower = SLOW_POWER;
      } else {
        currentBasePower = BASE_POWER;
      }
    
    bool trace_on_right_edge = (lap_count <= 1) || (lap_count >= 2 && lap_count % 2 != 0);

    int leftPower, rightPower;
    if (trace_on_right_edge) {
      //右エッジトレース: turn値が大きい(ラインから右にズレた)とき、左に曲がる
      leftPower = currentBasePower + static_cast<int>(turn);
      rightPower = currentBasePower - static_cast<int>(turn);
    } else {
      //左エッジトレース: turn値が大きい(ラインから左にズレた)とき、右に曲がる
      leftPower = currentBasePower - static_cast<int>(turn);
      rightPower = currentBasePower + static_cast<int>(turn);
    }
    
    leftWheel.setPower(std::max(-100, std::min(100, leftPower)));
    rightWheel.setPower(std::max(-100, std::min(100, rightPower)));
    
    previousError = error;
    clock.sleep(CONTROL_INTERVAL);
  }
}
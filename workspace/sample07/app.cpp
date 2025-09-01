//シミュレータ用に調整したプログラム
#include "app.h"
#include <stdio.h>
#include <algorithm>

#include "Motor.h"
#include "Clock.h"
#include "ColorSensor.h"
#include "ForceSensor.h"

using namespace spikeapi;

enum class RobotState {
  TRACE_BLACK,  //黒ラインをPID制御でトレース
  TRACE_BLUE    //青ライン検知数に応じてトレース方法を変える
};

extern "C" void main_task(intptr_t exinf) {
  
  Motor leftWheel(EPort::PORT_B, Motor::EDirection::COUNTERCLOCKWISE, true);
  Motor rightWheel(EPort::PORT_A, Motor::EDirection::CLOCKWISE, true);
  ColorSensor colorSensor(EPort::PORT_C);
  Clock clock;
  ForceSensor forceSensor(EPort::PORT_D);

  //パラメータ調整
  const uint32_t interval = 10 * 1000;
  const int basePower = 15;

  //黒ライン用PIDパラメータ
  const float Kp_black = 1.2f;
  const float Ki_black = 0.05f;
  const float Kd_black = 0.8f;
  const int target_black = 17;

  //青ライン用のPIDパラメータ
  const float Kp_blue = 1.0f;
  const float Ki_blue = 0.08f;
  const float Kd_blue = 1.0f;
  const int target_blue = 21;

  //色判別のためのHSV閾値
  const int BLUE_HUE_MIN = 215;
  const int BLUE_HUE_MAX = 250;
  const int BLUE_SATURATION_MIN = 40;
  const int BLACK_VALUE_MAX = 15;

  //スタート待機処理
  printf("Press Force Sensor to Start\n");
  while(!forceSensor.isTouched()) {
    clock.sleep(100 * 1000);
  }
  printf("Force Sensor Pressed, Start!\n");
  clock.sleep(500 * 1000);

  //制御用変数 
  RobotState currentState = RobotState::TRACE_BLACK;
  float integral = 0;
  
  //微分キックを防止する修正
  int32_t initial_reflection = colorSensor.getReflection();
  int previousError = target_black - initial_reflection;
  
  int lap_count = 0;
  int blue_detect_count = 0;
  int black_detect_count = 0;
  const int SWITCH_CONFIRM_COUNT = 2;
  
  //青ライン誤判定防止用タイマー
  int blue_ignore_timer = 0;

  while (1) {
    //色情報の取得、判定
    ColorSensor::HSV hsv;
    colorSensor.getHSV(hsv, true);
    bool is_blue = (hsv.h >= BLUE_HUE_MIN && hsv.h <= BLUE_HUE_MAX && hsv.s >= BLUE_SATURATION_MIN);
    bool is_black = (hsv.v <= BLACK_VALUE_MAX);

    //状態の遷移判定
    if (currentState == RobotState::TRACE_BLACK) {
      //青ライン無視タイマーが作動中か
      if (blue_ignore_timer > 0) {
        blue_ignore_timer--; //タイマーを1減らす
      } else {
        //タイマーが0の時だけ、通常通り青ラインを検知
        if (is_blue) {
          blue_detect_count++;
        } else {
          blue_detect_count = 0;
        }
        if (blue_detect_count >= SWITCH_CONFIRM_COUNT) {
          printf("青ライン検知： 青トレースに切り替え (検知回数: %d)\n", lap_count);
          currentState = RobotState::TRACE_BLUE;
          previousError = target_blue - colorSensor.getReflection();
          integral = 0;
        }
      }
    } else if (currentState == RobotState::TRACE_BLUE) {
      if (is_black && !is_blue) {
        black_detect_count++;
      } else {
        black_detect_count = 0;
      }
      if (black_detect_count >= SWITCH_CONFIRM_COUNT) {

        if (lap_count == 1 || lap_count == 2 || lap_count == 3) {
          printf("%d検知目完了、進行のため0.5秒直進\n", lap_count + 1);
          leftWheel.setPower(basePower);
          rightWheel.setPower(basePower);
          clock.sleep(500 * 1000); //0.5秒直進
          
          //タイマーをセット！(100ループ = 1秒間、青を無視)
          blue_ignore_timer = 200;
        }
        
        printf("黒ライン：PID制御を再開\n");
        currentState = RobotState::TRACE_BLACK;
        lap_count++;
        previousError = target_black - colorSensor.getReflection();
        integral = 0;
      }
    }

    //PID制御
    float Kp, Ki, Kd;
    int target;
    if (currentState == RobotState::TRACE_BLUE) {
      Kp = Kp_blue; Ki = Ki_blue; Kd = Kd_blue; target = target_blue;
    } else {
      Kp = Kp_black; Ki = Ki_black; Kd = Kd_black; target = target_black;
    }
    int32_t reflection = colorSensor.getReflection();
    int error = target - reflection;
    integral += error * (interval / 1000000.0f);
    float derivative = (error - previousError) / (interval / 1000000.0f);
    float turn = Kp * error + Ki * integral + Kd * derivative;

    int currentBasePower = (currentState == RobotState::TRACE_BLACK && lap_count == 2) ? 8 : basePower;
    int leftPower, rightPower;
    bool trace_on_right_edge = (lap_count <= 1) || (lap_count >= 2 && lap_count % 2 != 0);

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
    clock.sleep(interval);
  }
}

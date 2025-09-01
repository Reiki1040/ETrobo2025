#include <stdio.h>
#include <algorithm>

#include "Motor.h"
#include "Clock.h"
#include "ColorSensor.h"
#include "ForceSensor.h"

using namespace spikeapi;

const int basePower = 40; //基準となる走行速度
const int slowPower = 20;  //2回目の青ラインを抜けた後の低速走行時の速度

//黒ライン用PIDパラメータ
const float Kp_black = 0.9f;  //比例ゲイン
const float Ki_black = 0.0f; //積分ゲイン
const float Kd_black = 0.0f;  //微分ゲイン
const int target_black = 17;  //黒ラインの目標反射値、白と黒の反射値の間が理想的

//青ライン用PIDパラメータ
const float Kp_blue = 1.0f;   //比例ゲイン
const float Ki_blue = 0.08f;  //積分ゲイン
const float Kd_blue = 1.0f;   //微分ゲイン
const int target_blue = 21;   //青ラインの目標反射値、白と青の反射値の間が理想的


//色判別のためのHSV閾値
const int BLUE_HUE_MIN = 210;      //青と判断する色相(H)の最小値
const int BLUE_HUE_MAX = 240;      //青と判断する色相(H)の最大値
const int BLUE_SATURATION_MIN = 55;//青と判断する彩度(S)の最小値
const int BLUE_VALUE_MIN = 150;    //青と判断する明度(V)の最小値
const int BLUE_VALUE_MAX = 230;    //青と判断する明度(V)の最大値
const int BLACK_VALUE_MAX = 16;    //黒と判断する明度(V)の最大値

const uint32_t interval = 10 * 1000; //制御周期(10ms)
const int SWITCH_CONFIRM_COUNT = 1; //色検知の連続回数
const int SHARP_TURN_THRESHOLD = 60; //この値よりturnの絶対値が大きいと急旋回と判断

enum class RobotState {
  TRACE_BLACK,  //黒ラインをPID制御でトレース
  TRACE_BLUE    //青ライン検知数に応じてトレース方法を変更
};

extern "C" void main_task(intptr_t exinf) {
  //デバイスの初期化
  Motor leftWheel(EPort::PORT_B, Motor::EDirection::COUNTERCLOCKWISE, true);
  Motor rightWheel(EPort::PORT_A, Motor::EDirection::CLOCKWISE, true);
  ColorSensor colorSensor(EPort::PORT_C);
  Clock clock;
  ForceSensor forceSensor(EPort::PORT_D);

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
  
  //青ライン誤判定防止用タイマー
  int blue_ignore_timer = 0;

  while (1) {
    float Kp, Ki, Kd;
    int target;
    if (currentState == RobotState::TRACE_BLUE) {
      Kp = Kp_blue; Ki = Ki_blue; Kd = Kd_blue; target = target_blue;
    } else {
      Kp = Kp_black; Ki = Ki_black; Kd = Kd_black; target = target_black;
    }

    //センサー値の取得とPID計算
    ColorSensor::HSV hsv;
    colorSensor.getHSV(hsv, true);
    int32_t reflection = colorSensor.getReflection();
    int error = target - reflection;
    integral += error * (interval / 1000000.0f);
    float derivative = (error - previousError) / (interval / 1000000.0f);
    float turn = Kp * error + Ki * integral + Kd * derivative;

    //色情報の判定（急旋回中の誤判定防止)
    bool is_blue_candidate = (hsv.h >= BLUE_HUE_MIN && hsv.h <= BLUE_HUE_MAX &&
                              hsv.s >= BLUE_SATURATION_MIN &&
                              hsv.v >= BLUE_VALUE_MIN && hsv.v <= BLUE_VALUE_MAX);
    bool is_black = (hsv.v <= BLACK_VALUE_MAX);
    
    //急旋回中（turnの絶対値が大きい時)は青ラインの誤判定とみなし、青検知を無効化
    bool is_turning_sharply = std::abs(turn) > SHARP_TURN_THRESHOLD;
    bool is_blue = (currentState == RobotState::TRACE_BLACK) ? (is_blue_candidate && !is_turning_sharply) : is_blue_candidate;


    //状態の遷移判定
    if (currentState == RobotState::TRACE_BLACK) {
      if (blue_ignore_timer > 0) {
        blue_ignore_timer--;
      } else {
        if (is_blue) { //フィルタリングされたis_blueを使用
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
      if (is_black && !is_blue_candidate) { //ここでは元の判定結果を使用
        black_detect_count++;
      } else {
        black_detect_count = 0;
      }
      if (black_detect_count >= SWITCH_CONFIRM_COUNT) {
        if (lap_count == 0 || lap_count == 1 || lap_count == 2 || lap_count == 3) {
          printf("%d検知目抜け出し、進行のため0.15秒直進\n", lap_count + 1);
          leftWheel.setPower(basePower);
          rightWheel.setPower(basePower);
          clock.sleep(150 * 1000);
          blue_ignore_timer = 200;
        }
        
        printf("黒ライン：PID制御を再開\n");
        currentState = RobotState::TRACE_BLACK;
        lap_count++;
        previousError = target_black - colorSensor.getReflection();
        integral = 0;
      }
    }

    //モーター出力の決定
    int currentBasePower = (currentState == RobotState::TRACE_BLACK && lap_count == 2) ? 20 : basePower;
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

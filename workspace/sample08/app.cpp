//【キャリブレーション反映版】ダブルループ攻略プログラム
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
  
  //デバイスの初期化
  Motor leftWheel(EPort::PORT_B, Motor::EDirection::COUNTERCLOCKWISE, true); //左ホイールをBポートに設定
  Motor rightWheel(EPort::PORT_A, Motor::EDirection::CLOCKWISE, true);      //右ホイールをAポートに設定
  ColorSensor colorSensor(EPort::PORT_C);                                   //カラーセンサーをCポートに設定
  Clock clock;                                                              //クロックを初期化
  ForceSensor forceSensor(EPort::PORT_D);                                   //フォースセンサーをDポートに設定

  //パラメータ調整
  const uint32_t interval = 10 * 1000; //制御周期(10ms)
  const int basePower = 15;            //基準出力

  //黒ライン用PIDパラメータ
  const float Kp_black = 1.2f;  //比例ゲイン
  const float Ki_black = 0.05f; //積分ゲイン
  const float Kd_black = 0.8f;  //微分ゲイン
  const int target_black = 17;  //黒ラインの目標反射値、白と黒の反射値の間が理想的

  //青ライン用PIDパラメータ
  const float Kp_blue = 1.0f;   //比例ゲイン
  const float Ki_blue = 0.08f;  //積分ゲイン
  const float Kd_blue = 1.0f;   //微分ゲイン
  const int target_blue = 21;   //青ラインの目標反射値、白と青の反射値の間が理想的

  //色判別のためのHSV閾値
  const int BLUE_HUE_MIN = 215;      //青と判断する色相(H)の最小値
  const int BLUE_HUE_MAX = 250;      //青と判断する色相(H)の最大値
  const int BLUE_SATURATION_MIN = 40;//青と判断する彩度(S)の最小値
  const int BLACK_VALUE_MAX = 15;    //黒と判断する明度(V)の最大値

  //スタート待機処理
  printf("Press Force Sensor to Start\n"); 
  while(!forceSensor.isTouched()) {        //フォースセンサーがタッチされるまで待機
    clock.sleep(100 * 1000);
  }
  printf("Force Sensor Pressed, Start!\n"); //スタートを通知
  clock.sleep(500 * 1000);

  //制御用変数 
  RobotState currentState = RobotState::TRACE_BLACK; //現在の状態を黒ライントレースに設定
  float integral = 0;                                //積分項初期化
  
  //微分キックを防止する修正
  int32_t initial_reflection = colorSensor.getReflection();       //最初の反射値を取得
  int previousError = target_black - initial_reflection;          //最初の誤差を計算し、前回の誤差として保存
  
  int lap_count = 0;              //青ラインを検知した回数を初期化
  int blue_detect_count = 0;      //青ラインの連続検知カウンターを初期化
  int black_detect_count = 0;     //黒ラインの連続検知カウンターを初期化
  const int SWITCH_CONFIRM_COUNT = 2; //2回連続で色を検知したら状態を切り替える、即時検知で暴れるのを防ぐため
  
  //青ライン誤判定防止用タイマー
  int blue_ignore_timer = 0; //タイマーを初期化

  while (1) {
    //色情報の取得、判定
    ColorSensor::HSV hsv;                                                                      //HSV値を格納する構造体を宣言
    colorSensor.getHSV(hsv, true);                                                             //現在のHSV値を取得
    bool is_blue = (hsv.h >= BLUE_HUE_MIN && hsv.h <= BLUE_HUE_MAX && hsv.s >= BLUE_SATURATION_MIN); //青色かどうかを判定
    bool is_black = (hsv.v <= BLACK_VALUE_MAX);                                                //黒色かどうかを判定

    //状態の遷移判定
    if (currentState == RobotState::TRACE_BLACK) { //現在が黒ライントレース状態の場合
      //青ライン無視タイマーが作動中か
      if (blue_ignore_timer > 0) { //タイマーが0より大きい場合
        blue_ignore_timer--;       //タイマーを1減らす
      } else {                     //タイマーが0の場合
        //タイマーが0の時だけ、通常通り青ラインを検知
        if (is_blue) {             //青色を検知した場合
          blue_detect_count++;     //青色検知カウンターを増やす
        } else {                   //青色でない場合
          blue_detect_count = 0;   //カウンターをリセット
        }
        if (blue_detect_count >= SWITCH_CONFIRM_COUNT) { //青色を規定回数連続で検知した場合
          printf("青ライン検知： 青トレースに切り替え (検知回数: %d)\n", lap_count); //切り替えを通知
          currentState = RobotState::TRACE_BLUE;                                  //状態を青ライントレースに切り替え
          previousError = target_blue - colorSensor.getReflection();              //新しい目標値で前回の誤差を再計算
          integral = 0;                                                           //積分値をリセット
        }
      }
    } else if (currentState == RobotState::TRACE_BLUE) { //現在が青ライントレース状態の場合
      if (is_black && !is_blue) {                        //黒色かつ青色でない場合
        black_detect_count++;                            //黒色検知カウンターを増やす
      } else {                                           //それ以外の場合
        black_detect_count = 0;                          //カウンターをリセット
      }
      if (black_detect_count >= SWITCH_CONFIRM_COUNT) { //黒色を規定回数連続で検知した場合

        if (lap_count == 1 || lap_count == 2 || lap_count == 3) { //青ライン検知回数が1,2,3回目の場合
          printf("%d検知目完了、進行のため0.5秒直進\n", lap_count + 1); //直進処理を通知
          leftWheel.setPower(basePower);                                //左ホイールを直進パワーに設定
          rightWheel.setPower(basePower);                               //右ホイールを直進パワーに設定
          clock.sleep(500 * 1000);                                      //0.5秒直進、これは確実に黒ラインを検知して欲しいから
          
          //タイマーをセット！(200ループ = 2秒間、青を無視)
          blue_ignore_timer = 200;
        }
        
        printf("黒ライン：PID制御を再開\n");                                  //黒ライントレース再開を通知
        currentState = RobotState::TRACE_BLACK;                           //状態を黒ライントレースに切り替え
        lap_count++;                                                      //青ライン検知回数を増やす
        previousError = target_black - colorSensor.getReflection();       //新しい目標値で前回の誤差を再計算
        integral = 0;                                                     //積分値をリセット
      }
    }

    //PID制御
    float Kp, Ki, Kd; //PIDゲイン変数を宣言
    int target;       //目標値変数を宣言
    if (currentState == RobotState::TRACE_BLUE) {       //現在が青ライントレース状態の場合
      Kp = Kp_blue; Ki = Ki_blue; Kd = Kd_blue; target = target_blue; //青ライン用のパラメータを使用
    } else {                                            //現在が黒ライントレース状態の場合
      Kp = Kp_black; Ki = Ki_black; Kd = Kd_black; target = target_black; //黒ライン用のパラメータを使用
    }
    int32_t reflection = colorSensor.getReflection();                  //現在の反射値を取得
    int error = target - reflection;                                   //目標値との誤差を計算
    integral += error * (interval / 1000000.0f);                       //積分項を計算
    float derivative = (error - previousError) / (interval / 1000000.0f); //微分項を計算
    float turn = Kp * error + Ki * integral + Kd * derivative;         //操作量(turn)を計算

    int currentBasePower = (currentState == RobotState::TRACE_BLACK && lap_count == 2) ? 8 : basePower; //2回目の青ライン抜け後だけ速度を落とす
    int leftPower, rightPower;                                                                         //左右のモーター出力変数を宣言
    bool trace_on_right_edge = (lap_count <= 1) || (lap_count >= 2 && lap_count % 2 != 0);             //右エッジをトレースするかどうかの条件

    if (trace_on_right_edge) { //右エッジをトレースする場合
      leftPower = currentBasePower + static_cast<int>(turn);  //左モーターの出力を計算
      rightPower = currentBasePower - static_cast<int>(turn); //右モーターの出力を計算
    } else {                   //左エッジをトレースする場合
      leftPower = currentBasePower - static_cast<int>(turn);  //左モーターの出力を計算
      rightPower = currentBasePower + static_cast<int>(turn); //右モーターの出力を計算
    }

    leftWheel.setPower(std::max(-100, std::min(100, leftPower)));   //左モーターの出力を-100から100の範囲に制限して設定
    rightWheel.setPower(std::max(-100, std::min(100, rightPower))); //右モーターの出力を-100から100の範囲に制限して設定
    
    previousError = error;      //今回の誤差を次回の計算のために保存
    clock.sleep(interval);      //指定した時間(10ms)待機
  }
}

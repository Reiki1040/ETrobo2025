//PID制御ver
#include "app.h"
#include <stdio.h>
#include <algorithm>

#include "Motor.h"
#include "Clock.h"
#include "ColorSensor.h"

using namespace spikeapi;

extern "C" void main_task(intptr_t exinf) {
  Motor leftWheel(EPort::PORT_B, Motor::EDirection::COUNTERCLOCKWISE, true);
  Motor rightWheel(EPort::PORT_A, Motor::EDirection::CLOCKWISE, true);
  ColorSensor colorSensor(EPort::PORT_C);

  Clock clock;

  //パラメータ調整
  const uint32_t interval = 10 * 1000;  //10ms感覚で制御する
  const int basePower = 10;

  const float Kp = 1.0f;  //比例ゲイン
  const float Ki = 0.05f; //積分ゲイン
  const float Kd = 0.8f;  //微分ゲイン

  const int target = 17;//だいたい白と黒の間がよき

  float integral = 0;
  int previousError = 0;

  while (1) {
    int32_t reflection = colorSensor.getReflection();  //現在の反射値
    int error = target - reflection;

    integral += error * (interval / 1000000.0f);  //積分項（時間を考慮）
    float derivative = (error - previousError) / (interval / 1000000.0f);  //微分項

    float turn = Kp * error + Ki * integral + Kd * derivative;

    int leftPower = basePower + static_cast<int>(turn);
    int rightPower = basePower - static_cast<int>(turn);

    //範囲制限
    leftPower = std::max(-100, std::min(100, leftPower));
    rightPower = std::max(-100, std::min(100, rightPower));

    leftWheel.setPower(leftPower);
    rightWheel.setPower(rightPower);

    printf("反射値: %ld, target: %d, 誤差: %d, 左: %d, 右: %d\n", reflection, target, error, leftPower, rightPower);

    previousError = error;
    clock.sleep(interval);
  }
}
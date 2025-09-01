// 直進して反射値を取得するプログラム
#include "app.h"
#include <stdio.h>
#include <algorithm>

#include "Motor.h"
#include "Clock.h"
#include "ColorSensor.h"

using namespace spikeapi;

extern "C" void main_task(intptr_t exinf) {
  // --- デバイスの初期化 ---
  Motor leftWheel(EPort::PORT_B, Motor::EDirection::COUNTERCLOCKWISE, true);
  Motor rightWheel(EPort::PORT_A, Motor::EDirection::CLOCKWISE, true);
  ColorSensor colorSensor(EPort::PORT_C);
  Clock clock;

  // --- パラメータ設定 ---
  // 100msごとに値を取得・表示する (取得頻度を少し落とす)
  const uint32_t interval = 50 * 1000;
  // 直進するときのモーター出力 (ゆっくり進むように低めに設定)
  const int straightPower = 6;

  // --- メインループ ---
  while (1) {
    // カラーセンサーから現在の反射値を取得
    int32_t reflection = colorSensor.getReflection();

    // 取得した反射値をコンソールに出力
    printf("現在の反射値: %ld\n", reflection);

    // 左右のモーターに同じ出力を設定して直進させる
    leftWheel.setPower(straightPower);
    rightWheel.setPower(straightPower);

    // 指定した時間だけ待機
    clock.sleep(interval);
  }
}

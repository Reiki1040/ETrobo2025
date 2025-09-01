// 直進しながらHSV値を検出・表示するキャリブレーション用プログラム
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
  // 100msごとに値を取得・表示する
  const uint32_t interval = 100 * 1000;
  // 直進するときのモーター出力 (ゆっくり進むように低めに設定)
  const int straightPower = 5;

  // --- メインループ ---
  while (1) {
    // カラーセンサーから現在のHSV値を取得
    ColorSensor::HSV hsv;
    colorSensor.getHSV(hsv, true); // trueは表面の色を測定するオプション

    // 取得したHSV値をコンソールに出力
    // H: 色相(0-360), S: 彩度(0-100), V: 明度(0-100)
    printf("H: %3u, S: %3u, V: %3u\n", hsv.h, hsv.s, hsv.v);

    // 左右のモーターに同じ出力を設定して直進させる
    leftWheel.setPower(straightPower);
    rightWheel.setPower(straightPower);

    // 指定した時間だけ待機
    clock.sleep(interval);
  }
}

#include "app.h"
#include <stdio.h>

#include "Motor.h"
#include "Clock.h"
#include "ColorSensor.h"

using namespace spikeapi;

extern "C" void main_task(intptr_t exinf) {

  printf("main_task started!\n");
  ext_tsk();

}

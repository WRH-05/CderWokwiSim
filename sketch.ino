#include <Arduino.h>
#include "robot_controller.h"

void setup() {
  RobotController::begin();
}

void loop() {
  RobotController::update();
}

#pragma once

#include <Arduino.h>

namespace RobotController {

enum class ScanState : uint8_t {
  Idle,
  StartCapture,
  WaitCapture,
  WaitXMove,
  WaitYMove,
  Complete,
  Fault
};

void begin();
void update();

}  // namespace RobotController

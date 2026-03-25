#include "robot_controller.h"

#include <AccelStepper.h>

#include "robot_config.h"

namespace RobotController {

namespace {

enum class AfterCaptureAction : uint8_t {
  MoveX,
  MoveY,
  Complete
};

enum class UltrasonicState : uint8_t {
  Idle,
  TriggerHigh,
  WaitEchoRise,
  WaitEchoFall
};

AccelStepper xStepper(AccelStepper::DRIVER, RobotConfig::X_STEP_PIN,
                      RobotConfig::X_DIR_PIN);
AccelStepper yStepper(AccelStepper::DRIVER, RobotConfig::Y_STEP_PIN,
                      RobotConfig::Y_DIR_PIN);

ScanState currentState = ScanState::Idle;
AfterCaptureAction afterCaptureAction = AfterCaptureAction::MoveX;

bool scanToRight = true;
bool xBoundaryHitDuringMove = false;
bool panelCompleteAnnounced = false;
bool cameraHigh = false;

uint32_t captureStartMs = 0;

UltrasonicState ultrasonicState = UltrasonicState::Idle;
uint32_t ultrasonicLastPollMs = 0;
uint32_t trigHighStartUs = 0;
uint32_t echoWaitStartUs = 0;
uint32_t echoRiseUs = 0;

float lastDistanceCm = -1.0F;
bool panelGapDetected = false;
uint8_t panelGapConsecutiveReadings = 0;

const __FlashStringHelper* stateToText(const ScanState state) {
  switch (state) {
    case ScanState::Idle:
      return F("Idle");
    case ScanState::StartCapture:
      return F("StartCapture");
    case ScanState::WaitCapture:
      return F("WaitCapture");
    case ScanState::WaitXMove:
      return F("WaitXMove");
    case ScanState::WaitYMove:
      return F("WaitYMove");
    case ScanState::Complete:
      return F("Complete");
    case ScanState::Fault:
      return F("Fault");
    default:
      return F("Unknown");
  }
}

void changeState(const ScanState nextState) {
  if (currentState == nextState) {
    return;
  }
  Serial.print(F("[STATE] "));
  Serial.print(stateToText(currentState));
  Serial.print(F(" -> "));
  Serial.print(stateToText(nextState));
  Serial.print(F(" @ ms="));
  Serial.println(millis());
  currentState = nextState;
}

void setMotorEnabled(const bool enabled) {
  const uint8_t signal = RobotConfig::DRIVER_ENABLE_ACTIVE_LOW
                             ? (enabled ? LOW : HIGH)
                             : (enabled ? HIGH : LOW);

  digitalWrite(RobotConfig::X_ENABLE_PIN, signal);
  digitalWrite(RobotConfig::Y_ENABLE_PIN, signal);
}

bool isLeftLimitPressed() {
  const bool rawPressed = digitalRead(RobotConfig::X_LIMIT_LEFT_PIN) == HIGH;
  return RobotConfig::LIMIT_SWITCH_ACTIVE_LOW ? !rawPressed : rawPressed;
}

bool isRightLimitPressed() {
  const bool rawPressed = digitalRead(RobotConfig::X_LIMIT_RIGHT_PIN) == HIGH;
  return RobotConfig::LIMIT_SWITCH_ACTIVE_LOW ? !rawPressed : rawPressed;
}

void stopAndComplete() {
  xStepper.stop();
  yStepper.stop();
  setMotorEnabled(false);

  digitalWrite(RobotConfig::CAMERA_TRIGGER_PIN, LOW);
  cameraHigh = false;

  changeState(ScanState::Complete);

  if (!panelCompleteAnnounced) {
    panelCompleteAnnounced = true;
    Serial.println(F("[DONE] Panel Complete"));
  }
}

void scheduleCapture(const AfterCaptureAction nextAction) {
  afterCaptureAction = nextAction;
  changeState(ScanState::StartCapture);
}

void prepareXMove() {
  if (panelGapDetected) {
    scheduleCapture(AfterCaptureAction::Complete);
    return;
  }

  xBoundaryHitDuringMove = false;

  const long signedStep = scanToRight ? RobotConfig::STEP_DISTANCE_X
                                      : -RobotConfig::STEP_DISTANCE_X;
  const long moveStep = RobotConfig::INVERT_X_DIRECTION ? -signedStep : signedStep;
  const long target = xStepper.currentPosition() + moveStep;

  xStepper.moveTo(target);

  Serial.print(F("[MOVE-X] dir="));
  Serial.print(scanToRight ? F("RIGHT") : F("LEFT"));
  Serial.print(F(" current="));
  Serial.print(xStepper.currentPosition());
  Serial.print(F(" target="));
  Serial.println(target);

  changeState(ScanState::WaitXMove);
}

void prepareYMove() {
  if (panelGapDetected) {
    scheduleCapture(AfterCaptureAction::Complete);
    return;
  }

  const long signedStep = RobotConfig::STEP_DISTANCE_Y;
  const long moveStep = RobotConfig::INVERT_Y_DIRECTION ? -signedStep : signedStep;
  const long target = yStepper.currentPosition() + moveStep;

  yStepper.moveTo(target);

  Serial.print(F("[MOVE-Y] current="));
  Serial.print(yStepper.currentPosition());
  Serial.print(F(" target="));
  Serial.println(target);

  changeState(ScanState::WaitYMove);
}

void updateUltrasonic() {
  const uint32_t nowMs = millis();
  const uint32_t nowUs = micros();

  switch (ultrasonicState) {
    case UltrasonicState::Idle:
      if (nowMs - ultrasonicLastPollMs >= RobotConfig::ULTRASONIC_POLL_INTERVAL_MS) {
        digitalWrite(RobotConfig::ULTRASONIC_TRIG_PIN, HIGH);
        trigHighStartUs = nowUs;
        ultrasonicState = UltrasonicState::TriggerHigh;
      }
      break;

    case UltrasonicState::TriggerHigh:
      if (nowUs - trigHighStartUs >= 10U) {
        digitalWrite(RobotConfig::ULTRASONIC_TRIG_PIN, LOW);
        echoWaitStartUs = nowUs;
        ultrasonicState = UltrasonicState::WaitEchoRise;
      }
      break;

    case UltrasonicState::WaitEchoRise:
      if (digitalRead(RobotConfig::ULTRASONIC_ECHO_PIN) == HIGH) {
        echoRiseUs = nowUs;
        ultrasonicState = UltrasonicState::WaitEchoFall;
      } else if (nowUs - echoWaitStartUs >= RobotConfig::ULTRASONIC_TIMEOUT_US) {
        ultrasonicState = UltrasonicState::Idle;
        ultrasonicLastPollMs = nowMs;
        Serial.println(F("[SENSOR] Ultrasonic timeout waiting for echo rise"));
      }
      break;

    case UltrasonicState::WaitEchoFall:
      if (digitalRead(RobotConfig::ULTRASONIC_ECHO_PIN) == LOW) {
        const uint32_t pulseUs = nowUs - echoRiseUs;
        lastDistanceCm = static_cast<float>(pulseUs) / 58.0F;
        const bool currentGap = lastDistanceCm > RobotConfig::PANEL_GAP_THRESHOLD_CM;

        if (currentGap) {
          if (panelGapConsecutiveReadings < 255U) {
            ++panelGapConsecutiveReadings;
          }
        } else {
          panelGapConsecutiveReadings = 0;
        }

        panelGapDetected = panelGapConsecutiveReadings >= RobotConfig::PANEL_GAP_CONFIRM_READINGS;

        Serial.print(F("[SENSOR] distance_cm="));
        Serial.print(lastDistanceCm, 1);
        Serial.print(F(" threshold="));
        Serial.print(RobotConfig::PANEL_GAP_THRESHOLD_CM, 1);
        Serial.print(F(" consecutive="));
        Serial.print(panelGapConsecutiveReadings);
        Serial.print(F(" gap="));
        Serial.println(panelGapDetected ? F("YES") : F("NO"));

        ultrasonicState = UltrasonicState::Idle;
        ultrasonicLastPollMs = nowMs;
      } else if (nowUs - echoRiseUs >= RobotConfig::ULTRASONIC_TIMEOUT_US) {
        ultrasonicState = UltrasonicState::Idle;
        ultrasonicLastPollMs = nowMs;
        Serial.println(F("[SENSOR] Ultrasonic timeout waiting for echo fall"));
      }
      break;

    default:
      ultrasonicState = UltrasonicState::Idle;
      break;
  }
}

void runStateMachine() {
  switch (currentState) {
    case ScanState::Idle:
      scheduleCapture(AfterCaptureAction::MoveX);
      break;

    case ScanState::StartCapture:
      digitalWrite(RobotConfig::CAMERA_TRIGGER_PIN, HIGH);
      cameraHigh = true;
      captureStartMs = millis();

      Serial.print(F("[CAM] HIGH start_ms="));
      Serial.print(captureStartMs);
      Serial.print(F(" x="));
      Serial.print(xStepper.currentPosition());
      Serial.print(F(" y="));
      Serial.println(yStepper.currentPosition());

      changeState(ScanState::WaitCapture);
      break;

    case ScanState::WaitCapture:
      if (millis() - captureStartMs >= RobotConfig::CAMERA_EXPOSURE_MS) {
        digitalWrite(RobotConfig::CAMERA_TRIGGER_PIN, LOW);
        cameraHigh = false;

        Serial.print(F("[CAM] LOW end_ms="));
        Serial.println(millis());

        switch (afterCaptureAction) {
          case AfterCaptureAction::MoveX:
            prepareXMove();
            break;
          case AfterCaptureAction::MoveY:
            prepareYMove();
            break;
          case AfterCaptureAction::Complete:
            stopAndComplete();
            break;
          default:
            changeState(ScanState::Fault);
            break;
        }
      }
      break;

    case ScanState::WaitXMove: {
      if (scanToRight && isRightLimitPressed()) {
        xBoundaryHitDuringMove = true;
        xStepper.stop();
      } else if (!scanToRight && isLeftLimitPressed()) {
        xBoundaryHitDuringMove = true;
        xStepper.stop();
      }

      if (xStepper.distanceToGo() == 0) {
        if (xBoundaryHitDuringMove) {
          Serial.print(F("[LIMIT] X "));
          Serial.println(scanToRight ? F("RIGHT triggered") : F("LEFT triggered"));

          scheduleCapture(AfterCaptureAction::MoveY);
        } else {
          scheduleCapture(AfterCaptureAction::MoveX);
        }
      }
      break;
    }

    case ScanState::WaitYMove:
      if (yStepper.distanceToGo() == 0) {
        if (panelGapDetected) {
          scheduleCapture(AfterCaptureAction::Complete);
        } else {
          scanToRight = !scanToRight;
          Serial.print(F("[SNAKE] New X direction="));
          Serial.println(scanToRight ? F("RIGHT") : F("LEFT"));
          scheduleCapture(AfterCaptureAction::MoveX);
        }
      }
      break;

    case ScanState::Complete:
      // Hold safe state.
      break;

    case ScanState::Fault:
      setMotorEnabled(false);
      if (cameraHigh) {
        digitalWrite(RobotConfig::CAMERA_TRIGGER_PIN, LOW);
        cameraHigh = false;
      }
      break;

    default:
      changeState(ScanState::Fault);
      break;
  }
}

}  // namespace

void begin() {
  Serial.begin(115200);
  const uint32_t serialStartMs = millis();
  while (!Serial && (millis() - serialStartMs) < RobotConfig::SERIAL_STARTUP_WAIT_MS) {
  }

  pinMode(RobotConfig::X_ENABLE_PIN, OUTPUT);
  pinMode(RobotConfig::Y_ENABLE_PIN, OUTPUT);

  pinMode(RobotConfig::X_LIMIT_LEFT_PIN, INPUT_PULLUP);
  pinMode(RobotConfig::X_LIMIT_RIGHT_PIN, INPUT_PULLUP);

  pinMode(RobotConfig::ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(RobotConfig::ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(RobotConfig::ULTRASONIC_TRIG_PIN, LOW);

  pinMode(RobotConfig::CAMERA_TRIGGER_PIN, OUTPUT);
  digitalWrite(RobotConfig::CAMERA_TRIGGER_PIN, LOW);

  xStepper.setMaxSpeed(RobotConfig::X_MAX_SPEED_STEPS_PER_SEC);
  xStepper.setAcceleration(RobotConfig::X_ACCEL_STEPS_PER_SEC2);
  yStepper.setMaxSpeed(RobotConfig::Y_MAX_SPEED_STEPS_PER_SEC);
  yStepper.setAcceleration(RobotConfig::Y_ACCEL_STEPS_PER_SEC2);
  xStepper.setMinPulseWidth(20);
  yStepper.setMinPulseWidth(20);

  xStepper.enableOutputs();
  yStepper.enableOutputs();

  xStepper.setCurrentPosition(0);
  yStepper.setCurrentPosition(0);

  setMotorEnabled(true);

  Serial.println(F("============================================"));
  Serial.println(F("Solar Inspection Robot - FSM Start"));
  Serial.println(F("Path: Boustrophedon / Snake"));
  Serial.print(F("STEP_DISTANCE_X="));
  Serial.println(RobotConfig::STEP_DISTANCE_X);
  Serial.print(F("STEP_DISTANCE_Y="));
  Serial.println(RobotConfig::STEP_DISTANCE_Y);
  Serial.print(F("CAMERA_EXPOSURE_MS="));
  Serial.println(RobotConfig::CAMERA_EXPOSURE_MS);
  Serial.print(F("PANEL_GAP_THRESHOLD_CM="));
  Serial.println(RobotConfig::PANEL_GAP_THRESHOLD_CM, 1);
  Serial.print(F("LIMIT_LEFT_PIN="));
  Serial.print(RobotConfig::X_LIMIT_LEFT_PIN);
  Serial.print(F(" pressed="));
  Serial.println(isLeftLimitPressed() ? F("YES") : F("NO"));
  Serial.print(F("LIMIT_RIGHT_PIN="));
  Serial.print(RobotConfig::X_LIMIT_RIGHT_PIN);
  Serial.print(F(" pressed="));
  Serial.println(isRightLimitPressed() ? F("YES") : F("NO"));
  Serial.println(F("============================================"));

  currentState = ScanState::Idle;
  panelCompleteAnnounced = false;
  panelGapDetected = false;
  panelGapConsecutiveReadings = 0;
  scanToRight = true;
}

void update() {
  if (currentState == ScanState::Complete) {
    return;
  }

  xStepper.run();
  yStepper.run();

  updateUltrasonic();

  if (panelGapDetected && currentState != ScanState::Complete) {
    stopAndComplete();
    return;
  }

  runStateMachine();
}

}  // namespace RobotController

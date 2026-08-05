#pragma once

#include <AccelStepper.h>
#include <Arduino.h>

#include "ChipSorterConfig.h"

class MotionController {
 public:
  struct AxisConfig {
    uint8_t limitPin;
    int8_t homeDirection;
  };

  explicit MotionController(Stream& serial);

  void begin();
  void update();
  void handleLine(const char* line);
  void runTestLoop();

 private:
  enum class Action : uint8_t {
    Idle,
    Homing,
    MoveTube,
    PushOut,
    PushReturn,
  };

  Stream& serial_;
  AccelStepper xStepper_;
  AccelStepper yStepper_;
  AccelStepper zStepper_;

  Action action_;
  uint8_t homingAxis_;
  uint8_t targetTube_;
  bool homed_;

  void dispatchCommand(char* mutableLine);
  void startHome();
  void startMoveTube(uint8_t tubeIndex);
  void startPush();
  void stopMotion();
  void sendOk(const char* command);
  void sendDone(const char* command);
  void sendError(const char* reason);
  void sendStatus() const;

  void updateHome();
  void updateMoveTube();
  void updatePush();

  bool isLimitTriggered(uint8_t pin) const;
  bool anyStepperBusy();
  void setMotionTarget(AccelStepper& stepper, long steps);
  AccelStepper& stepperForAxis(uint8_t axis);
  const AxisConfig& axisConfig(uint8_t axis) const;
};
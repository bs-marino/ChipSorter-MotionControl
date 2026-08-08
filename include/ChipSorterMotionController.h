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
  bool isBusy();

 private:
  enum class Action : uint8_t {
    Idle,
    Homing,
    Move,
    PushOut,
    PushReturn,
    PullOut,
    PullReturn,
  };

  enum class HomeState : uint8_t {
    Idle,
    XSeekForwardTrigger,
    XSeekForwardRelease,
    XSeekBackwardTrigger,
    XMoveToOffset,
    YSeekReverseTrigger,
    YBackoff,
    ZSeekReverseTrigger,
    ZBackoff,
  };

  Stream& serial_;
  AccelStepper xStepper_;
  AccelStepper yStepper_;
  AccelStepper zStepper_;

  Action action_;
  uint8_t homingAxis_;
  uint8_t currentTube_;
  uint8_t targetTube_;
  int8_t lastMoveDeltaTubes_;
  long lastMoveDeltaSteps_;
  uint16_t pushCyclesRemaining_;
  HomeState homeState_;
  uint8_t homePendingMask_;
  long xHomeForwardTriggerPos_;
  long xHomeReverseTriggerPos_;
  long xHomeTargetPos_;
  long yHomeTargetPos_;
  long zHomeTargetPos_;
  unsigned long lastLimitPollUs_;
  bool homed_;
  bool xLimitState_;
  bool yLimitState_;
  bool zLimitState_;

  void dispatchCommand(char* mutableLine);
  void startHome(uint8_t homeMask);
  void startMoveTube(uint8_t tubeIndex);
  void startPush(uint16_t repeatCount = 1);
  void startPull();
  void stopMotion();
  void sendOk(const char* command);
  void sendOkMove(uint8_t fromTube, uint8_t toTube, int8_t deltaTubes, long deltaSteps);
  void sendDone(const char* command);
  void sendError(const char* reason);
  void sendStatus() const;
  void sendLimitState(const char* axis, bool triggered);
  void beginNextHomeAxis();
  void completeHome();

  void updateHome();
  void updateMoveTube();
  void updatePush();
  void updateLimitSwitchStateEvents();

  bool isLimitTriggered(uint8_t pin, bool activeLow) const;
  bool anyStepperBusy();
  void runSteppersBurst();
  void pollLimitsIfDue();
  void normalizeTablePosition();
  AccelStepper& stepperForAxis(uint8_t axis);
  const AxisConfig& axisConfig(uint8_t axis) const;
};
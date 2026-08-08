#include "ChipSorterMotionController.h"

#include <string.h>

namespace {
const MotionController::AxisConfig kAxisConfigs[] = {
    {chip_sorter::kXLimitPin, chip_sorter::kXHomeDirection},
    {chip_sorter::kYLimitPin, chip_sorter::kYHomeDirection},
    {chip_sorter::kZLimitPin, chip_sorter::kZHomeDirection},
};

constexpr long kFullTableRotationSteps = chip_sorter::kTableStepsPerTube * chip_sorter::kTubeCount;
constexpr uint8_t kHomeMaskX = 0x01;
constexpr uint8_t kHomeMaskY = 0x02;
constexpr uint8_t kHomeMaskZ = 0x04;
}  // namespace

MotionController::MotionController(Stream& serial)
    : serial_(serial),
    xStepper_(AccelStepper::DRIVER, chip_sorter::kXStepPin, chip_sorter::kXDirPin),
    yStepper_(AccelStepper::DRIVER, chip_sorter::kYStepPin, chip_sorter::kYDirPin),
    zStepper_(AccelStepper::DRIVER, chip_sorter::kZStepPin, chip_sorter::kZDirPin),
      action_(Action::Idle),
      homingAxis_(0),
      currentTube_(0),
      targetTube_(0),
      lastMoveDeltaTubes_(0),
      lastMoveDeltaSteps_(0),
        pushCyclesRemaining_(0),
        homeState_(HomeState::Idle),
        homePendingMask_(0),
        xHomeForwardTriggerPos_(0),
        xHomeReverseTriggerPos_(0),
        xHomeTargetPos_(0),
        yHomeTargetPos_(0),
        zHomeTargetPos_(0),
      lastLimitPollUs_(0),
      homed_(true),
      xLimitState_(false),
      yLimitState_(false),
      zLimitState_(false) {}

void MotionController::begin() {
  pinMode(chip_sorter::kStepperEnablePin, OUTPUT);
  digitalWrite(chip_sorter::kStepperEnablePin, LOW);

  pinMode(chip_sorter::kXLimitPin, INPUT_PULLUP);
  pinMode(chip_sorter::kYLimitPin, INPUT_PULLUP);
  pinMode(chip_sorter::kZLimitPin, INPUT_PULLUP);

  xStepper_.setEnablePin(chip_sorter::kStepperEnablePin);
  yStepper_.setEnablePin(chip_sorter::kStepperEnablePin);
  zStepper_.setEnablePin(chip_sorter::kStepperEnablePin);

  xStepper_.setPinsInverted(false, false, true);
  yStepper_.setPinsInverted(false, false, true);
  zStepper_.setPinsInverted(false, false, true);

  xStepper_.setMaxSpeed(chip_sorter::kMaxMotionSpeed);
  yStepper_.setMaxSpeed(chip_sorter::kMaxMotionSpeed);
  zStepper_.setMaxSpeed(chip_sorter::kMaxMotionSpeed);

  xStepper_.setAcceleration(chip_sorter::kMotionAcceleration);
  yStepper_.setAcceleration(chip_sorter::kMotionAcceleration);
  zStepper_.setAcceleration(chip_sorter::kMotionAcceleration);

  xStepper_.setMinPulseWidth(5);
  yStepper_.setMinPulseWidth(5);
  zStepper_.setMinPulseWidth(5);

  xStepper_.enableOutputs();
  yStepper_.enableOutputs();
  zStepper_.enableOutputs();

  // Until homing is finalized, assume startup is tube 0.
  xStepper_.setCurrentPosition(0);
  currentTube_ = 0;
  targetTube_ = 0;
  homed_ = true;

  xLimitState_ = isLimitTriggered(chip_sorter::kXLimitPin, chip_sorter::kXLimitSwitchActiveLow);
  yLimitState_ = isLimitTriggered(chip_sorter::kYLimitPin, chip_sorter::kYLimitSwitchActiveLow);
  zLimitState_ = isLimitTriggered(chip_sorter::kZLimitPin, chip_sorter::kZLimitSwitchActiveLow);
}

void MotionController::update() {
  pollLimitsIfDue();

  switch (action_) {
    case Action::Idle:
      runSteppersBurst();
      break;
    case Action::Homing:
      updateHome();
      break;
    case Action::Move:
      updateMoveTube();
      break;
    case Action::PushOut:
    case Action::PushReturn:
    case Action::PullOut:
    case Action::PullReturn:
      updatePush();
      break;
  }
}

bool MotionController::isBusy() {
  return action_ != Action::Idle || anyStepperBusy();
}

void MotionController::handleLine(const char* line) {
  if (line == nullptr) {
    return;
  }

  char buffer[64];
  strncpy(buffer, line, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0';
  dispatchCommand(buffer);
}

void MotionController::dispatchCommand(char* mutableLine) {
  char* command = strtok(mutableLine, " \t");
  char* arg = strtok(nullptr, " \t");
  char* extra = strtok(nullptr, " \t");

  if (command == nullptr) {
    return;
  }

  if (extra != nullptr) {
    sendError("BAD_ARGS");
    return;
  }

  if (strcmp(command, "STATUS") == 0) {
    sendStatus();
    return;
  }

  if (strcmp(command, "STOP") == 0) {
    stopMotion();
    sendOk("STOP");
    sendDone("STOP");
    return;
  }

  if (action_ != Action::Idle) {
    sendError("BUSY");
    return;
  }

  if (strcmp(command, "HOME") == 0) {
    if (arg == nullptr || strcmp(arg, "X") == 0) {
      startHome(kHomeMaskX);
      sendOk("HOME X");
      return;
    }
    if (strcmp(arg, "ALL") == 0) {
      startHome(kHomeMaskX | kHomeMaskY | kHomeMaskZ);
      sendOk("HOME ALL");
      return;
    }
    if (strcmp(arg, "Y") == 0) {
      startHome(kHomeMaskY);
      sendOk("HOME Y");
      return;
    }
    if (strcmp(arg, "Z") == 0) {
      startHome(kHomeMaskZ);
      sendOk("HOME Z");
      return;
    }
    sendError("BAD_ARGS");
    return;
  }

  if (strcmp(command, "MOVE") == 0) {
    if (arg == nullptr) {
      sendError("BAD_ARGS");
      return;
    }
    char* endPtr = nullptr;
    long tube = strtol(arg, &endPtr, 10);
    if (endPtr == arg || *endPtr != '\0' || tube < 0 || tube >= chip_sorter::kTubeCount) {
      sendError("OUT_OF_RANGE");
      return;
    }
    const uint8_t fromTube = currentTube_;
    startMoveTube(static_cast<uint8_t>(tube));
    sendOkMove(fromTube, targetTube_, lastMoveDeltaTubes_, lastMoveDeltaSteps_);
    return;
  }

  if (strcmp(command, "PUSH") == 0) {
    uint16_t repeatCount = 1;
    if (arg != nullptr) {
      char* endPtr = nullptr;
      long parsed = strtol(arg, &endPtr, 10);
      if (endPtr == arg || *endPtr != '\0' || parsed <= 0 || parsed > 1000) {
        sendError("BAD_ARGS");
        return;
      }
      repeatCount = static_cast<uint16_t>(parsed);
    }
    startPush(repeatCount);
    sendOk("PUSH");
    return;
  }

  if (strcmp(command, "PULL") == 0) {
    if (arg != nullptr) {
      sendError("BAD_ARGS");
      return;
    }
    startPull();
    sendOk("PULL");
    return;
  }

  sendError("UNKNOWN");
}

void MotionController::startHome(uint8_t homeMask) {
  homed_ = false;
  action_ = Action::Homing;
  homePendingMask_ = homeMask;
  homeState_ = HomeState::Idle;
  xHomeForwardTriggerPos_ = 0;
  xHomeReverseTriggerPos_ = 0;
  xHomeTargetPos_ = xStepper_.currentPosition();
  yHomeTargetPos_ = yStepper_.currentPosition();
  zHomeTargetPos_ = zStepper_.currentPosition();
  beginNextHomeAxis();
}

void MotionController::startMoveTube(uint8_t tubeIndex) {
  const int tubeCount = static_cast<int>(chip_sorter::kTubeCount);
  const int current = static_cast<int>(currentTube_);
  const int target = static_cast<int>(tubeIndex);

  const int forwardTubes = (target - current + tubeCount) % tubeCount;
  const int backwardTubes = (tubeCount - forwardTubes) % tubeCount;

  int deltaTubes = 0;
  if (forwardTubes <= backwardTubes) {
    // Tie goes forward by requirement.
    deltaTubes = forwardTubes;
  } else {
    deltaTubes = -backwardTubes;
  }

  const long deltaSteps = static_cast<long>(deltaTubes) * chip_sorter::kTableStepsPerTube;

  targetTube_ = tubeIndex;
  lastMoveDeltaTubes_ = static_cast<int8_t>(deltaTubes);
  lastMoveDeltaSteps_ = deltaSteps;
  xStepper_.move(deltaSteps);
  yStepper_.moveTo(yStepper_.currentPosition());
  zStepper_.moveTo(zStepper_.currentPosition());
  action_ = Action::Move;
}

void MotionController::startPush(uint16_t repeatCount) {
  pushCyclesRemaining_ = repeatCount;
  yStepper_.move(chip_sorter::kPusherStrokeSteps);
  action_ = Action::PushOut;
}

void MotionController::startPull() {
  zStepper_.move(chip_sorter::kPullerStrokeSteps);
  action_ = Action::PullOut;
}

void MotionController::stopMotion() {
  xStepper_.stop();
  yStepper_.stop();
  zStepper_.stop();

  xStepper_.disableOutputs();
  yStepper_.disableOutputs();
  zStepper_.disableOutputs();

  action_ = Action::Idle;
  homed_ = false;
}

void MotionController::sendOk(const char* command) {
  serial_.print(F("OK "));
  serial_.println(command);
}

void MotionController::sendOkMove(uint8_t fromTube, uint8_t toTube, int8_t deltaTubes, long deltaSteps) {
  serial_.print(F("OK MOVE from="));
  serial_.print(fromTube);
  serial_.print(F(" to="));
  serial_.print(toTube);
  serial_.print(F(" dir="));
  if (deltaTubes >= 0) {
    serial_.print(F("FWD"));
  } else {
    serial_.print(F("REV"));
  }
  serial_.print(F(" tubes="));
  serial_.print(deltaTubes >= 0 ? deltaTubes : -deltaTubes);
  serial_.print(F(" steps="));
  serial_.println(deltaSteps);
}

void MotionController::sendDone(const char* command) {
  serial_.print(F("DONE "));
  serial_.println(command);
}

void MotionController::sendError(const char* reason) {
  serial_.print(F("ERR "));
  serial_.println(reason);
}

void MotionController::sendStatus() const {
  serial_.print(F("STATUS "));
  serial_.print(homed_ ? 1 : 0);
  serial_.print(' ');
  serial_.print(currentTube_);
  serial_.print(' ');
  serial_.println(action_ == Action::Idle ? 0 : 1);
}

void MotionController::sendLimitState(const char* axis, bool triggered) {
  serial_.print(F("LIMIT "));
  serial_.print(axis);
  serial_.print(' ');
  serial_.println(triggered ? 1 : 0);
}

void MotionController::beginNextHomeAxis() {
  if ((homePendingMask_ & kHomeMaskX) != 0) {
    homePendingMask_ &= static_cast<uint8_t>(~kHomeMaskX);
    homeState_ = HomeState::XSeekForwardTrigger;
    return;
  }

  if ((homePendingMask_ & kHomeMaskY) != 0) {
    homePendingMask_ &= static_cast<uint8_t>(~kHomeMaskY);
    homeState_ = HomeState::YSeekReverseTrigger;
    return;
  }

  if ((homePendingMask_ & kHomeMaskZ) != 0) {
    homePendingMask_ &= static_cast<uint8_t>(~kHomeMaskZ);
    homeState_ = HomeState::ZSeekReverseTrigger;
    return;
  }

  completeHome();
}

void MotionController::completeHome() {
  currentTube_ = 0;
  targetTube_ = 0;
  homed_ = true;
  action_ = Action::Idle;
  homeState_ = HomeState::Idle;
  sendDone("HOME");
}

void MotionController::updateHome() {
  switch (homeState_) {
    case HomeState::Idle:
      action_ = Action::Idle;
      return;

    case HomeState::XSeekForwardTrigger:
      xStepper_.setSpeed(static_cast<float>(chip_sorter::kHomeSeekSpeed));
      xStepper_.runSpeed();
      if (isLimitTriggered(chip_sorter::kXLimitPin, chip_sorter::kXLimitSwitchActiveLow)) {
        xHomeForwardTriggerPos_ = xStepper_.currentPosition();
        homeState_ = HomeState::XSeekForwardRelease;
      }
      return;

    case HomeState::XSeekForwardRelease:
      xStepper_.setSpeed(static_cast<float>(chip_sorter::kHomeSeekSpeed));
      xStepper_.runSpeed();
      if (!isLimitTriggered(chip_sorter::kXLimitPin, chip_sorter::kXLimitSwitchActiveLow)) {
        homeState_ = HomeState::XSeekBackwardTrigger;
      }
      return;

    case HomeState::XSeekBackwardTrigger:
      xStepper_.setSpeed(static_cast<float>(-chip_sorter::kHomeSeekSpeed));
      xStepper_.runSpeed();
      if (isLimitTriggered(chip_sorter::kXLimitPin, chip_sorter::kXLimitSwitchActiveLow)) {
        xHomeReverseTriggerPos_ = xStepper_.currentPosition();
        const long midpoint = (xHomeForwardTriggerPos_ + xHomeReverseTriggerPos_) / 2;
        xHomeTargetPos_ = midpoint + chip_sorter::kXHomeOffsetSteps;
        xStepper_.moveTo(xHomeTargetPos_);
        homeState_ = HomeState::XMoveToOffset;
      }
      return;

    case HomeState::XMoveToOffset:
      xStepper_.run();
      if (xStepper_.distanceToGo() == 0) {
        xStepper_.setCurrentPosition(0);
        beginNextHomeAxis();
      }
      return;

    case HomeState::YSeekReverseTrigger:
      yStepper_.setSpeed(static_cast<float>(chip_sorter::kHomeSeekSpeed * chip_sorter::kYHomeDirection));
      yStepper_.runSpeed();
      if (isLimitTriggered(chip_sorter::kYLimitPin, chip_sorter::kYLimitSwitchActiveLow)) {
        yStepper_.setCurrentPosition(0);
        yHomeTargetPos_ = -chip_sorter::kYHomeBackoffSteps * chip_sorter::kYHomeDirection;
        yStepper_.moveTo(yHomeTargetPos_);
        homeState_ = HomeState::YBackoff;
      }
      return;

    case HomeState::YBackoff:
      yStepper_.run();
      if (yStepper_.distanceToGo() == 0) {
        yStepper_.setCurrentPosition(0);
        beginNextHomeAxis();
      }
      return;

    case HomeState::ZSeekReverseTrigger:
      zStepper_.setSpeed(static_cast<float>(chip_sorter::kHomeSeekSpeed * chip_sorter::kZHomeDirection));
      zStepper_.runSpeed();
      if (isLimitTriggered(chip_sorter::kZLimitPin, chip_sorter::kZLimitSwitchActiveLow)) {
        zStepper_.setCurrentPosition(0);
        zHomeTargetPos_ = -chip_sorter::kZHomeBackoffSteps * chip_sorter::kZHomeDirection;
        zStepper_.moveTo(zHomeTargetPos_);
        homeState_ = HomeState::ZBackoff;
      }
      return;

    case HomeState::ZBackoff:
      zStepper_.run();
      if (zStepper_.distanceToGo() == 0) {
        zStepper_.setCurrentPosition(0);
        beginNextHomeAxis();
      }
      return;
  }
}

void MotionController::updateMoveTube() {
  runSteppersBurst();

  if (!anyStepperBusy()) {
    currentTube_ = targetTube_;
    normalizeTablePosition();
    action_ = Action::Idle;
    sendDone("MOVE");
  }
}

void MotionController::updatePush() {
  runSteppersBurst();

  if (action_ == Action::PushOut && yStepper_.distanceToGo() == 0) {
    yStepper_.move(-chip_sorter::kPusherStrokeSteps);
    action_ = Action::PushReturn;
    return;
  }

  if (action_ == Action::PushReturn && yStepper_.distanceToGo() == 0) {
    if (pushCyclesRemaining_ > 1) {
      pushCyclesRemaining_--;
      yStepper_.move(chip_sorter::kPusherStrokeSteps);
      action_ = Action::PushOut;
    } else {
      pushCyclesRemaining_ = 0;
      action_ = Action::Idle;
      sendDone("PUSH");
    }
    return;
  }

  if (action_ == Action::PullOut && zStepper_.distanceToGo() == 0) {
    zStepper_.move(-chip_sorter::kPullerStrokeSteps);
    action_ = Action::PullReturn;
    return;
  }

  if (action_ == Action::PullReturn && zStepper_.distanceToGo() == 0) {
    action_ = Action::Idle;
    sendDone("PULL");
  }
}

bool MotionController::isLimitTriggered(uint8_t pin, bool activeLow) const {
  return digitalRead(pin) == (activeLow ? LOW : HIGH);
}

bool MotionController::anyStepperBusy() {
  return xStepper_.distanceToGo() != 0 || yStepper_.distanceToGo() != 0 || zStepper_.distanceToGo() != 0;
}

void MotionController::runSteppersBurst() {
  for (uint8_t i = 0; i < chip_sorter::kStepperRunBurstCount; ++i) {
    const bool stepped = xStepper_.run() || yStepper_.run() || zStepper_.run();
    if (!stepped) {
      break;
    }
  }
}

void MotionController::pollLimitsIfDue() {
  const unsigned long nowUs = micros();
  if (lastLimitPollUs_ == 0 || (nowUs - lastLimitPollUs_) >= chip_sorter::kLimitPollIntervalUs) {
    lastLimitPollUs_ = nowUs;
    updateLimitSwitchStateEvents();
  }
}

void MotionController::normalizeTablePosition() {
  if (kFullTableRotationSteps <= 0) {
    return;
  }

  long tablePos = xStepper_.currentPosition() % kFullTableRotationSteps;
  if (tablePos < 0) {
    tablePos += kFullTableRotationSteps;
  }
  xStepper_.setCurrentPosition(tablePos);
}

void MotionController::updateLimitSwitchStateEvents() {
  const bool yNow = isLimitTriggered(chip_sorter::kYLimitPin, chip_sorter::kYLimitSwitchActiveLow);
  const bool zNow = isLimitTriggered(chip_sorter::kZLimitPin, chip_sorter::kZLimitSwitchActiveLow);

  xLimitState_ = isLimitTriggered(chip_sorter::kXLimitPin, chip_sorter::kXLimitSwitchActiveLow);

  if (yNow != yLimitState_) {
    yLimitState_ = yNow;
    if (action_ == Action::Homing) {
      sendLimitState("Y", yNow);
    } else if (yNow) {
      sendError("LIMIT Y");
    }
  }

  if (zNow != zLimitState_) {
    zLimitState_ = zNow;
    if (action_ == Action::Homing) {
      sendLimitState("Z", zNow);
    } else if (zNow) {
      sendError("LIMIT Z");
    }
  }
}

AccelStepper& MotionController::stepperForAxis(uint8_t axis) {
  switch (axis) {
    case 0:
      return xStepper_;
    case 1:
      return yStepper_;
    default:
      return zStepper_;
  }
}

const MotionController::AxisConfig& MotionController::axisConfig(uint8_t axis) const {
  return kAxisConfigs[axis];
}
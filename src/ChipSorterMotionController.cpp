#include "ChipSorterMotionController.h"

#include <string.h>

namespace {
const MotionController::AxisConfig kAxisConfigs[] = {
    {chip_sorter::kXLimitPin, chip_sorter::kXHomeDirection},
    {chip_sorter::kYLimitPin, chip_sorter::kYHomeDirection},
    {chip_sorter::kZLimitPin, chip_sorter::kZHomeDirection},
};
}  // namespace

MotionController::MotionController(Stream& serial)
    : serial_(serial),
      xStepper_(AccelStepper::DRIVER, chip_sorter::kXStepPin, chip_sorter::kXDirPin),
      yStepper_(AccelStepper::DRIVER, chip_sorter::kYStepPin, chip_sorter::kYDirPin),
      zStepper_(AccelStepper::DRIVER, chip_sorter::kZStepPin, chip_sorter::kZDirPin),
      action_(Action::Idle),
      homingAxis_(0),
      targetTube_(0),
      homed_(false) {}

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

  xStepper_.setMinPulseWidth(3);
  yStepper_.setMinPulseWidth(3);
  zStepper_.setMinPulseWidth(3);

  xStepper_.enableOutputs();
  yStepper_.enableOutputs();
  zStepper_.enableOutputs();
}

void MotionController::update() {
  switch (action_) {
    case Action::Idle:
      xStepper_.run();
      yStepper_.run();
      zStepper_.run();
      break;
    case Action::Homing:
      updateHome();
      break;
    case Action::MoveTube:
      updateMoveTube();
      break;
    case Action::PushOut:
    case Action::PushReturn:
      updatePush();
      break;
  }
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

void MotionController::runTestLoop() {
  enum class TestState : uint8_t {
    XForward,
    YForward,
    YBackward,
    XBackward,
  };

  static TestState state = TestState::XForward;
  static bool motionIssued = false;

  if (!motionIssued) {
    switch (state) {
      case TestState::XForward:
        setMotionTarget(xStepper_, chip_sorter::kTestRotationSteps);
        break;
      case TestState::YForward:
        setMotionTarget(yStepper_, chip_sorter::kTestRotationSteps);
        break;
      case TestState::YBackward:
        setMotionTarget(yStepper_, -chip_sorter::kTestRotationSteps);
        break;
      case TestState::XBackward:
        setMotionTarget(xStepper_, -chip_sorter::kTestRotationSteps);
        break;
    }
    motionIssued = true;
  }

  xStepper_.run();
  yStepper_.run();
  zStepper_.run();

  if (anyStepperBusy()) {
    return;
  }

  motionIssued = false;
  switch (state) {
    case TestState::XForward:
      state = TestState::YForward;
      break;
    case TestState::YForward:
      state = TestState::YBackward;
      break;
    case TestState::YBackward:
      state = TestState::XBackward;
      break;
    case TestState::XBackward:
      state = TestState::XForward;
      break;
  }
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
    if (arg != nullptr) {
      sendError("BAD_ARGS");
      return;
    }
    startHome();
    sendOk("HOME");
    return;
  }

  if (strcmp(command, "MOVE_TUBE") == 0) {
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
    startMoveTube(static_cast<uint8_t>(tube));
    sendOk("MOVE_TUBE");
    return;
  }

  if (strcmp(command, "PUSH") == 0) {
    if (arg != nullptr) {
      sendError("BAD_ARGS");
      return;
    }
    startPush();
    sendOk("PUSH");
    return;
  }

  sendError("UNKNOWN");
}

void MotionController::startHome() {
  homed_ = false;
  action_ = Action::Homing;
  homingAxis_ = 0;
}

void MotionController::startMoveTube(uint8_t tubeIndex) {
  targetTube_ = tubeIndex;
  xStepper_.moveTo(static_cast<long>(tubeIndex) * chip_sorter::kTableStepsPerTube);
  yStepper_.moveTo(yStepper_.currentPosition());
  zStepper_.moveTo(zStepper_.currentPosition());
  action_ = Action::MoveTube;
}

void MotionController::startPush() {
  zStepper_.move(chip_sorter::kPusherStrokeSteps);
  action_ = Action::PushOut;
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
  serial_.print(targetTube_);
  serial_.print(' ');
  serial_.println(action_ == Action::Idle ? 0 : 1);
}

void MotionController::updateHome() {
  if (homingAxis_ >= 3) {
    action_ = Action::Idle;
    homed_ = true;
    targetTube_ = 0;
    xStepper_.setCurrentPosition(0);
    yStepper_.setCurrentPosition(0);
    zStepper_.setCurrentPosition(0);
    sendDone("HOME");
    return;
  }

  const AxisConfig& axis = axisConfig(homingAxis_);
  AccelStepper& stepper = stepperForAxis(homingAxis_);

  if (stepper.distanceToGo() == 0) {
    if (!isLimitTriggered(axis.limitPin)) {
      stepper.setSpeed(static_cast<float>(chip_sorter::kHomeSeekSpeed * axis.homeDirection));
      stepper.runSpeed();
      return;
    }

    stepper.stop();
    stepper.setCurrentPosition(0);
    stepper.move(-chip_sorter::kHomeBackoffSteps * axis.homeDirection);
  }

  stepper.run();
  if (stepper.distanceToGo() == 0) {
    stepper.setCurrentPosition(0);
    homingAxis_++;
  }
}

void MotionController::updateMoveTube() {
  xStepper_.run();
  yStepper_.run();
  zStepper_.run();

  if (!anyStepperBusy()) {
    action_ = Action::Idle;
    sendDone("MOVE_TUBE");
  }
}

void MotionController::updatePush() {
  xStepper_.run();
  yStepper_.run();
  zStepper_.run();

  if (action_ == Action::PushOut && zStepper_.distanceToGo() == 0) {
    zStepper_.move(-chip_sorter::kPusherStrokeSteps);
    action_ = Action::PushReturn;
    return;
  }

  if (action_ == Action::PushReturn && zStepper_.distanceToGo() == 0) {
    action_ = Action::Idle;
    sendDone("PUSH");
  }
}

bool MotionController::isLimitTriggered(uint8_t pin) const {
  return digitalRead(pin) == (chip_sorter::kLimitSwitchActiveLow ? LOW : HIGH);
}

bool MotionController::anyStepperBusy() {
  return xStepper_.distanceToGo() != 0 || yStepper_.distanceToGo() != 0 || zStepper_.distanceToGo() != 0;
}

void MotionController::setMotionTarget(AccelStepper& stepper, long steps) {
  stepper.move(steps);
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
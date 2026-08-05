#pragma once

#include <Arduino.h>

namespace chip_sorter {

constexpr uint32_t kSerialBaud = CHIP_SORTER_BAUD;

constexpr uint8_t kStepperEnablePin = 8;

constexpr uint8_t kXStepPin = 2;
constexpr uint8_t kXDirPin = 5;
constexpr uint8_t kXLimitPin = 9;

constexpr uint8_t kYStepPin = 3;
constexpr uint8_t kYDirPin = 6;
constexpr uint8_t kYLimitPin = 10;

constexpr uint8_t kZStepPin = 4;
constexpr uint8_t kZDirPin = 7;
constexpr uint8_t kZLimitPin = 11;

constexpr uint8_t kTubeCount = 10;
constexpr long kTableStepsPerTube = 800;
constexpr long kTestRotationSteps = kTableStepsPerTube;

constexpr long kHomeSeekSpeed = 300;
constexpr long kHomeBackoffSteps = 80;

constexpr long kMaxMotionSpeed = 1200;
constexpr float kMotionAcceleration = 900.0f;

constexpr long kPusherStrokeSteps = 250;

constexpr bool kLimitSwitchActiveLow = true;
constexpr int8_t kXHomeDirection = -1;
constexpr int8_t kYHomeDirection = -1;
constexpr int8_t kZHomeDirection = -1;

}  // namespace chip_sorter
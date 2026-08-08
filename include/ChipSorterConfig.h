#pragma once

#include <Arduino.h>

#ifndef CHIP_SORTER_BAUD
#define CHIP_SORTER_BAUD 115200
#endif

#ifndef CHIP_SORTER_LINK_SERIAL_MODE
#define CHIP_SORTER_LINK_SERIAL_MODE 0
#endif

#ifndef CHIP_SORTER_LINK_BAUD
#if CHIP_SORTER_LINK_SERIAL_MODE == 1
#define CHIP_SORTER_LINK_BAUD 19200
#else
#define CHIP_SORTER_LINK_BAUD CHIP_SORTER_BAUD
#endif
#endif

namespace chip_sorter {

constexpr uint8_t stepperStepsPerRevolution = 200;
// Torque-first baseline for reliable indexing.
constexpr uint8_t stepperMicrosteps = 4;
constexpr uint8_t pinionTeeth = 10;
constexpr float pullerToothSpacing = 3.09;
constexpr float pusherToothSpacing = 3.09;
constexpr uint8_t gearTeeth = 210;
constexpr uint8_t pusherTeeth = 10;
constexpr uint8_t pullerTeeth = 10;
constexpr uint16_t pusherTravel = 44; //44
constexpr uint16_t pullerTravel = 20; //20

constexpr uint32_t kUsbSerialBaud = CHIP_SORTER_BAUD;
constexpr uint32_t kLinkSerialBaud = CHIP_SORTER_LINK_BAUD;

constexpr uint8_t kLinkSoftwareRxPin = 12;
constexpr uint8_t kLinkSoftwareTxPin = 13;

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
constexpr long kTableStepsPerTube = (static_cast<long>(stepperStepsPerRevolution) *
									 static_cast<long>(stepperMicrosteps) *
									 static_cast<long>(gearTeeth)) /
									static_cast<long>(pinionTeeth) / static_cast<long>(kTubeCount);
constexpr long kTestRotationSteps = kTableStepsPerTube;

constexpr long kHomeSeekSpeed = 400;
constexpr long kHomeBackoffSteps = 80;
constexpr long kYHomeBackoffSteps = 30;  // actual tbd
constexpr long kZHomeBackoffSteps = 45;
// Home offset from magnetic midpoint in motor steps.
// Positive values move in forward direction from midpoint before zeroing.
constexpr long kXHomeOffsetSteps = 0;

constexpr long kMaxMotionSpeed = 3000;
constexpr float kMotionAcceleration = 5000.0f;

// Conservative scheduler defaults for smooth motion on Uno.
// If needed, raise kStepperRunBurstCount gradually (2, 3, ...) while testing.
constexpr uint8_t kStepperRunBurstCount = 1;
// Poll limits at a moderate rate to reduce runtime overhead.
constexpr unsigned long kLimitPollIntervalUs = 5000;

constexpr long kPusherStrokeSteps = ((static_cast<long>(stepperStepsPerRevolution) * static_cast<long>(stepperMicrosteps) * (static_cast<long>(pusherTravel)) / static_cast<long>(pinionTeeth)) / static_cast<long>(pusherToothSpacing));
constexpr long kPullerStrokeSteps = ((static_cast<long>(stepperStepsPerRevolution) * static_cast<long>(stepperMicrosteps) * (static_cast<long>(pullerTravel)) / static_cast<long>(pinionTeeth)) / static_cast<long>(pullerToothSpacing));

constexpr bool kXLimitSwitchActiveLow = true;
constexpr bool kYLimitSwitchActiveLow = false;
constexpr bool kZLimitSwitchActiveLow = false;
constexpr int8_t kXHomeDirection = -1;
constexpr int8_t kYHomeDirection = -1;
constexpr int8_t kZHomeDirection = -1;

}  // namespace chip_sorter
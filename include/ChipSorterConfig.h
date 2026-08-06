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
constexpr uint8_t stepperMicrosteps = 1;
constexpr uint8_t pinionTeeth = 10;
constexpr uint8_t gearTeeth = 210;
constexpr uint8_t pusherTeeth = 10;
constexpr uint8_t pullerTeeth = 10;
constexpr uint16_t pusherTravel = 44;
constexpr uint16_t pullerTravel = 20;

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

constexpr long kHomeSeekSpeed = 300;
constexpr long kHomeBackoffSteps = 80;

constexpr long kMaxMotionSpeed = 1200;
constexpr float kMotionAcceleration = 2000.0f;

constexpr long kPusherStrokeSteps = (static_cast<long>(pusherTravel) * static_cast<long>(stepperStepsPerRevolution) * static_cast<long>(stepperMicrosteps) / pusherTeeth);  
constexpr long kPullerStrokeSteps = (static_cast<long>(pullerTravel) * static_cast<long>(stepperStepsPerRevolution) * static_cast<long>(stepperMicrosteps) / pullerTeeth);

constexpr bool kLimitSwitchActiveLow = true;
constexpr int8_t kXHomeDirection = -1;
constexpr int8_t kYHomeDirection = -1;
constexpr int8_t kZHomeDirection = -1;

}  // namespace chip_sorter
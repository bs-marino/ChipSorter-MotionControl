#include <Arduino.h>

#include "ChipSorterConfig.h"
#include "ChipSorterMotionController.h"

#if CHIP_SORTER_LINK_SERIAL_MODE == 1
#include <SoftwareSerial.h>
#endif

namespace {
// The controller talks to the main processor over the configured link serial.
#if CHIP_SORTER_LINK_SERIAL_MODE == 1
SoftwareSerial linkSerial(chip_sorter::kLinkSoftwareRxPin, chip_sorter::kLinkSoftwareTxPin);
Stream& linkStream = linkSerial;
#else
Stream& linkStream = Serial;
#endif

MotionController controller(linkStream);
#if !CHIP_SORTER_TEST_LOOP
char serialLine[64];
size_t serialLineLength = 0;
#endif
}  // namespace

void setup() {
#if CHIP_SORTER_LINK_SERIAL_MODE == 1
  Serial.begin(chip_sorter::kUsbSerialBaud);
  linkSerial.begin(chip_sorter::kLinkSerialBaud);
#else
  Serial.begin(chip_sorter::kLinkSerialBaud);
#endif
  controller.begin();
}

void loop() {
#if CHIP_SORTER_TEST_LOOP
  controller.runTestLoop();
#else
  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());
    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      serialLine[serialLineLength] = '\0';
      controller.handleLine(serialLine);
      serialLineLength = 0;
      continue;
    }

    if (serialLineLength < sizeof(serialLine) - 1) {
      serialLine[serialLineLength++] = incoming;
    }
  }

  controller.update();
#endif
}
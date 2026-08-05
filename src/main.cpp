#include <Arduino.h>

#include "ChipSorterConfig.h"
#include "ChipSorterMotionController.h"

namespace {
MotionController controller(Serial);
#if !CHIP_SORTER_TEST_LOOP
char serialLine[64];
size_t serialLineLength = 0;
#endif
}  // namespace

void setup() {
  Serial.begin(chip_sorter::kSerialBaud);
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
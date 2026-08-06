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

#if CHIP_SORTER_TEST_LOOP
const char* kSimulatedCommands[] = {
  "MOVE_TUBE 3",
  "PUSH",
  "MOVE_TUBE 7",
  "PUSH",
  "MOVE_TUBE 8",
  "PUSH",
  "MOVE_TUBE 4",
  "PUSH",
};
constexpr size_t kSimulatedCommandCount = sizeof(kSimulatedCommands) / sizeof(kSimulatedCommands[0]);
#endif
}  // namespace

void setup() {
#if CHIP_SORTER_LINK_SERIAL_MODE == 1
  Serial.begin(chip_sorter::kUsbSerialBaud);
  linkSerial.begin(chip_sorter::kLinkSerialBaud);
  delay(2000);  // Wait for the serial port to be ready
#else
  Serial.begin(chip_sorter::kLinkSerialBaud);
  delay(2000);  // Wait for the serial port to be ready
#endif
  controller.begin();
  Serial.println(F("Chip Sorter Motion Controller"));   
}

void loop() {
#if CHIP_SORTER_TEST_LOOP
  static size_t commandIndex = 0;
  static unsigned long nextIssueAtMs = 0;

  controller.update();

  if (controller.isBusy()) {
    return;
  }

  const unsigned long nowMs = millis();
  if (nowMs < nextIssueAtMs) {
    return;
  }

  const char* command = kSimulatedCommands[commandIndex];
  Serial.print(F("SIM> "));
  Serial.println(command);
  controller.handleLine(command);

  commandIndex = (commandIndex + 1) % kSimulatedCommandCount;
  nextIssueAtMs = nowMs + 250;
#else
  while (linkStream.available() > 0) {
    const char incoming = static_cast<char>(linkStream.read());

#if CHIP_SORTER_LINK_SERIAL_MODE == 0
    if (incoming == '\b' || static_cast<uint8_t>(incoming) == 127) {
      if (serialLineLength > 0) {
        serialLineLength--;
        Serial.print(F("\b \b"));
      }
      continue;
    }
#endif

    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
#if CHIP_SORTER_LINK_SERIAL_MODE == 0
      Serial.println();
#endif
      serialLine[serialLineLength] = '\0';
      controller.handleLine(serialLine);
      serialLineLength = 0;
      continue;
    }

    if (serialLineLength < sizeof(serialLine) - 1) {
      serialLine[serialLineLength++] = incoming;
#if CHIP_SORTER_LINK_SERIAL_MODE == 0
      Serial.write(incoming);
#endif
    }
  }

  controller.update();
#endif
}
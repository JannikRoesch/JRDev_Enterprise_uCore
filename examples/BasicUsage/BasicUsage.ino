#include <Arduino.h>
#include "Core/ESP32Core.h"
#include "Utilities/Logger.h"
#include "uConfig.h"

void setup() {
    Serial.begin(115200);
    delay(500);

    JRDev::Logger::info("System Start");
    JRDev::ESP32Core::printChipInfo();
    JRDev::ESP32Core::setCpuFrequencyMhz(DEFAULT_CPU_FREQ);
    JRDev::ESP32Core::enableWatchdog(5000);
}

void loop() {
    delay(1000);
}

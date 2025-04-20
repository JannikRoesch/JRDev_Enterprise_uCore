#include <Arduino.h>
#include "System/PowerManager.h"
#include "System/FilesystemManager.h"
#include "Utilities/Logger.h"

void setup() {
    Serial.begin(115200);
    delay(500);

    JRDev::Logger::info("System starting...");
    
    if (JRDev::FilesystemManager::begin()) {
        JRDev::FilesystemManager::writeFile("/boot.txt", "System rebooted");
    }

    JRDev::PowerManager::enterDeepSleep(10e6);  // 10 Sekunden
}

void loop() {
    // nie erreicht
}

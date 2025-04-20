#pragma once

#include <Arduino.h>

namespace JRDev {

class BLEManager {
public:
    static void begin(const String& deviceName = "ESP32-BLE");
    static void sendAdvertisement(const String& payload);
};

}

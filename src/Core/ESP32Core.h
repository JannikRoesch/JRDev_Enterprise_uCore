#pragma once

#include <Arduino.h>

namespace JRDev {

class ESP32Core {
public:
    static void printChipInfo();
    static void setCpuFrequencyMhz(uint8_t mhz);
    static void enableWatchdog(uint32_t timeout_ms);
};

}

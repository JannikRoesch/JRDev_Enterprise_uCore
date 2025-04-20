#pragma once

#include <HardwareSerial.h>

namespace JRDev {

class UARTManager {
public:
    static void begin(HardwareSerial& serial = Serial1, uint32_t baud = 115200, int tx = 17, int rx = 16);
    static void write(const String& data);
    static String readLine();
};

}

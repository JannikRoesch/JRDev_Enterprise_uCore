#pragma once

#include <Wire.h>

namespace JRDev {

class I2CManager {
public:
    static void begin(uint8_t sda = 21, uint8_t scl = 22, uint32_t freq = 400000);
    static bool deviceExists(uint8_t address);
};

}

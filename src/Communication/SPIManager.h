#pragma once

#include <SPI.h>

namespace JRDev {

class SPIManager {
public:
    static void begin(uint8_t sck = 18, uint8_t miso = 19, uint8_t mosi = 23, uint8_t cs = 5);
    static void transfer(uint8_t data);
    static uint8_t receive();
};

}

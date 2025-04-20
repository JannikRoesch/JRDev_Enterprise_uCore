#include "I2CManager.h"
#include "../Utilities/Logger.h"

using namespace JRDev;

void I2CManager::begin(uint8_t sda, uint8_t scl, uint32_t freq) {
    Wire.begin(sda, scl, freq);
    Logger::info("I2C initialized (SDA: %d, SCL: %d, Freq: %lu Hz)", sda, scl, freq);
}

bool I2CManager::deviceExists(uint8_t address) {
    Wire.beginTransmission(address);
    return (Wire.endTransmission() == 0);
}

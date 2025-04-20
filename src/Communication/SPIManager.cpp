#include "SPIManager.h"
#include "../Utilities/Logger.h"

using namespace JRDev;

static SPIClass* _spi = nullptr;
static uint8_t _csPin = 5;

void SPIManager::begin(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t cs) {
    _spi = new SPIClass(VSPI);
    _csPin = cs;

    _spi->begin(sck, miso, mosi, cs);
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);

    Logger::info("SPI initialized (SCK=%d, MISO=%d, MOSI=%d, CS=%d)", sck, miso, mosi, cs);
}

void SPIManager::transfer(uint8_t data) {
    digitalWrite(_csPin, LOW);
    _spi->transfer(data);
    digitalWrite(_csPin, HIGH);
}

uint8_t SPIManager::receive() {
    digitalWrite(_csPin, LOW);
    uint8_t result = _spi->transfer(0x00);
    digitalWrite(_csPin, HIGH);
    return result;
}

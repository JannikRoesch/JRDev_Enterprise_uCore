#include "UARTManager.h"
#include "../Utilities/Logger.h"

using namespace JRDev;

static HardwareSerial* _serial = nullptr;

void UARTManager::begin(HardwareSerial& serial, uint32_t baud, int tx, int rx) {
    _serial = &serial;
    _serial->begin(baud, SERIAL_8N1, rx, tx);
    Logger::info("UART started @ %lu baud (TX=%d, RX=%d)", baud, tx, rx);
}

void UARTManager::write(const String& data) {
    if (_serial) {
        _serial->println(data);
    }
}

String UARTManager::readLine() {
    String line;
    if (_serial && _serial->available()) {
        line = _serial->readStringUntil('\n');
    }
    return line;
}

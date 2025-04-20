#include "BLEManager.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "../Utilities/Logger.h"

using namespace JRDev;

static BLEAdvertising* _advertising = nullptr;

void BLEManager::begin(const String& deviceName) {
    BLEDevice::init(deviceName.c_str());
    BLEServer* pServer = BLEDevice::createServer();

    _advertising = BLEDevice::getAdvertising();
    _advertising->setScanResponse(false);
    _advertising->setMinPreferred(0x06);
    _advertising->setMinPreferred(0x12);

    Logger::info("BLE initialized with name: %s", deviceName.c_str());
}

void BLEManager::sendAdvertisement(const String& payload) {
    if (!_advertising) {
        Logger::error("BLE not initialized.");
        return;
    }

    BLEAdvertisementData advData;
    advData.setName("ESP32-BLE");
    advData.addData(std::string("\x09" + payload).c_str());

    _advertising->setAdvertisementData(advData);
    _advertising->start();

    Logger::info("BLE Advertisement started with payload: %s", payload.c_str());
}

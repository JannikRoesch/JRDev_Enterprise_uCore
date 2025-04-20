#include "GPIOConfigManager.h"
#include <ArduinoJson.h>
#include "../Utilities/Logger.h"

using namespace JRDev;

std::map<uint8_t, GPIORole> GPIOConfigManager::_pinRoles;

void GPIOConfigManager::setRole(uint8_t pin, GPIORole role) {
    _pinRoles[pin] = role;
}

GPIORole GPIOConfigManager::getRole(uint8_t pin) {
    return _pinRoles.count(pin) ? _pinRoles[pin] : GPIORole::NONE;
}

void GPIOConfigManager::configurePin(uint8_t pin, GPIORole role, bool output) {
    pinMode(pin, output ? OUTPUT : INPUT);
    setRole(pin, role);
    Logger::info("Configured pin %d as %s", pin, output ? "OUTPUT" : "INPUT");
}

void GPIOConfigManager::fromJson(const String& jsonConfig) {
    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, jsonConfig);
    if (err) {
        Logger::error("Failed to parse GPIO config: %s", err.c_str());
        return;
    }

    for (JsonPair kv : doc.as<JsonObject>()) {
        uint8_t pin = atoi(kv.key().c_str());
        GPIORole role = static_cast<GPIORole>(kv.value().as<uint8_t>());
        setRole(pin, role);
    }

    Logger::info("GPIO config loaded from JSON.");
}

String GPIOConfigManager::toJson() {
    StaticJsonDocument<512> doc;
    for (auto& pair : _pinRoles) {
        doc[String(pair.first)] = static_cast<uint8_t>(pair.second);
    }
    String out;
    serializeJson(doc, out);
    return out;
}

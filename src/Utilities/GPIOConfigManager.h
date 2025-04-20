#pragma once

#include <Arduino.h>
#include <map>

namespace JRDev {

enum class GPIORole {
    NONE,
    LED,
    RELAY,
    BUTTON,
    SENSOR,
    CUSTOM
};

class GPIOConfigManager {
public:
    static void setRole(uint8_t pin, GPIORole role);
    static GPIORole getRole(uint8_t pin);
    static void configurePin(uint8_t pin, GPIORole role, bool output = true);
    static void fromJson(const String& jsonConfig);
    static String toJson();

private:
    static std::map<uint8_t, GPIORole> _pinRoles;
};

}

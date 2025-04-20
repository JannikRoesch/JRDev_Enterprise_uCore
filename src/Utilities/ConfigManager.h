#pragma once

#include <Arduino.h>
#include <map>

namespace JRDev {

class ConfigManager {
public:
    static void set(const String& key, const String& value);
    static String get(const String& key, const String& fallback = "");

private:
    static std::map<String, String> _configMap;
};

}

#include "ConfigManager.h"

using namespace JRDev;

std::map<String, String> ConfigManager::_configMap;

void ConfigManager::set(const String& key, const String& value) {
    _configMap[key] = value;
}

String ConfigManager::get(const String& key, const String& fallback) {
    return _configMap.count(key) ? _configMap[key] : fallback;
}

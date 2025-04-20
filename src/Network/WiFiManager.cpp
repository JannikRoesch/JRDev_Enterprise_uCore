#include "WiFiManager.h"
#include "../Utilities/Logger.h"

using namespace JRDev;

bool WiFiManager::connect(const char* ssid, const char* pass) {
    Logger::info("Connecting to WiFi SSID: %s", ssid);
    WiFi.begin(ssid, pass);

    int retries = 20;
    while (WiFi.status() != WL_CONNECTED && retries-- > 0) {
        delay(500);
        Logger::info(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Logger::info("WiFi connected. IP: %s", WiFi.localIP().toString().c_str());
        return true;
    } else {
        Logger::error("WiFi connection failed.");
        return false;
    }
}

bool WiFiManager::isConnected() {
    return WiFi.status() == WL_CONNECTED;
}

void WiFiManager::disconnect() {
    WiFi.disconnect(true);
    Logger::warn("WiFi disconnected.");
}

#pragma once

#include <WiFi.h>
#include "../uConfig.h"

namespace JRDev {

class WiFiManager {
public:
    static bool connect(const char* ssid = DEFAULT_WIFI_SSID, const char* pass = DEFAULT_WIFI_PASS);
    static bool isConnected();
    static void disconnect();
};

}

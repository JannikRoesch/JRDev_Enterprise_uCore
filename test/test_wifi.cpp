#include <Arduino.h>
#include <unity.h>
#include "Network/WiFiManager.h"
#include "uConfig.h"

void test_wifi_connection() {
    bool connected = JRDev::WiFiManager::connect(DEFAULT_WIFI_SSID, DEFAULT_WIFI_PASS);
    TEST_ASSERT_TRUE_MESSAGE(connected, "WiFi failed to connect");
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_wifi_connection);
    UNITY_END();
}

void loop() {}

#include <Arduino.h>
#include <unity.h>
#include "Utilities/ConfigManager.h"

void test_config_set_get() {
    JRDev::ConfigManager::set("key", "value");
    String result = JRDev::ConfigManager::get("key");
    TEST_ASSERT_EQUAL_STRING("value", result.c_str());
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_config_set_get);
    UNITY_END();
}

void loop() {}

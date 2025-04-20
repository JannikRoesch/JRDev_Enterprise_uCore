#include <Arduino.h>
#include <unity.h>
#include "Core/ESP32Core.h"

void test_chip_info() {
    JRDev::ESP32Core::printChipInfo();
    TEST_ASSERT_EQUAL(2, ESP.getChipCores());  // abhängig vom Board
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_chip_info);
    UNITY_END();
}

void loop() {}

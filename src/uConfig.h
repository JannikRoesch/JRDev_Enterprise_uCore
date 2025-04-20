#pragma once

// Konfigurationsprofil
#define CONFIG_PROFILE_ADVANCED

#ifdef CONFIG_PROFILE_ADVANCED
    #define ENABLE_LOGGING true
    #define DEFAULT_CPU_FREQ 160
    #define DEFAULT_WIFI_SSID "MySSID"
    #define DEFAULT_WIFI_PASS "MyPassword"
    #define DEFAULT_MQTT_BROKER "mqtt.example.com"
    #define DEFAULT_OTA_HOSTNAME "esp32-enterprise"
#else
    #define ENABLE_LOGGING false
    #define DEFAULT_CPU_FREQ 80
#endif

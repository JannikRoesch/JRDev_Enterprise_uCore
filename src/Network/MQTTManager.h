#pragma once

#include <PubSubClient.h>
#include <WiFi.h>
#include "../uConfig.h"

namespace JRDev {

class MQTTManager {
public:
    static void begin(const char* server = DEFAULT_MQTT_BROKER, uint16_t port = 8883);
    static void loop();
    static void publish(const char* topic, const char* payload);
    static void setCallback(MQTT_CALLBACK_SIGNATURE);
    static bool isConnected();

private:
    static WiFiClientSecure _wifiClient;
    static PubSubClient _mqttClient;
    static void reconnect();
};

}

#include <Arduino.h>
#include "Network/WiFiManager.h"
#include "Network/MQTTManager.h"
#include "Network/OTAUpdater.h"

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.printf("[MQTT] Message received on %s: ", topic);
    for (int i = 0; i < length; i++) Serial.print((char)payload[i]);
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    JRDev::WiFiManager::connect();
    JRDev::OTAUpdater::begin();
    JRDev::MQTTManager::begin();
    JRDev::MQTTManager::setCallback(mqttCallback);
    JRDev::MQTTManager::publish("esp32/status", "Online");
}

void loop() {
    JRDev::MQTTManager::loop();
    JRDev::OTAUpdater::handle();
}

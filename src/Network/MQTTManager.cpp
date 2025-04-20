#include "MQTTManager.h"
#include "../Utilities/Logger.h"

using namespace JRDev;

WiFiClientSecure MQTTManager::_wifiClient;
PubSubClient MQTTManager::_mqttClient(_wifiClient);

void MQTTManager::begin(const char* server, uint16_t port) {
    _wifiClient.setInsecure(); // Optional: Zertifikat prüfen
    _mqttClient.setServer(server, port);
    reconnect();
}

void MQTTManager::reconnect() {
    int retries = 5;
    while (!_mqttClient.connected() && retries-- > 0) {
        Logger::info("Connecting to MQTT broker...");
        String clientId = "ESP32Client-" + String(random(0xffff), HEX);
        if (_mqttClient.connect(clientId.c_str())) {
            Logger::info("MQTT connected.");
        } else {
            Logger::warn("Failed. Code: %d. Retrying...", _mqttClient.state());
            delay(1000);
        }
    }
}

void MQTTManager::loop() {
    if (!_mqttClient.connected()) {
        reconnect();
    }
    _mqttClient.loop();
}

void MQTTManager::publish(const char* topic, const char* payload) {
    if (_mqttClient.connected()) {
        _mqttClient.publish(topic, payload);
        Logger::info("Published to %s: %s", topic, payload);
    }
}

void MQTTManager::setCallback(MQTT_CALLBACK_SIGNATURE) {
    _mqttClient.setCallback(callback);
}

bool MQTTManager::isConnected() {
    return _mqttClient.connected();
}

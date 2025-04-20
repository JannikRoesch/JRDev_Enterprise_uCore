#include "AppController.h"
#include "../Utilities/Logger.h"
#include "../Utilities/ErrorReporter.h"
#include "../Network/WiFiManager.h"
#include "../Network/MQTTManager.h"
#include "../Network/OTAUpdater.h"
#include "../Network/SecureOTAManager.h"
#include "../Communication/WebSerialManager.h"

using namespace JRDev;

static unsigned long _lastStatusTime = 0;

void AppController::begin() {
    Logger::info("AppController starting...");

    WiFiManager::connect();
    OTAUpdater::begin();
    SecureOTAManager::begin();
    MQTTManager::begin();

    WebSerialManager::begin();
    WebSerialManager::setCommandCallback([](const String& cmd) {
        Logger::info("[WebSerial CMD] %s", cmd.c_str());
        // Command parsing hier möglich
		if (cmd == "heap") {
            JRDev::WebSerialManager::send("Heap: " + String(ESP.getFreeHeap()) + " Bytes");
        } else if (cmd == "reset") {
            JRDev::WebSerialManager::send("Reboot in 2s...");
            delay(2000);
            ESP.restart();
        } else if (cmd == "status") {
            JRDev::WebSerialManager::send("Uptime: " + String(millis() / 1000) + "s");
        } else {
            JRDev::WebSerialManager::send("Unbekannter Befehl: " + cmd);
        }
    });

    ErrorReporter::setOnErrorCallback([](ErrorCode code, String msg) {
        Logger::error("GLOBAL ERROR: [%s] %s", errorToString(code), msg.c_str());
        // Option: MQTTManager::publish("errors", ...);
    });

    Logger::info("AppController ready.");
}

void AppController::loop() {
    MQTTManager::loop();
    OTAUpdater::handle();
    WebSerialManager::handleClient();

    // Optional: regelmäßiger Status-Output
    if (millis() - _lastStatusTime > 10000) {
        Logger::info("System running...");
        _lastStatusTime = millis();
    }
}

void AppController::safeReboot() {
    Logger::warn("Safe reboot triggered.");
    delay(500);
    ESP.restart();
}

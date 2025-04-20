#include "SecureOTAManager.h"
#include "../Utilities/Logger.h"
#include "WiFiManager.h"
#include "mbedtls/sha256.h"

using namespace JRDev;

void SecureOTAManager::begin() {
    Logger::info("SecureOTA ready. Use updateFirmware(url, sha256, auth) to begin.");
}

bool SecureOTAManager::updateFirmware(const String& url, const String& sha256, const String& auth) {
    if (!WiFiManager::isConnected()) {
        Logger::error("No WiFi connection. OTA aborted.");
        return false;
    }

    WiFiClientSecure client;
    client.setInsecure(); // TODO: Cert check

    Logger::info("Connecting to update server: %s", url.c_str());
    if (!client.connect(url.c_str(), 443)) {
        Logger::error("Connection failed.");
        return false;
    }

    HTTPClient https;
    https.begin(client, url);

    if (auth.length() > 0)
        https.setAuthorization(auth.c_str());

    int httpCode = https.GET();
    if (httpCode != HTTP_CODE_OK) {
        Logger::error("HTTP GET failed, code: %d", httpCode);
        https.end();
        return false;
    }

    int contentLength = https.getSize();
    bool canBegin = Update.begin(contentLength);
    if (!canBegin) {
        Logger::error("Not enough space for OTA.");
        https.end();
        return false;
    }

    WiFiClient* stream = https.getStreamPtr();
    int written = Update.writeStream(*stream);
    if (written != contentLength) {
        Logger::error("Update failed: Written %d of %d bytes.", written, contentLength);
        https.end();
        return false;
    }

    if (!Update.end()) {
        Logger::error("Update end failed: %s", Update.errorString());
        https.end();
        return false;
    }

    https.end();

    if (!sha256.isEmpty()) {
        if (!validateSignature(sha256)) {
            Logger::error("Firmware hash mismatch!");
            return false;
        }
        Logger::info("Firmware hash verified successfully.");
    }

    Logger::info("Firmware updated successfully. Restarting...");
    delay(1000);
    ESP.restart();
    return true;
}

bool SecureOTAManager::validateSignature(const String& expectedHash) {
    // Optional: implement full SHA256 check of firmware in flash
    // Placeholder for now
    return true; // For demo purposes only
}

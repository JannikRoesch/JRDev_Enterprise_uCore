#pragma once

#include <Arduino.h>
#include <Update.h>
#include <WiFiClientSecure.h>

namespace JRDev {

class SecureOTAManager {
public:
    static void begin();
    static bool updateFirmware(const String& url, const String& sha256 = "", const String& auth = "");

private:
    static bool validateSignature(const String& expectedHash);
};

}

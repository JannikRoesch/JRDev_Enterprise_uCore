#include "ErrorCodes.h"

using namespace JRDev;

const char* errorToString(ErrorCode code) {
    switch (code) {
        case ErrorCode::OK: return "OK";
        case ErrorCode::WIFI_FAILED: return "WiFi connection failed";
        case ErrorCode::MQTT_DISCONNECTED: return "MQTT disconnected";
        case ErrorCode::FS_MOUNT_FAILED: return "Filesystem mount failed";
        case ErrorCode::CONFIG_INVALID: return "Configuration invalid";
        default: return "Unknown error";
    }
}

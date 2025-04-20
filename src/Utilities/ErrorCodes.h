#pragma once

namespace JRDev {

enum class ErrorCode {
    OK = 0,
    WIFI_FAILED,
    MQTT_DISCONNECTED,
    FS_MOUNT_FAILED,
    CONFIG_INVALID,
    UNKNOWN
};

const char* errorToString(ErrorCode code);

}

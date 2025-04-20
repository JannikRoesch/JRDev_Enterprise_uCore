#pragma once

#include "ErrorCodes.h"
#include <Arduino.h>

namespace JRDev {

class ErrorReporter {
public:
    static void report(ErrorCode code, const String& message = "");
    static void reset();
    static ErrorCode lastError();
    static String lastMessage();
    static void setOnErrorCallback(void (*callback)(ErrorCode, String));

private:
    static ErrorCode _lastError;
    static String _lastMessage;
    static void (*_callback)(ErrorCode, String);
};

}

#include "ErrorReporter.h"
#include "../Utilities/Logger.h"

using namespace JRDev;

ErrorCode ErrorReporter::_lastError = ErrorCode::OK;
String ErrorReporter::_lastMessage = "";
void (*ErrorReporter::_callback)(ErrorCode, String) = nullptr;

void ErrorReporter::report(ErrorCode code, const String& message) {
    _lastError = code;
    _lastMessage = message;

    Logger::error("Error occurred: [%s] %s", errorToString(code), message.c_str());

    if (_callback) {
        _callback(code, message);
    }
}

void ErrorReporter::reset() {
    _lastError = ErrorCode::OK;
    _lastMessage = "";
}

ErrorCode ErrorReporter::lastError() {
    return _lastError;
}

String ErrorReporter::lastMessage() {
    return _lastMessage;
}

void ErrorReporter::setOnErrorCallback(void (*callback)(ErrorCode, String)) {
    _callback = callback;
}

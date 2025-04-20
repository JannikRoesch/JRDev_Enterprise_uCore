#include "Logger.h"
#include <stdarg.h>

using namespace JRDev;

void Logger::info(const char* format, ...) {
#if ENABLE_LOGGING
    Serial.print("[INFO] ");
    va_list args;
    va_start(args, format);
    Serial.vprintf(format, args);
    va_end(args);
    Serial.println();
#endif
}

void Logger::warn(const char* format, ...) {
#if ENABLE_LOGGING
    Serial.print("[WARN] ");
    va_list args;
    va_start(args, format);
    Serial.vprintf(format, args);
    va_end(args);
    Serial.println();
#endif
}

void Logger::error(const char* format, ...) {
#if ENABLE_LOGGING
    Serial.print("[ERROR] ");
    va_list args;
    va_start(args, format);
    Serial.vprintf(format, args);
    va_end(args);
    Serial.println();
#endif
}

#pragma once

#include <Arduino.h>
#include "../uConfig.h"

namespace JRDev {

class Logger {
public:
    static void info(const char* format, ...);
    static void warn(const char* format, ...);
    static void error(const char* format, ...);
};

}

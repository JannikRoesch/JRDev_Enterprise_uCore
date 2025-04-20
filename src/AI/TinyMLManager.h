#pragma once

#include <Arduino.h>

namespace JRDev {

class TinyMLManager {
public:
    static void begin();
    static float* runInference(const float* input, size_t length);
    static void printOutput(float* output, size_t length);
};

}

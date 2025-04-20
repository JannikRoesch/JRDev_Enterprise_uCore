#pragma once

namespace JRDev {

class PowerManager {
public:
    static void enterDeepSleep(uint64_t duration_us = 10e6);
    static void enterLightSleep(uint64_t duration_us = 10e6);
};

}

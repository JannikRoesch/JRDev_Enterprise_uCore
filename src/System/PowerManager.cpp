#include "PowerManager.h"
#include "esp_sleep.h"
#include "../Utilities/Logger.h"

using namespace JRDev;

void PowerManager::enterDeepSleep(uint64_t duration_us) {
    Logger::info("Entering deep sleep for %.2f seconds", duration_us / 1e6);
    esp_sleep_enable_timer_wakeup(duration_us);
    esp_deep_sleep_start();
}

void PowerManager::enterLightSleep(uint64_t duration_us) {
    Logger::info("Entering light sleep for %.2f seconds", duration_us / 1e6);
    esp_sleep_enable_timer_wakeup(duration_us);
    esp_light_sleep_start();
}

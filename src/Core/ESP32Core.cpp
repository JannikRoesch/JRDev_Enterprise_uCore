#include "ESP32Core.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "../uConfig.h"
#include "../Utilities/Logger.h"

using namespace JRDev;

void ESP32Core::printChipInfo() {
    Logger::info("Chip Info:");
    Logger::info("Cores: %d", ESP.getChipCores());
    Logger::info("Revision: %d", ESP.getChipRevision());
    Logger::info("Flash Size: %dMB", ESP.getFlashChipSize() / (1024 * 1024));
}

void ESP32Core::setCpuFrequencyMhz(uint8_t mhz) {
    setCpuFrequencyMhz(mhz);
    Logger::info("CPU Frequency set to %d MHz", mhz);
}

void ESP32Core::enableWatchdog(uint32_t timeout_ms) {
    esp_task_wdt_init(timeout_ms / 1000, true);
    esp_task_wdt_add(NULL);
    Logger::warn("Watchdog enabled with %lu ms timeout", timeout_ms);
}

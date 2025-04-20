#include "FilesystemManager.h"
#include <LittleFS.h>
#include "../Utilities/Logger.h"

using namespace JRDev;

bool FilesystemManager::begin() {
    if (!LittleFS.begin()) {
        Logger::error("LittleFS mount failed!");
        return false;
    }
    Logger::info("LittleFS mounted.");
    return true;
}

String FilesystemManager::readFile(const char* path) {
    File file = LittleFS.open(path, "r");
    if (!file) {
        Logger::error("Failed to open file: %s", path);
        return "";
    }
    return file.readString();
}

void FilesystemManager::writeFile(const char* path, const String& data) {
    File file = LittleFS.open(path, "w");
    if (!file) {
        Logger::error("Failed to write file: %s", path);
        return;
    }
    file.print(data);
    Logger::info("File %s written.", path);
}

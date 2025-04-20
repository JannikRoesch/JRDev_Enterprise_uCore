#pragma once

namespace JRDev {

class FilesystemManager {
public:
    static bool begin();
    static String readFile(const char* path);
    static void writeFile(const char* path, const String& data);
};

}

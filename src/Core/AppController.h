#pragma once

#include <Arduino.h>

namespace JRDev {

class AppController {
public:
    static void begin();      // Initialisierung
    static void loop();       // Zyklische Hauptschleife
    static void safeReboot(); // Geordneter Reboot
};

}

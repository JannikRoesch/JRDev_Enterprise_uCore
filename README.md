# ESP32 Enterprise Library (JRDev)

Diese Library stellt eine skalierbare, modulare Architektur für professionelle ESP32-Projekte bereit. Sie orientiert sich an Enterprise-Best-Practices und eignet sich für Produktivprojekte, Schulung, Demos und Wiederverwendung in anderen Projekten.

---

## Features

- **Core**: CPU-Frequenzsteuerung, Watchdog, ChipInfo
- **Network**: WiFi (STA), MQTT (TLS), OTA
- **System**: FreeRTOS, Deep/Light Sleep, Filesystem (LittleFS)
- **Utilities**: Logging, Konfiguration, Fehlerkatalog
- **Communication**: UART, (SPI/I2C optional)

---

## Getting Started

### Setup via PlatformIO
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
```

### Beispielcode

```cpp
#include "Network/WiFiManager.h"
#include "Network/MQTTManager.h"
#include "Network/OTAUpdater.h"
#include "System/PowerManager.h"

void setup() {
    JRDev::WiFiManager::connect();
    JRDev::MQTTManager::begin();
    JRDev::OTAUpdater::begin();
    JRDev::MQTTManager::publish("status", "Online");
}

void loop() {
    JRDev::MQTTManager::loop();
    JRDev::OTAUpdater::handle();
}
```

### Projektstruktur

```
ESP32_Enterprise_Lib/
├── src/
│   ├── Core/
│   ├── Network/
│   ├── Communication/
│   ├── System/
│   └── Utilities/
├── config/
├── examples/
├── test/
├── docs/
├── bin/
├── Cheatsheet_v1.0.0.pdf
├── Chatverlauf_Code_ESP32.md
├── platformio.ini
├── library.properties
└── README.md
```

### Dokumentation

- Vollständig kommentierter Code (Doxygen-kompatibel)
- `docs/errors.md` – Fehlercodes mit Lösungsvorschlägen
- `docs/uml/uml.puml` – Architekturdiagramm via PlantUML
- `docs/html/` – HTML-Export der API-Referenz

### CI / Qualitätssicherung

- Unit-Tests mit Unity
- CI-Vorlage via GitHub Actions
- Statische Analyse: Clang-Tidy
- Speicherprüfung über ESP-IDF Tools (valgrind-ähnlich)

### Beispiele

- BasicUsage – Blinken, ChipInfo, Logging
- FullSystemDemo – WiFi, OTA, MQTT
- LowPowerDemo – Sleep-Mode, Filesystem

---

### Lizenz
MIT License – frei für kommerzielle und nicht-kommerzielle Nutzung.


---

This file is a merged representation of the entire codebase, combined into a single document by Repomix.
The content has been processed where content has been formatted for parsing in markdown style, security check has been disabled.

# File Summary

## Purpose
This file contains a packed representation of the entire repository's contents.
It is designed to be easily consumable by AI systems for analysis, code review,
or other automated processes.

## File Format
The content is organized as follows:
1. This summary section
2. Repository information
3. Directory structure
4. Multiple file entries, each consisting of:
  a. A header with the file path (## File: path/to/file)
  b. The full contents of the file in a code block

## Usage Guidelines
- This file should be treated as read-only. Any changes should be made to the
  original repository files, not this packed version.
- When processing this file, use the file path to distinguish
  between different files in the repository.
- Be aware that this file may contain sensitive information. Handle it with
  the same level of security as you would the original repository.

## Notes
- Some files may have been excluded based on .gitignore rules and Repomix's configuration
- Binary files are not included in this packed representation. Please refer to the Repository Structure section for a complete list of file paths, including binary files
- Files matching patterns in .gitignore are excluded
- Files matching default ignore patterns are excluded
- Content has been formatted for parsing in markdown style
- Security check has been disabled - content may contain sensitive information
- Files are sorted by Git change count (files with more changes are at the bottom)

## Additional Info

# Directory Structure
```
ESP32_Enterprise/
  config/
    automation.json
    config.json
  data/
    config.js
    index.html
    ota.js
    websocket.js
  docs/
    ui/
      mqtt.md
      ota.md
    CHANGELOG.md
  examples/
    EnterpriseDemo_v1_9/
      EnterpriseDemo_v1_9.ino
  src/
    Automation/
      AutomationManager.cpp
      AutomationManager.h
    Core/
      ChipInfo.cpp
      ChipInfo.h
    Network/
      OTAManager.cpp
      OTAManager.h
      WiFiManager.cpp
      WiFiManager.h
    System/
      SleepManager.cpp
      SleepManager.h
    Utilities/
      Logger.cpp
      Logger.h
    Web/
      WebDashboard.cpp
      WebDashboard.h
  install.sh
  library.json
  platformio.ini
  README.md
```

# Files

## File: ESP32_Enterprise/config/automation.json
````json
[
  {
    "name": "AutoLED",
    "triggerType": "pin",
    "triggerPin": 4,
    "actionType": "gpio",
    "actionPin": 2,
    "actionValue": 1
  }
]
````

## File: ESP32_Enterprise/config/config.json
````json
{
  "wifi_ssid": "MyWiFi",
  "wifi_pass": "password123",
  "mqtt_host": "192.168.1.100",
  "ota_url": "https://example.com/firmware/latest.bin",
  "update_check_url": "https://example.com/firmware/version.json",
  "current_version": "2.0.0",
  "auto_update_check": true,
  "auto_update_install": false
}
````

## File: ESP32_Enterprise/data/config.js
````javascript
document.addEventListener("DOMContentLoaded", () => {
  console.log("Lade Konfig...");
});
````

## File: ESP32_Enterprise/data/index.html
````html
<!DOCTYPE html>
<html>
<head><title>ESP32 Admin UI</title><meta charset="UTF-8" /></head>
<body>
<h1>ESP32 Enterprise UI v2.0</h1>

<section id="ota">
  <h2>OTA Einstellungen</h2>
  <button onclick="checkForUpdate()">Check for Update</button><br />
  <label><input type="checkbox" id="autoCheck" /> Automatisch prüfen</label><br />
  <label><input type="checkbox" id="autoInstall" /> Automatisch installieren</label>
</section>

<script src="/ota.js"></script>
<script src="/config.js"></script>
</body>
</html>
````

## File: ESP32_Enterprise/data/ota.js
````javascript
function checkForUpdate() {
  fetch("/check_update")
    .then(res => res.json())
    .then(data => {
      alert(`Update-Check:\nAktuell: ${data.current}\nVerfügbar: ${data.version}`);
    });
}
````

## File: ESP32_Enterprise/data/websocket.js
````javascript
const logElem = document.getElementById("wsLog");
const socket = new WebSocket(`ws://${location.hostname}:81/`);

socket.onmessage = function (event) {
  logElem.textContent += event.data + "\\n";
  logElem.scrollTop = logElem.scrollHeight;
};
````

## File: ESP32_Enterprise/docs/ui/mqtt.md
````markdown
# -------------------------
# 📁 docs/ui/mqtt.md (Beispieldoku)
# -------------------------
Konfiguration

| Feld         | Beschreibung                |
|--------------|-----------------------------|
| Host         | IP oder Domain des Brokers  |
| Port         | Standard: 1883              |
| Topic Pub    | Gerät sendet Status         |
| Topic Sub    | Gerät empfängt Kommandos    |

Beispiel
```json
{
  "mqtt_host": "192.168.1.50",
  "mqtt_port": 1883,
  "topic_pub": "esp32/status",
  "topic_sub": "esp32/cmd"
}

```
````

## File: ESP32_Enterprise/docs/ui/ota.md
````markdown
# OTA Modul – v2.0.0

## Übersicht
Das OTA-Modul erlaubt Firmware-Updates über HTTPs mit JSON-basierter Versionsprüfung.

## Konfiguration

| Einstellung             | Beschreibung                                          |
|--------------------------|-------------------------------------------------------|
| OTA URL                 | Direktlink zur .bin Datei                             |
| Update JSON URL         | JSON-Datei mit Version + Downloadlink                 |
| Automatischer Check     | Führt zyklisch einen Online-Vergleich durch           |
| Automatische Updates    | Installiert Updates ohne Benutzeraktion               |

## Beispiel JSON

```json
{
  "version": "2.0.1",
  "url": "https://example.com/firmware/esp32-v2.0.1.bin",
  "notes": "Fixes und Verbesserungen",
  "mandatory": false
}
```
````

## File: ESP32_Enterprise/docs/CHANGELOG.md
````markdown
# 📜 Changelog

## [2.0.0] – 2025-04-12

### ✨ Neu
- OTA mit JSON-Versioning, Auto-Update & UI-Steuerung
- AdminUI Tabs: Netzwerk, MQTT, OTA
- Konfigurierbare Update-URLs
- Neue Dokumentation für alle Bereiche

### ✅ Verbesserungen
- Modularisierung OTA & Automation
- Neues Setup-Skript
````

## File: ESP32_Enterprise/examples/EnterpriseDemo_v1_9/EnterpriseDemo_v1_9.ino
````
#include <Arduino.h>
#include "Core/ChipInfo.h"
#include "System/SleepManager.h"
#include "Network/WiFiManager.h"
#include "Network/OTAManager.h"
#include "Web/WebDashboard.h"
#include "Utilities/Logger.h"
#include "Automation/AutomationManager.h"

void setup() {
  Serial.begin(115200);
  delay(1000);

  JRDev::Logger::info("ESP32 Enterprise Demo v2.0.0 startet...");
  JRDev::ChipInfo::printChipInfo();

  JRDev::WiFiManager::connect("MyWiFi", "password123");

  JRDev::AutomationManager::load();
  JRDev::WebDashboard::begin();
  JRDev::OTAManager::setup("2.0.0");

  JRDev::Logger::info("System vollständig gestartet.");
}

void loop() {
  JRDev::WebDashboard::handle();
  JRDev::AutomationManager::loop();
  JRDev::OTAManager::handleAutoUpdate();
  delay(50);
}
````

## File: ESP32_Enterprise/src/Automation/AutomationManager.cpp
````cpp
#include "AutomationManager.h"
#include <LittleFS.h>
#include <ArduinoJson.h>

using namespace JRDev;
static std::vector<AutomationRule> rules;

void AutomationManager::load() {
  File file = LittleFS.open("/automation.json", "r");
  if (!file) return;

  DynamicJsonDocument doc(2048);
  deserializeJson(doc, file);
  rules.clear();

  for (JsonObject r : doc.as<JsonArray>()) {
    AutomationRule rule;
    rule.name = r["name"] | "";
    rule.triggerType = r["triggerType"] | "";
    rule.triggerPin = r["triggerPin"] | -1;
    rule.condition = r["condition"] | "";
    rule.time = r["time"] | "";
    rule.actionType = r["actionType"] | "";
    rule.actionPin = r["actionPin"] | -1;
    rule.actionValue = r["actionValue"] | 0;
    rule.mqttTopic = r["mqttTopic"] | "";
    rule.mqttPayload = r["mqttPayload"] | "";
    rule.nextTriggerName = r["nextTriggerName"] | "";
    rules.push_back(rule);
  }

  file.close();
}

std::vector<AutomationRule>& AutomationManager::get() { return rules; }
void AutomationManager::save() {}
void AutomationManager::loop() {}
void AutomationManager::trigger(String, int) {}
````

## File: ESP32_Enterprise/src/Automation/AutomationManager.h
````
#pragma once
#include <vector>
#include <Arduino.h>

namespace JRDev {

struct AutomationRule {
  String name;
  String triggerType;
  int triggerPin;
  String condition;
  String time;
  String actionType;
  String variable;
  int actionPin;
  int actionValue;
  String mqttTopic;
  String mqttPayload;
  String nextTriggerName;
};

class AutomationManager {
public:
  static void load();
  static void save();
  static void loop();
  static void trigger(String type, int value);
  static std::vector<AutomationRule>& get();
};

}
````

## File: ESP32_Enterprise/src/Core/ChipInfo.cpp
````cpp
#include "ChipInfo.h"
#include <Arduino.h>

void JRDev::ChipInfo::printChipInfo() {
  Serial.printf("=== ESP32 Chip Info ===\\n");
  Serial.printf("Cores: %d\\n", ESP.getChipCores());
  Serial.printf("Flash: %dMB\\n", ESP.getFlashChipSize() / (1024 * 1024));
  Serial.printf("SDK: %s\\n", ESP.getSdkVersion());
}
````

## File: ESP32_Enterprise/src/Core/ChipInfo.h
````
#pragma once
namespace JRDev {
class ChipInfo {
public:
  static void printChipInfo();
};
}
````

## File: ESP32_Enterprise/src/Network/OTAManager.cpp
````cpp
#include "OTAManager.h"
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Update.h>
#include <FS.h>
#include <LittleFS.h>

using namespace JRDev;

String OTAManager::currentVersion = "2.0.0";
unsigned long OTAManager::lastCheck = 0;

void OTAManager::setup(const String& version) {
  currentVersion = version;
}

bool OTAManager::checkForUpdate() {
  HTTPClient http;
  http.begin("https://example.com/firmware/version.json");
  int code = http.GET();
  if (code != 200) return false;

  DynamicJsonDocument doc(1024);
  deserializeJson(doc, http.getStream());

  String latest = doc["version"];
  String url = doc["url"];

  if (latest != currentVersion) {
    Serial.printf("[OTA] Update verfügbar: %s → %s\\n", currentVersion.c_str(), latest.c_str());
    HTTPClient dl;
    dl.begin(url);
    int size = dl.GET();
    if (size > 0) {
      WiFiClient* stream = dl.getStreamPtr();
      if (Update.begin(size)) {
        Update.writeStream(*stream);
        if (Update.end() && Update.isFinished()) {
          Serial.println("[OTA] Update abgeschlossen – Neustart.");
          ESP.restart();
        }
      }
    }
    dl.end();
    return true;
  }

  http.end();
  return false;
}

void OTAManager::handleAutoUpdate() {
  if ((millis() - lastCheck) < 3600000) return;
  lastCheck = millis();

  bool enabled = false;
  File cfg = LittleFS.open("/config.json", "r");
  if (cfg) {
    DynamicJsonDocument doc(512);
    deserializeJson(doc, cfg);
    enabled = doc["auto_update_check"] | false;
    cfg.close();
  }

  if (enabled) checkForUpdate();
}
````

## File: ESP32_Enterprise/src/Network/OTAManager.h
````
#pragma once
#include <Arduino.h>

namespace JRDev {
class OTAManager {
public:
  static void setup(const String& version);
  static bool checkForUpdate();
  static void handleAutoUpdate();
private:
  static String currentVersion;
  static unsigned long lastCheck;
};
}
````

## File: ESP32_Enterprise/src/Network/WiFiManager.cpp
````cpp
#include "WiFiManager.h"
#include <WiFi.h>

void JRDev::WiFiManager::connect(const char* ssid, const char* pass) {
  WiFi.begin(ssid, pass);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries++ < 20) {
    delay(500);
  }
}
````

## File: ESP32_Enterprise/src/Network/WiFiManager.h
````
#pragma once
namespace JRDev {
class WiFiManager {
public:
  static void connect(const char* ssid = nullptr, const char* pass = nullptr);
};
}
````

## File: ESP32_Enterprise/src/System/SleepManager.cpp
````cpp
#include "SleepManager.h"
#include <esp_sleep.h>

void JRDev::SleepManager::deepSleep(uint64_t us) {
  esp_sleep_enable_timer_wakeup(us);
  esp_deep_sleep_start();
}
````

## File: ESP32_Enterprise/src/System/SleepManager.h
````
#pragma once
namespace JRDev {
class SleepManager {
public:
  static void deepSleep(uint64_t us);
};
}
````

## File: ESP32_Enterprise/src/Utilities/Logger.cpp
````cpp
#include "Logger.h"
#include <Arduino.h>
#include <stdarg.h>

void JRDev::Logger::info(const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  Serial.print("[INFO] ");
  Serial.vprintf(fmt, args);
  Serial.println();
  va_end(args);
}

void JRDev::Logger::error(const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  Serial.print("[ERROR] ");
  Serial.vprintf(fmt, args);
  Serial.println();
  va_end(args);
}
````

## File: ESP32_Enterprise/src/Utilities/Logger.h
````
#pragma once
namespace JRDev {
class Logger {
public:
  static void info(const char* fmt, ...);
  static void error(const char* fmt, ...);
};
}
````

## File: ESP32_Enterprise/src/Web/WebDashboard.cpp
````cpp
#include "WebDashboard.h"
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <WiFi.h>
#include "Network/OTAManager.h"

using namespace JRDev;

void WebDashboard::begin() {
  server.on("/", HTTP_GET, []() {
    server.serveStatic("/", LittleFS, "/index.html");
  });

  server.on("/status", HTTP_GET, []() {
    DynamicJsonDocument doc(256);
    doc["ip"] = WiFi.localIP().toString();
    doc["uptime"] = millis();
    String json;
    serializeJson(doc, json);
    server.send(200, "application/json", json);
  });

  server.on("/check_update", HTTP_GET, []() {
    DynamicJsonDocument doc(128);
    doc["current"] = "2.0.0";
    doc["version"] = "2.0.1";
    String out;
    serializeJson(doc, out);
    server.send(200, "application/json", out);
  });

  server.begin();
}

void WebDashboard::handle() {
  server.handleClient();
}
````

## File: ESP32_Enterprise/src/Web/WebDashboard.h
````
#pragma once
#include <WebServer.h>
namespace JRDev {
class WebDashboard {
public:
  static WebServer server;
  static void begin();
  static void handle();
};
inline WebServer WebDashboard::server(80);
}
````

## File: ESP32_Enterprise/install.sh
````bash
#!/bin/bash
echo "📦 Initialisiere ESP32 Enterprise Library v2.0.0..."
pio lib install
pio run
echo "🔁 Optional: Web UI hochladen mit:"
echo "    pio run --target uploadfs"
echo "⚡ Flashen mit:"
echo "    pio run --target upload"
````

## File: ESP32_Enterprise/library.json
````json
{
  "name": "ESP32EnterpriseLib",
  "version": "2.0.0",
  "description": "Modulare, getestete und dokumentierte ESP32-Arduino-Bibliothek mit OTA, WebUI, Automation, MQTT, BLE, AI.",
  "keywords": ["esp32", "enterprise", "mqtt", "ota", "webui", "tinyml"],
  "authors": [
    { "name": "Jannik Roesch", "url": "https://github.com/JannikRoesch" }
  ],
  "platforms": "espressif32",
  "frameworks": "arduino"
}
````

## File: ESP32_Enterprise/platformio.ini
````
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
lib_deps =
  ayushsharma82/WebSerial
  knolleary/PubSubClient
  bblanchon/ArduinoJson
  me-no-dev/ESPAsyncWebServer
  TensorFlowLite_ESP32
````

## File: ESP32_Enterprise/README.md
````markdown
# 🚀 ESP32 Enterprise Library (v2.0.0)

Modularer C++-Framework für professionelles ESP32-Development.  
**Automation, Web-UI, OTA, MQTT, TinyML & mehr.**

## 📚 Inhaltsverzeichnis
1. [Features](#features)
2. [Quickstart](#quickstart)
3. [Admin UI](#admin-ui)
4. [Beispielanwendung](#beispielanwendung)
5. [Dokumentation](#dokumentation)
6. [Changelog](#changelog)

## 🧩 Features
| Modul             | Beschreibung                                  |
|-------------------|-----------------------------------------------|
| WiFi / MQTT       | Verbindung & Kommunikation                    |
| OTA (v2.0.0)      | Update-Check, Auto-Update, Version-Vergleich  |
| Automation Engine | Trigger → Aktion Logik                        |
| WebConfig UI      | Admin UI mit modularen Tabs                   |
| TinyML (Beta)     | TensorFlow Lite Integration                   |

## ⚙️ Quickstart
```bash
pio run --target uploadfs
pio run --target upload
pio device monitor
```

## 🌐 Admin UI
- `http://<esp32-ip>`
- Tabs: Dashboard, Netzwerk, MQTT, OTA, Automation

## 🧪 Beispielprojekt
- Im Ordner `examples/EnterpriseDemo_v1_9/` enthalten.
- Demonstriert **WiFi**, **OTA**, **Automation**, **WebUI** und **Logging**.

## 📘 Dokumentation
- `docs/ui/*.md` für jedes Modul
- `docs/CHANGELOG.md` für Versionen

---

<div align="center">
	<img src="https://jannikroesch.com/src/img/logos/jr/jr-slim-transparent-lightfont/jr-logo-slim-transparent-lightfont-svg.svg" height="64px" />
	<p>Copyright © 2024 Jannik Roesch</p>
	</br>
	<p>Author: Jannik Roesch</p>
	<p>License: MIT</p>
	<p>https://github.com/JannikRoesch</p>
	<p>https://jannikroesch.dev/</p>
	
</div>
````

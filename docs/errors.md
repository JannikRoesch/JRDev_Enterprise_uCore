# Fehlercodes – Übersicht

Diese Datei enthält alle möglichen Fehlercodes, die über `JRDev::ErrorCode` zurückgegeben werden können – mit kurzer Erläuterung und ggf. Lösungshinweisen.

| Code                  | Beschreibung                                      | Lösungsvorschlag                             |
|-----------------------|---------------------------------------------------|----------------------------------------------|
| OK                    | Kein Fehler                                       | –                                            |
| WIFI_FAILED           | Verbindung zu WiFi fehlgeschlagen                 | SSID/Passwort prüfen, AP-Reichweite prüfen   |
| MQTT_DISCONNECTED     | Verbindung zum MQTT-Broker unterbrochen           | TLS prüfen, Reconnect aktiv?                 |
| FS_MOUNT_FAILED       | LittleFS konnte nicht eingebunden werden          | Formatierung oder Dateisystem prüfen         |
| CONFIG_INVALID        | Konfigurationseinstellungen unzulässig/fehlen    | `uConfig.h` prüfen oder `ConfigManager`      |
| UNKNOWN               | Allgemeiner oder unbekannter Fehler               | Weitere Logs aktivieren, Debuggen            |

#pragma once

#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFi.h>

namespace JRDev {

class WebSerialManager {
public:
    static void begin(uint16_t httpPort = 80, uint16_t wsPort = 81);
    static void handleClient();
    static void send(const String& message);
    static void setCommandCallback(void (*callback)(const String&));

private:
    static WebServer _server;
    static WebSocketsServer _webSocket;
    static void (*_cmdCallback)(const String&);
    static void handleCommandMessage(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
};

}

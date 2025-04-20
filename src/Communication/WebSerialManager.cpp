#include "WebSerialManager.h"
#include "../Utilities/Logger.h"

using namespace JRDev;

WebServer WebSerialManager::_server(80);
WebSocketsServer WebSerialManager::_webSocket(81);
void (*WebSerialManager::_cmdCallback)(const String&) = nullptr;

void WebSerialManager::begin(uint16_t httpPort, uint16_t wsPort) {
    _server = WebServer(httpPort);
    _webSocket = WebSocketsServer(wsPort);

    _server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
    _server.begin();

    _webSocket.begin();
    _webSocket.onEvent(handleCommandMessage);

    Logger::info("WebServer gestartet auf Port %d", httpPort);
    Logger::info("WebSocket läuft auf /ws (Port %d)", wsPort);
}

void WebSerialManager::handleClient() {
    _server.handleClient();
    _webSocket.loop();
}

void WebSerialManager::send(const String& message) {
    _webSocket.broadcastTXT(message);
}

void WebSerialManager::setCommandCallback(void (*callback)(const String&)) {
    _cmdCallback = callback;
}

void WebSerialManager::handleCommandMessage(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
    if (type == WStype_TEXT) {
        String msg = String((char*)payload);
        Logger::info("[WebSocket CMD] %s", msg.c_str());
        if (_cmdCallback) {
            _cmdCallback(msg);
        } else {
            send("Kein Kommando-Handler definiert.");
        }
    }
}

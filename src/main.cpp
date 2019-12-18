#include <Arduino.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

extern "C" {
#include <buffer.h>
}

AsyncWebServer webserver(80);
AsyncWebSocket websocket("/ws");

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {

}

void setup() {
    SPIFFS.begin();
    webserver.serveStatic("/", SPIFFS, "/www/").setDefaultFile("index.html");
    websocket.onEvent(onWsEvent);
    webserver.addHandler(&websocket);
    webserver.begin();
}

void loop() {

}
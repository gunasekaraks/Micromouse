#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiServer.h>

class WiFiManager {
private:
    WiFiServer server;
    WiFiClient client;
    const char* ssid;
    const char* password;
    
public:
    WiFiManager(const char* wifi_ssid, const char* wifi_password, int port = 8888);
    
    bool begin();
    bool isConnected();
    bool hasClient();
    void sendData(String data);
    void sendDataLn(String data);
    void update();
    void printStatus();
};

#endif

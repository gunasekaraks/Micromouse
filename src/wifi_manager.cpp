#include "wifi_manager.h"

namespace {
// Default WiFi credentials (keep centralized here)
constexpr const char* kDefaultSsid = "Amiru";
constexpr const char* kDefaultPassword = "amiru123";
}

WiFiManager::WiFiManager()
    : WiFiManager(kDefaultSsid, kDefaultPassword, 8888) {
}

WiFiManager::WiFiManager(const char* wifi_ssid, const char* wifi_password, int port)
    : server(port), ssid(wifi_ssid), password(wifi_password) {
}

bool WiFiManager::begin() {
    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        printStatus();
        server.begin();
        return true;
    } else {
        Serial.println("\nWiFi connection failed!");
        return false;
    }
}

bool WiFiManager::isConnected() {
    return WiFi.status() == WL_CONNECTED;
}

bool WiFiManager::hasClient() {
    if (!client || !client.connected()) {
        client = server.available();
        if (client) {
            Serial.println("WiFi client connected!");
            client.println("Connected to Micromouse ESP32");
            client.println("Receiving sensor data...\n");
            return true;
        }
    }
    return client && client.connected();
}

void WiFiManager::sendData(String data) {
    // Send to Serial Monitor
    Serial.print(data);
    
    // Send to WiFi if client connected
    if (client && client.connected()) {
        client.print(data);
    }
}

void WiFiManager::sendDataLn(String data) {
    sendData(data + "\n");
}

void WiFiManager::update() {
    hasClient();  // Check for new connections
}

void WiFiManager::printStatus() {
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("TCP Server started on port 8888");
    Serial.print("Connect via: telnet ");
    Serial.print(WiFi.localIP());
    Serial.println(" 8888");
}

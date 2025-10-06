#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>

const char* hostname = "esp12f-fieroALDL";
const unsigned long TIMEOUT_MS = (60 * 1000) * 1; // 1 minute
unsigned long lastActiveTime = 0;
bool apActive = false;
IPAddress AP_IP(10,1,1,1);
IPAddress AP_subnet(255,255,255,0);
AsyncWebServer server(80);

void startAP() {
  Serial.println("Starting AP...");
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_IP, AP_subnet);
  WiFi.softAP(hostname);
  apActive = true;
  lastActiveTime = millis();
  Serial.print("AP started. Timeout: ");
  Serial.println((TIMEOUT_MS / 1000) / 60);
}

void stopAP() {
  Serial.println("No clients, disabling AP.");
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  apActive = false;
}

void setup() {
  Serial.begin(9600);
  Serial.println("\nBooting esp12f-fieroALDL");
  startAP();
  ElegantOTA.begin(&server);
  Serial.println("AP + OTA ready");
}

void loop() {
  // Check number of connected stations
  int numClients = WiFi.softAPgetStationNum();

  if (numClients > 0) {
    // Clients connected → reset timer
    lastActiveTime = millis();
  } else {
    // No clients → check if timeout expired
    if (apActive && millis() - lastActiveTime > TIMEOUT_MS) {
      stopAP();
    }
  }

  delay(1000);
}
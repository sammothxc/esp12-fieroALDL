#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>

#define LED_PIN 2
#define ALDL_DATA_PIN 3
#define ALDL_DATA_DURATION 2875
#define STATE_WAIT_SYNC 0
#define STATE_SYNC 1
#define STATE_DATA 2

const char* ssid = "esp12f-fieroALDL";
const char* password = "pontiacfiero";
IPAddress AP_IP(10,1,1,1);
IPAddress AP_subnet(255,255,255,0);
AsyncWebServer server(80);

void readALDLdata(void) {
  unsigned long ms = 0;
  unsigned long start = 0;
  unsigned long pulse = 0;
  int ALDL_DATA = 0;
  int bit = 0;
  int sync = 0;
  int state = STATE_WAIT_SYNC;
  int EXIT = 0;

  while(1) {
    // wait for possible partial pulse to finish
    while(digitalRead(ALDL_DATA_PIN) == 1) {if(Serial.available() > 0) {EXIT = 1; break;}}
    if(EXIT == 1) break;
    while(digitalRead(ALDL_DATA_PIN) == 0) {if(Serial.available() > 0) {EXIT = 1; break;}}
    if(EXIT == 1) break;

    // start timing the first pulse
    while(1) {
      while(digitalRead(ALDL_DATA_PIN) == 1) {if(Serial.available() > 0) {EXIT = 1; break;}}
      if(EXIT == 1) break;
      start = micros();

      while(1) {
        ms = micros();
        pulse = ms - start;
  
        if(pulse > ALDL_DATA_DURATION) {
          ALDL_DATA = digitalRead(ALDL_DATA_PIN);

          switch(state) {
            case STATE_WAIT_SYNC:
              if(ALDL_DATA == 0) {
                sync++;
                if(sync == 9) {state = STATE_SYNC;}
              }
            break;

            case STATE_SYNC:
              if(ALDL_DATA == 1) {bit++; state = STATE_DATA;}
            break;

            case STATE_DATA:
              bit++;
              if(bit == 225) {bit = 0; sync = 0; state = STATE_WAIT_SYNC;} // end of frame
              else {state = STATE_SYNC;}
            break;
          }

          switch(bit)
          {
            case 146: // not implemented in hardware
              Serial.print("0");
            break;

            case 147: // not implemented in hardware
              Serial.print("0");
            break;

            default:
              if(ALDL_DATA == 1) {Serial.print("0"); digitalWrite(LED_PIN, 0);}
              else {Serial.print("1"); digitalWrite(LED_PIN, 1);}
            break;
          }
          while(digitalRead(ALDL_DATA_PIN) == 0) {if(Serial.available() > 0) {EXIT = 1; break;}}
          break;
        }
      }
      if(EXIT == 1) break;
    }
    if(EXIT == 1) break;
  }
  return;
}

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
</html>
)rawliteral";

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(ALDL_DATA_PIN, INPUT_PULLUP);
  digitalWrite(LED_PIN, 0);
  Serial.begin(9600);
  Serial.println("\nBooting esp12f-fieroALDL...");
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_IP, AP_subnet);
  WiFi.softAP(ssid, password);
  ElegantOTA.begin(&server);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){request->send_P(200, "text/html", index_html);});
  server.begin();
  Serial.println("AP and OTA ready.");
  digitalWrite(LED_PIN, 1);
}

void loop() {
  readALDLdata();
}
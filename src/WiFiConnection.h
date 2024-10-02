#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <iostream>

using namespace std;

class WiFiConnection
{
private:
  const char *SSID;
  const char *PASSWORD;

public:
  WiFiConnection(const char *SSID, const char *PASSWORD)
      : SSID(SSID), PASSWORD(PASSWORD) {}

  void connect()
  {
    WiFi.begin(SSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(1000);
      Serial.println("Conectando a WiFi...");
    }
    Serial.println("Conectado a WiFi");
  }
};
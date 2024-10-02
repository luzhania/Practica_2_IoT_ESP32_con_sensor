#include <Arduino.h>
#include <string>
#include "UltraSonicSensor.h"
#include "WiFiConnection.h"
#include "Context.h"
const char *SERVER_HOST = "192.168.43.111";
const unsigned int SERVER_PORT = 8080;

class SensorClient
{
private:
  Context *context;
  WiFiClient client;
  WiFiConnection *wifiConnection;
  UltrasonicSensor *ultrasonicSensor;
  bool isConnected = false;

public:
  SensorClient(const char *SSID, const char *PASSWORD, unsigned int trigPin = 19, unsigned int echoPin = 22)
  {
    context = new Context(new State3);
    wifiConnection = new WiFiConnection(SSID, PASSWORD);
    ultrasonicSensor = new UltrasonicSensor(trigPin, echoPin);
  }

  ~SensorClient()
  {
    delete context;
    delete ultrasonicSensor;
    delete wifiConnection;
  }

  void setup()
  {
    Serial.begin(115200);
    wifiConnection->connect();
  }

  void loop()
  {
    if (!isConnected)
    {
      connectToServer();
    }
    else
    {
      sendDataToServer();
    }
  }

  void connectToServer()
  {
    if (!client.connected())
    {
      if (client.connect(SERVER_HOST, SERVER_PORT))
      {
        Serial.println("Conectado al servidor");
        isConnected = true;
        requestGetRanges();
      }
      else
      {
        Serial.println("Error al conectar con el servidor, intentando nuevamente...");
        delay(1000); 
      }
    }
  }

  void sendDataToServer()
  {
    float distance = ultrasonicSensor->getDistanceInCM();
    unsigned int actualState = context->determineState(distance);

    if (context->stateChanged(actualState))
    {
      context->changeContext(actualState);
      sendState();
    }

    if (!client.connected())
    {
      Serial.println("Desconectado del servidor, intentando reconectar...");
      isConnected = false; 
    }
    else
    {
      if (client.available() > 0)
      {
        String response = client.readStringUntil('\n');
        Serial.println("Respuesta del servidor: " + response);
      }
    }
  }

  void sendState()
  {
    unsigned int state = context->getStateID();

    if (client.connected())
    {
      String command = "PUT " + String(state);
      client.println(command);
      Serial.println("Comando enviado: " + command);
    }
    else
    {
      Serial.println("Error: No conectado al servidor, intentando reconectar...");
      isConnected = false; 
    }
  }

  void requestGetRanges()
  {
    if (client.connected())
    {
      client.println("GET RANGES");
      responseGetRanges();
    }
    else
    {
      Serial.println("Error: No conectado al servidor, intentando reconectar...");
      isConnected = false; 
    }
  }

  void setRanges(const String &response) {
    size_t space_pos = response.indexOf(' ');

    context->setStride(response.substring(0, space_pos).toInt());
    context->setLedQty(response.substring(space_pos + 1).toInt());
  }

  void responseGetRanges()
  {
    while(!client.available())
    {
      Utilities::nonBlockingDelay(500, []()
      {;});
    }
    String response = client.readStringUntil('\n');
    Serial.println("Respuesta del servidor: " + response);
    setRanges(response);
  }
};

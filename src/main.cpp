#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <iostream>
#include <string>
#include <functional>

using namespace std;

class Utilities
{
public:

  static void NonBlockingDelay(unsigned long milliseconds, std::function<void()> callback)
  {
    static unsigned long lastMeasurement = 0;
    unsigned long currentMillis = millis();

    if (currentMillis - lastMeasurement >= milliseconds)
    {
      callback();
      lastMeasurement = currentMillis;
    }
  }
};

class UltrasonicSensor
{
private:
  unsigned int triggerPin;
  unsigned int echoPin;

public:
  UltrasonicSensor(unsigned int trigger, unsigned int echo)
  {
    triggerPin = trigger;
    echoPin = echo;
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
  }

  float readDistance()
  {
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    return pulseIn(echoPin, HIGH);
  }

  float getDistanceInCM()
  {
    return 0.01723 * readDistance();
  }
};

class Context;
class State;

class State
{
protected:
  Context *context;

public:
  virtual ~State() {}
  void set_context(Context *context)
  {
    this->context = context;
  }

  virtual unsigned int getStateID() = 0;
};

class State0 : public State
{
public:
  unsigned int getStateID() override
  {
    return 0;
  }
};

class State1 : public State
{
public:
  unsigned int getStateID() override
  {
    return 1;
  }
};

class State2 : public State
{
public:
  unsigned int getStateID() override
  {
    return 2;
  }
};

class State3 : public State
{
public:
  unsigned int getStateID() override
  {
    return 3;
  }
};

class Context
{
private:
  State *state;
  unsigned int stride;
  unsigned int ledQty;
  unsigned int lastSentState;

public:
  Context(State *state, unsigned int stride = 6, unsigned int ledQty = 3)
      : state(nullptr), stride(stride), ledQty(ledQty), lastSentState(ledQty + 1)
  {
    this->transitionTo(state);
  }

  ~Context()
  {
    delete state;
  }

  void transitionTo(State *state)
  {
    Serial.print("Context: Transition to state ");
    Serial.println(state->getStateID());
    if (this->state != nullptr)
      delete this->state;
    this->state = state;
    this->state->set_context(this);
  }

  unsigned int getStateID()
  {
    return this->state->getStateID();
  }

  void setStride(unsigned int newStride)
  {
    stride = newStride;
  }

  void setLedQty(unsigned int newLedQty)
  {
    ledQty = newLedQty;
  }

  unsigned int determineState(float distance)
  {
    unsigned int state = distance / stride;
    return (state <= ledQty) ? state : ledQty;
  }

  bool stateChanged(unsigned int &actualState)
  {
    return actualState != lastSentState;
  }

  void changeContext(unsigned int actualState)
  {
    lastSentState = actualState;

    if (actualState == 0)
    {
      transitionTo(new State0());
    }
    else if (actualState == 1)
    {
      transitionTo(new State1());
    }
    else if (actualState == 2)
    {
      transitionTo(new State2());
    }
    else
    {
      transitionTo(new State3());
    }
  }
};

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
      Utilities::NonBlockingDelay(500, []()
      {;});
    }
    String response = client.readStringUntil('\n');
    Serial.println("Respuesta del servidor: " + response);
    setRanges(response);
  }
};

SensorClient client("Galaxy S9+7c14", "betitox007.,", 17, 19);

void setup()
{
  client.setup();
}

void loop()
{
  Utilities::NonBlockingDelay(100, []()
                              { client.loop(); });
}

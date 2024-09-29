#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <iostream>
#include <string>

class Utilities
{
public:
  static void nonBlockingDelayFor(unsigned long microseconds)
  {
    unsigned long startTime = micros();
    while (micros() - startTime < microseconds)
      ;
  }

  static void serialPrintNonBlockingDelay(unsigned long milliseconds, unsigned int cm)
  {
    static unsigned long lastMeasurement = 0;
    unsigned long currentMillis = millis();

    if (currentMillis - lastMeasurement >= 100)
    {
      Serial.print(cm);
      Serial.println("cm");
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
    Utilities::nonBlockingDelayFor(2);
    digitalWrite(triggerPin, HIGH);
    Utilities::nonBlockingDelayFor(10);
    digitalWrite(triggerPin, LOW);
    return pulseIn(echoPin, HIGH);
  }

  float getDistanceInCM()
  {
    return 0.01723 * readDistance();
  }
};


// Configuración del servidor TCP
const char *SERVER_HOST = "192.168.100.11";
const unsigned int SERVER_PORT = 8080;

// Prototipos de las clases para patrón State
class Context;
class State;

// Clase base para los estados
class State
{
protected:
  Context *context_;

public:
  virtual ~State() {}
  void set_context(Context *context)
  {
    this->context_ = context;
  }

  virtual void handleDistance(float distance) = 0;
  virtual int getStateID() = 0;
};

// Clase Contexto que contiene el estado actual
class Context
{
private:
  State *state_;

public:
  Context(State *state) : state_(nullptr)
  {
    this->transitionTo(state);
  }

  ~Context()
  {
    delete state_;
  }

  void transitionTo(State *state)
  {
    std::cout << "Context: Transition to state " << state->getStateID() << "\n";
    if (this->state_ != nullptr)
      delete this->state_;
    this->state_ = state;
    this->state_->set_context(this);
  }

  void request(float distance)
  {
    this->state_->handleDistance(distance);
  }

  int getStateID()
  {
    return this->state_->getStateID();
  }
};

// Estado 0: Encender 3 LEDs
class State0 : public State
{
public:
  void handleDistance(float distance) override
  {
    std::cout << "State 0: Encender 3 LEDs\n";
  }

  int getStateID() override
  {
    return 0;
  }
};

// Estado 1: Encender 2 LEDs
class State1 : public State
{
public:
  void handleDistance(float distance) override
  {
    std::cout << "State 1: Encender 2 LEDs\n";
  }

  int getStateID() override
  {
    return 1;
  }
};

// Estado 2: Encender 1 LED
class State2 : public State
{
public:
  void handleDistance(float distance) override
  {
    std::cout << "State 2: Encender 1 LED\n";
  }

  int getStateID() override
  {
    return 2;
  }
};

// Estado 3: No encender LEDs
class State3 : public State
{
public:
  void handleDistance(float distance) override
  {
    std::cout << "State 3: No encender LEDs\n";
  }

  int getStateID() override
  {
    return 3;
  }
};

class WiFiConnection {
private:
    const char *SSID;
    const char *PASSWORD;

public:
    WiFiConnection(const char *SSID, const char *PASSWORD)
        : SSID(SSID), PASSWORD(PASSWORD) {}

    void connect() {
        WiFi.begin(SSID, PASSWORD);
        while (WiFi.status() != WL_CONNECTED) {
            delay(1000);
            Serial.println("Conectando a WiFi...");
        }
        Serial.println("Conectado a WiFi");
    }
};


// Clase principal para el ESP32
class SensorClient
{
private:
  Context *context;
  WiFiClient client;
  WiFiConnection *wifiConnection;
  UltrasonicSensor *ultrasonicSensor; // Instancia del sensor ultrasónico
  unsigned int stride;
  unsigned int led_qty;
  int lastSentState;

public:
  SensorClient(const char *SSID, const char *PASSWORD, unsigned int trigPin = 19, unsigned int echoPin = 22)
  {
    context = new Context(new State3);
    wifiConnection = new WiFiConnection(SSID, PASSWORD);
    ultrasonicSensor = new UltrasonicSensor(trigPin, echoPin);
    stride = 6;
    led_qty = 3;
    lastSentState = -1;
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
    if (client.connected())
    {
      float distance = ultrasonicSensor->getDistanceInCM();
      int actualState = determineState(distance);

      if (stateChanged(actualState))
      {
        changeContext(actualState);
        context->request(distance);
        sendState();
      }
    }
    else
    {
      reconnectToServer();
    }

    delay(1000);
  }

  bool stateChanged(int actualState)
  {
    return actualState != context->getStateID() || actualState != lastSentState;
  }

  void reconnectToServer()
  {
    Serial.println("Intentando conectar al servidor...");
    if (client.connect(SERVER_HOST, SERVER_PORT))
    {
      Serial.println("Conectado al servidor TCP");
      registerDevice();
      getRanges();
    }
    else
    {
      Serial.println("Error al conectar al servidor TCP");
    }
  }

  void registerDevice()
  {
    if (client.connected())
    {
      client.print("REGISTER SENSOR");
    }
    else
    {
      Serial.println("Error al conectar con el servidor");
    }
  }

  void getRanges()
  {
    String response = sendCommand("GET RANGES");
    parseRanges(response);
  }

  String sendCommand(const String &command)
  {
    if (client.connected())
    {
      client.print(command);
      while (!client.available())
      {
        delay(100);
      }
      String response = client.readStringUntil('\n');
      Serial.println("Respuesta del servidor: " + response);
      return response;
    }
    else
    {
      Serial.println("Error: no conectado al servidor");
      return "";
    }
  }

  void parseRanges(const String &response)
  {
    size_t space_pos = response.indexOf(' ');

    stride = response.substring(0, space_pos).toInt();
    led_qty = response.substring(space_pos + 1).toInt();
  }

  int determineState(float distance)
  {
    return distance / stride;
  }

  void changeContext(int state)
  {
    if (state == 0)
    {
      context->transitionTo(new State0());
    }
    else if (state == 1)
    {
      context->transitionTo(new State1());
    }
    else if (state == 2)
    {
      context->transitionTo(new State2());
    }
    else
    {
      context->transitionTo(new State3());
    }
  }

  void sendState()
  {
    unsigned int state = context->getStateID();
    if (client.connected() && state != lastSentState)
    {
      String command = "PUT " + String(state);
      client.print(command);
      Serial.println("Comando enviado: " + command);
      lastSentState = state;
    }
    else if (!client.connected())
    {
      Serial.println("Error: No conectado al servidor");
    }
  }
};

SensorClient client("HUAWEI-2.4G-M6xZ", "HT7KU2Xv", 19, 22);

void setup()
{
  client.setup();
}

void loop()
{
  client.loop();
}
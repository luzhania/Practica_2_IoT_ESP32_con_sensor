#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <iostream>
#include <string>
#include <functional>

using namespace std;

class Utilities {
public:
  static void nonBlockingDelayFor(unsigned long microseconds) {
    unsigned long startTime = micros();
    while (micros() - startTime < microseconds)
      ;
  }

  static void serialPrintNonBlockingDelay(unsigned long milliseconds, unsigned int cm) {
    static unsigned long lastMeasurement = 0;
    unsigned long currentMillis = millis();

    if (currentMillis - lastMeasurement >= 100) {
      Serial.print(cm);
      Serial.println("cm");
      lastMeasurement = currentMillis;
    }
  }

  static void NonBlockingDelay(unsigned long milliseconds, std::function<void()> callback) {
    static unsigned long lastMeasurement = 0;
    unsigned long currentMillis = millis();

    if (currentMillis - lastMeasurement >= milliseconds) {
      callback();
      lastMeasurement = currentMillis;
    }
  }
};

class UltrasonicSensor {
private:
  unsigned int triggerPin;
  unsigned int echoPin;

public:
  UltrasonicSensor(unsigned int trigger, unsigned int echo) {
    triggerPin = trigger;
    echoPin = echo;
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
  }

  float readDistance() {
    digitalWrite(triggerPin, LOW);
    Utilities::nonBlockingDelayFor(2);
    digitalWrite(triggerPin, HIGH);
    Utilities::nonBlockingDelayFor(10);
    digitalWrite(triggerPin, LOW);
    return pulseIn(echoPin, HIGH);
  }

  float getDistanceInCM() {
    return 0.01723 * readDistance();
  }
};



class Context;
class State;

class State {
protected:
  Context *context;

public:
  virtual ~State() {}
  void set_context(Context *context) {
    this->context = context;
  }

  virtual void handleDistance() = 0;
  virtual unsigned int getStateID() = 0;
};

class State0 : public State {
public:
  void handleDistance() override {
    std::cout << "State 0: Encender 3 LEDs\n";
  }

  unsigned int getStateID() override {
    return 0;
  }
};

class State1 : public State {
public:
  void handleDistance() override {
    std::cout << "State 1: Encender 2 LEDs\n";
  }

  unsigned int getStateID() override {
    return 1;
  }
};

class State2 : public State {
public:
  void handleDistance() override {
    std::cout << "State 2: Encender 1 LED\n";
  }

  unsigned int getStateID() override {
    return 2;
  }
};

class State3 : public State {
public:
  void handleDistance() override {
    std::cout << "State 3: No encender LEDs\n";
  }

  unsigned int getStateID() override {
    return 3;
  }
};

class Context {
private:
  State *state;
  unsigned int stride;
  unsigned int led_qty;
  unsigned int lastSentState;

public:
  Context(State *state, unsigned int stride = 6, unsigned int led_qty = 3)
    : state(nullptr), stride(stride), led_qty(led_qty), lastSentState(led_qty + 1) {
    this->transitionTo(state);
  }

  ~Context() {
    delete state;
  }

  void transitionTo(State *state) {
    cout << "Context: Transition to state " << state->getStateID() << "\n";
    if (this->state != nullptr)
      delete this->state;
    this->state = state;
    this->state->set_context(this);
  }

  void request() {
    this->state->handleDistance();
  }

  unsigned int getStateID() {
    return this->state->getStateID();
  }

  void setStride(unsigned int newStride) {
    stride = newStride;
  }

  void setLedQty(unsigned int newLedQty) {
    led_qty = newLedQty;
  }

  unsigned int determineState(float distance) {
    unsigned int state = distance / stride;
    return (state <= led_qty) ? state : led_qty;
  }

  bool stateChanged(unsigned int &actualState) {
    return actualState != lastSentState;
  }

  void changeContext(unsigned int actualState) {
    lastSentState = actualState;

    if (actualState == 0) {
      transitionTo(new State0());
    } else if (actualState == 1) {
      transitionTo(new State1());
    } else if (actualState == 2) {
      transitionTo(new State2());
    } else {
      transitionTo(new State3());
    }
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

const char *SERVER_HOST = "192.168.100.11";
const unsigned int SERVER_PORT = 8080;

class SensorClient {
private:
  Context *context;
  WiFiClient client;
  WiFiConnection *wifiConnection;
  UltrasonicSensor *ultrasonicSensor;
  bool isConnected = false;

public:
  SensorClient(const char *SSID, const char *PASSWORD, unsigned int trigPin = 19, unsigned int echoPin = 22) {
    context = new Context(new State3);
    wifiConnection = new WiFiConnection(SSID, PASSWORD);
    ultrasonicSensor = new UltrasonicSensor(trigPin, echoPin);
  }

  ~SensorClient() {
    delete context;
    delete ultrasonicSensor;
    delete wifiConnection;
  }

  void setup() {
    Serial.begin(115200);
    wifiConnection->connect();
  }

  void loop() {
    if (!isConnected) {
      connectToServer();
    } else {
      sendDataToServer();
    }
  }

  void connectToServer() {
    if (!client.connected()) {
      if (client.connect(SERVER_HOST, SERVER_PORT)) {
        Serial.println("Conectado al servidor");

        client.print("REGISTER SENSOR");
        Utilities::NonBlockingDelay(500, [this]() {
          this->getRanges();  // Ahora se llama a getRanges correctamente
        });
        isConnected = true;
      } else {
        Serial.println("Error al conectar con el servidor");

        isConnected = false;
      }
    }
  }

  void getRanges() {
    String response = sendCommand("GET RANGES");
    parseRanges(response);
  }

  String sendCommand(const String &command) {
    if (client.connected()) {
      client.print(command);
      while (!client.available()) {
        Utilities::NonBlockingDelay(500, []() {
          ;
        });
      }
      String response = client.readStringUntil('\n');
      Serial.println("Respuesta del servidor: " + response);
      return response;
    } else {
      Serial.println("Error: no conectado al servidor");
      return "";
    }
  }

  void parseRanges(const String &response) {
    size_t space_pos = response.indexOf(' ');

    context->setStride(response.substring(0, space_pos).toInt());
    context->setLedQty(response.substring(space_pos + 1).toInt());
  }

  void sendDataToServer() {
    if (client.connected()) {
      float distance = ultrasonicSensor->getDistanceInCM();
      unsigned int actualState = context->determineState(distance);
      if (context->stateChanged(actualState)) {
        context->changeContext(actualState);
        context->request();
        sendState();
      }
    } else {
      Serial.println("Error: No conectado al servidor");
      isConnected = false;
      client.stop();
    }
  }

  void sendState() {
    unsigned int state = context->getStateID();
    String command = "PUT " + String(state);
    client.print(command);
    Serial.println("Comando enviado: " + command);
  }
};

SensorClient client("HUAWEI-2.4G-M6xZ", "HT7KU2Xv", 19, 22);

void setup() {
  client.setup();
}

void loop() {
  Utilities::NonBlockingDelay(500, []() {
    client.loop();
  });
}

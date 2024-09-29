#include <WiFi.h>
#include <WiFiClient.h>
#include <iostream>

// Wi-Fi Configuración
// Wi-Fi Configuración
const char *ssid = "HUAWEI-2.4G-M6xZ";  // Cambia esto por tu SSID
const char *password = "HT7KU2Xv";      // Cambia esto por tu contraseña

// Configuración del servidor TCP
const char *serverHost = "192.168.100.11";  // Cambia esto por la IP de tu servidor
const unsigned int serverPort = 8080;       // Cambia esto por el puerto del servidor

// Pines del sensor ultrasónico
const unsigned int trigPin = 19;
const unsigned int echoPin = 22;

// Prototipos de las clases para patrón State
class Context;
class State;

// Clase base para los estados
class State {
protected:
  Context *context_;

public:
  virtual ~State() {}
  void set_context(Context *context) {
    this->context_ = context;
  }

  virtual void handleDistance(float distance) = 0;
  virtual int getStateID() = 0;
};

// Clase Contexto que contiene el estado actual
class Context {
private:
  State *state_;

public:
  Context(State *state)
    : state_(nullptr) {
    this->transitionTo(state);
  }

  ~Context() {
    delete state_;
  }

  void transitionTo(State *state) {
    std::cout << "Context: Transition to state " << state->getStateID() << "\n";
    if (this->state_ != nullptr)
      delete this->state_;
    this->state_ = state;
    this->state_->set_context(this);
  }

  void request(float distance) {
    this->state_->handleDistance(distance);
  }

  int getStateID() {
    return this->state_->getStateID();
  }
};

// Estado 0: Encender 3 LEDs
class State0 : public State {
public:
  void handleDistance(float distance) override {
    std::cout << "State 0: Encender 3 LEDs\n";
  }

  int getStateID() override {
    return 0;
  }
};

// Estado 1: Encender 2 LEDs
class State1 : public State {
public:
  void handleDistance(float distance) override {
    std::cout << "State 1: Encender 2 LEDs\n";
  }

  int getStateID() override {
    return 1;
  }
};

// Estado 2: Encender 1 LED
class State2 : public State {
public:
  void handleDistance(float distance) override {
    std::cout << "State 2: Encender 1 LED\n";
  }

  int getStateID() override {
    return 2;
  }
};

// Estado 3: No encender LEDs
class State3 : public State {
public:
  void handleDistance(float distance) override {
    std::cout << "State 3: No encender LEDs\n";
  }

  int getStateID() override {
    return 3;
  }
};

// Clase principal para el ESP32
class ESP32Client {
private:
  Context *context;
  WiFiClient client;
  unsigned int stride;
  unsigned int led_qty;
  int lastSentState;  // Nueva variable para almacenar el último estado enviado

public:
  ESP32Client() {
    context = new Context(new State3);  // Iniciar en el estado 3 por defecto
    stride = 6;                         // Valores predeterminados
    led_qty = 3;                        // Valores predeterminados
    lastSentState = -1;                 // Inicializar con un valor inválido para indicar que no se ha enviado ningún estado
  }

  ~ESP32Client() {
    delete context;
  }

  void setup() {
    Serial.begin(115200);
    setupWiFi();
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
  }

  void setupWiFi() {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Conectando a WiFi...");
    }
    Serial.println("Conectado a WiFi");
  }

  void loop() {
    if (!client.connected()) {
      reconnectToServer();
    }

    if (client.connected()) {
      // Proceso normal del cliente si está conectado
      float distance = measureDistance();
      int actualState = determineState(distance);

      // Si el estado ha cambiado comparado con el último estado enviado
      if (actualState != context->getStateID() || actualState != lastSentState) {
        changeContext(actualState);
        context->request(distance);  // Ejecuta la acción para el nuevo estado
        sendState();                 // Enviar el estado actualizado al servidor
      }
    }

    delay(1000);  // Ajusta este delay según lo necesario
  }

  void reconnectToServer() {
    Serial.println("Intentando conectar al servidor...");
    if (client.connect(serverHost, serverPort)) {
      Serial.println("Conectado al servidor TCP");
      registerDevice();
      getRanges();
    } else {
      Serial.println("Error al conectar al servidor TCP");
    }
  }

  void registerDevice() {
    if (client.connected()) {
      client.print("REGISTER SENSOR");
    } else {
      Serial.println("Error al conectar con el servidor");
    }
  }

  void getRanges() {
    String response = sendCommand("GET RANGES");
    parseRanges(response);
  }

  String sendCommand(const String &command) {
    if (client.connected()) {
      client.print(command);  //////
      while (!client.available()) {
        delay(100);
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
    if (response.startsWith("{")) {
      // Parse "stride"
      int start = response.indexOf("\"stride\": ");
      if (start != -1) {
        start += 10;  // Mover el inicio después de "stride":
        int end = response.indexOf(",", start);
        stride = response.substring(start, end).toInt();
        std::cout << "stride: " << stride << std::endl;
      } else {
        std::cout << "Error: 'stride' no encontrado" << std::endl;
      }

      // Parse "LED_qty"
      start = response.indexOf("\"LED_qty\": ");
      if (start != -1) {
        start += 11;  // Mover el inicio después de "LED_qty":
        int end = response.indexOf("}", start);
        if (end == -1) {
          end = response.length();  // Por si es el último valor y no hay '}'.
        }
        led_qty = response.substring(start, end).toInt();
        std::cout << "LED_qty: " << led_qty << std::endl;
      } else {
        std::cout << "Error: 'LED_qty' no encontrado" << std::endl;
      }

      // Mostrar con printf
      Serial.printf("Recibido: stride = %d, LED_qty = %d\n", stride, led_qty);
    }
  }


  float measureDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    float distance = (duration * 0.0343) / 2;
    Serial.printf("Distancia medida: %.2f cm\n", distance);
    return distance;
  }

  int determineState(float distance) {
    return distance / stride;
  }

  void changeContext(int state) {
    if (state == 0) {
      context->transitionTo(new State0());
    } else if (state == 1) {
      context->transitionTo(new State1());
    } else if (state == 2) {
      context->transitionTo(new State2());
    } else {
      context->transitionTo(new State3());
    }
  }

  void sendState() {
    unsigned int state = context->getStateID();  // Obtener el estado actual del contexto
    if (client.connected() && state != lastSentState) {
      // Enviar el comando PUT al servidor con el estado actual
      String command = "PUT " + String(state);
      client.print(command + "\n");  // Asegúrate de enviar el carácter de nueva línea
      Serial.println("Comando enviado: " + command);

      lastSentState = state;  // Actualizar el último estado enviado
    } else if (!client.connected()) {
      Serial.println("Error: No conectado al servidor");
    }
  }
};

// Instancia del cliente ESP32
ESP32Client client;

void setup() {
  client.setup();
}

void loop() {
  client.loop();
}

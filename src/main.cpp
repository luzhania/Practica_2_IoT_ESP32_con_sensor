#include <WiFi.h>
#include <WiFiClient.h>
#include <iostream>

// Wi-Fi Configuración
const char* ssid = "HUAWEI-2.4G-M6xZ";       // Cambia esto por tu SSID
const char* password = "HT7KU2Xv"; // Cambia esto por tu contraseña

// Configuración del servidor TCP
const char* serverHost = "192.168.100.11"; // Cambia esto por la IP de tu servidor
const unsigned int serverPort = 8080;              // Cambia esto por el puerto del servidor

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
    Context(State *state) : state_(nullptr) {
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
    float stride;
    int led_qty;

public:
    ESP32Client() {
        context = new Context(new State3); // Iniciar en el estado 3 por defecto
        stride = 6;   // Valores predeterminados
        led_qty = 3;  // Valores predeterminados
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

    void loop() {
        if (!client.connected()) {
            reconnectToServer();
        }
        
        if (client.connected()) {
            // Proceso normal del cliente si está conectado
            float distance = measureDistance();
            determineState(distance);
            sendState();
        }

        delay(1000);
    }

    void setupWiFi() {
        WiFi.begin(ssid, password);
        while (WiFi.status() != WL_CONNECTED) {
            delay(1000);
            Serial.println("Conectando a WiFi...");
        }
        Serial.println("Conectado a WiFi");
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
        sendCommand("REGISTER");
    }

    void getRanges() {
        String response = sendCommand("GET RANGES");
        parseRanges(response);
    }

    String sendCommand(const String& command) {
        if (client.connected()) {
            client.print(command + "\n");
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

    void parseRanges(const String& response) {
        if (response.startsWith("{")) {
            int start = response.indexOf("stride:");
            int end = response.indexOf(",", start);
            stride = response.substring(start + 7, end).toFloat();

            start = response.indexOf("LEDs_qty:");
            end = response.indexOf("}", start);
            led_qty = response.substring(start + 9, end).toInt();

            Serial.printf("Recibido: stride = %.2f, LED_qty = %d\n", stride, led_qty);
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

    void determineState(float distance) {
        if (distance < stride) {
            context->transitionTo(new State0());
        } else if (distance < stride * 2) {
            context->transitionTo(new State1());
        } else if (distance < stride * 3) {
            context->transitionTo(new State2());
        } else {
            context->transitionTo(new State3());
        }
        context->request(distance);
    }

    void sendState() {
        int state = context->getStateID();
        sendCommand("PUT " + String(state));
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

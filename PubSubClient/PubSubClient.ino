#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
//*************** CONFIGURACIÓN DE RED ****************
const char *ssid = "Feri";
const char *passwd = "wififeri23";

//*************** CONFIGURACIÓN DE MQTT ****************
char *server = "192.168.51.92";
int port = 1883;
const char *mqtt_user = "cami21";
const char *mqtt_password = "cami21";

WiFiClient wlanclient;
PubSubClient mqttClient(wlanclient);

//*************** DEFINICIÓN DE TOPICS MQTT ****************
const char* topics[] = {
  "/sensor/temperature",
  "/sensor/humidity",
  "/sensor/light",
  "/sensor/gas",
  "/sensor/co",
  "/sensor/status"
};

//*************** PINES DE SENSORES ****************
// Pines Digitales: DHT11 y LED
#define DHTPIN D4
#define DHTTYPE DHT11
#define LED_PIN D3

// Pines MUX
#define S0 D5
#define S1 D6
#define S2 D7
#define S3 D8
#define SIG A0

DHT dht(DHTPIN, DHTTYPE);

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display


//*************** CONSTANTES Y VARIABLES ****************
const float TEMP_MIN = 18.0, TEMP_MAX = 24.0;
const float HUM_MIN = 30.0, HUM_MAX = 50.0;
const int LIGHT_MIN = 80, LIGHT_MAX = 400;
const int CO_MAX = 200;
const int GAS_SAFE_LEVEL = 50;

unsigned long lastMeasurementTime = 0;
const unsigned long measurementInterval = 5000; // Intervalo de medición en ms (5 segundos)
unsigned long displayTime = 2000; // Tiempo de visualización de cada dato en ms
unsigned long previousMillis = 0;

int displayStep = 0; // Variable para controlar la visualización de datos en la pantalla LCD

//*************** CONFIGURACIÓN INICIAL ****************
void setup() {
  Serial.begin(115200);

  // Configuración WiFi
  WiFi.begin(ssid, passwd);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");

  // Configuración MQTT
  mqttClient.setServer(server, port);
  mqttClient.setCallback(mqttCallback);

  // Inicializar sensores y periféricos
  dht.begin();
  pinMode(LED_PIN, OUTPUT);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  // lcd
  lcd.init();                      // initialize the lcd 
  lcd.init();
  // Print a message to the LCD.
  lcd.backlight();

  // Intentar conexión inicial a MQTT
  reconnect();
}

//*************** FUNCIÓN DE CONEXIÓN A MQTT ****************
void reconnect() {
  while (!mqttClient.connected()) {
    Serial.println();
    Serial.println("Trying to connect to the MQTT broker...");

    if (mqttClient.connect("ESP-Client", mqtt_user, mqtt_password)) {
      Serial.println("Connected to MQTT Broker");
      subscribeToTopics();  // Suscribirse a los temas nuevamente
    } else {
      Serial.print("Fallo, rc=");
      Serial.println(mqttClient.state());
      Serial.println("Trying to connect each 5 seconds");
      delay(5000); // Esperar 5 segundos antes de reintentar
    }
  }
}

//*************** FUNCIÓN DE SELECCIÓN DEL MUX ****************
int readMux(int channel) {
  digitalWrite(S0, bitRead(channel, 0));
  digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));
  digitalWrite(S3, bitRead(channel, 3));
  return analogRead(SIG);
}

//*************** FUNCIONES DE LECTURA DE SENSORES ****************
float readTemperature() {
  return dht.readTemperature();
}

float readHumidity() {
  return dht.readHumidity();
}

int readLightLevel() {
  return readMux(0); // Canal 0 en el MUX para el LDR
}

int readGasLevel() {
  return readMux(1); // Canal 1 en el MUX para el MQ-2
}

int readCOLevel() {
  return readMux(2); // Canal 2 en el MUX para el MQ-7
}

//*************** FUNCIÓN PARA PUBLICAR LOS DATOS ****************
void publishData(float temperature, float humidity, int lightLevel, int gasLevel, int coLevel, bool airQualityOptimal) {
  char tempString[8], humString[8], lightString[8], gasString[8], coString[8];

  dtostrf(temperature, 6, 2, tempString);
  dtostrf(humidity, 6, 2, humString);
  dtostrf(lightLevel, 6, 0, lightString);
  dtostrf(gasLevel, 6, 0, gasString);
  dtostrf(coLevel, 6, 0, coString);

  publishToTopic("/sensor/temperature", tempString);
  publishToTopic("/sensor/humidity", humString);
  publishToTopic("/sensor/light", lightString);
  publishToTopic("/sensor/gas", gasString);
  publishToTopic("/sensor/co", coString);
  publishToTopic("/sensor/status", airQualityOptimal ? "Calidad del aire óptima" : "Calidad del aire no óptima");
}

//*************** FUNCIÓN DE MONITOREO Y ALERTAS ****************
void monitorAirQuality() {
  float temperature = readTemperature();
  float humidity = readHumidity();
  int lightLevel = readLightLevel();
  int gasLevel = readGasLevel();
  int coLevel = readCOLevel();

  bool airQualityOptimal = 
      (temperature >= TEMP_MIN && temperature <= TEMP_MAX) &&
      (humidity >= HUM_MIN && humidity <= HUM_MAX) &&
      (lightLevel >= LIGHT_MIN && lightLevel <= LIGHT_MAX) &&
      (coLevel <= CO_MAX) && (gasLevel < GAS_SAFE_LEVEL);

  publishData(temperature, humidity, lightLevel, gasLevel, coLevel, airQualityOptimal);

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= displayTime) {
    previousMillis = currentMillis;
    lcd.clear();

    // Mostrar cada valor uno a uno en la LCD
    switch (displayStep) {
      case 0:
        lcd.setCursor(0, 0);
        lcd.print("Temp: ");
        lcd.print(temperature);
        lcd.print(" C");
        break;
      case 1:
        lcd.setCursor(0, 0);
        lcd.print("Hum: ");
        lcd.print(humidity);
        lcd.print(" %");
        break;
      case 2:
        lcd.setCursor(0, 0);
        lcd.print("Luz: ");
        lcd.print(lightLevel);
        break;
      case 3:
        lcd.setCursor(0, 0);
        lcd.print("Gas: ");
        lcd.print(gasLevel);
        break;
      case 4:
        lcd.setCursor(0, 0);
        lcd.print("CO: ");
        lcd.print(coLevel);
        break;
      case 5:
        lcd.setCursor(0, 0);
        lcd.print(airQualityOptimal ? "Calidad: OPTIMA" : "Calidad: NO OPTIMA");
        digitalWrite(LED_PIN, airQualityOptimal ? LOW : HIGH);
        break;
    }
    displayStep = (displayStep + 1) % 6; // Resetear el paso después de la última pantalla
  }
}

//*************** LOOP PRINCIPAL ****************
void loop() {
  mqttClient.loop();
  if (!mqttClient.connected()) {
    reconnect();
  }

  unsigned long currentMillis = millis();
  if (currentMillis - lastMeasurementTime >= measurementInterval) {
    lastMeasurementTime = currentMillis;
    monitorAirQuality();
  }
}

//*************** FUNCIÓN DE PUBLICACIÓN A MQTT ****************
boolean publishToTopic(const char *topic, const char *message) {
  return mqttClient.publish(topic, message);
}

//*************** FUNCIÓN DE CALLBACK DE MQTT ****************
void mqttCallback(char *topicChar, byte *payload, unsigned int length) {
  Serial.println();
  String topic = String(topicChar);
  Serial.print("Message arrived on Topic: ");
  Serial.println(topic);

  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  if (topic == "/sensor/status") {
    Serial.println(message);
  }
}

//*************** FUNCIÓN DE SUSCRIPCIÓN A TEMAS MQTT ****************
void subscribeToTopics() {
  const int numTopics = sizeof(topics) / sizeof(topics[0]);
  for (int i = 0; i < numTopics; i++) {
    mqttClient.subscribe(topics[i]);
  }
}

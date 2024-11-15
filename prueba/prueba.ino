#include <ESP8266WiFi.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

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
unsigned long lastDisplayTime = 0;
const unsigned long displayInterval = 2000; // Intervalo de visualización en LCD en ms (2 segundos)

int displayState = 0; // Estado para cambiar entre las medidas a mostrar

//*************** CONFIGURACIÓN INICIAL ****************
void setup() {
  Serial.begin(115200);

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
  return readMux(0); // Canal 0 en el MUX
}

int readGasLevel() {
  return readMux(1); // Canal 1 en el MUX
}

int readCOLevel() {
  return readMux(2); // Canal 2 en el MUX
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

  // Imprimir valores en el monitor serial
  Serial.println("===== Datos de Sensores =====");
  Serial.print("Temperatura: ");
  Serial.print(temperature);
  Serial.println(" C");

  Serial.print("Humedad: ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("Nivel de Luz: ");
  Serial.println(lightLevel);

  Serial.print("Nivel de Gas (MQ-2): ");
  Serial.println(gasLevel);

  Serial.print("Nivel de CO (MQ-7): ");
  Serial.println(coLevel);

  Serial.print("Estado de Calidad del Aire: ");
  Serial.println(airQualityOptimal ? "OPTIMA" : "NO OPTIMA");

  // Encender o apagar LED según el estado de calidad del aire
  //digitalWrite(LED_PIN, airQualityOptimal ? LOW : HIGH);

  Serial.println("=============================");

  // Mostrar datos en la LCD
  displayOnLCD(temperature, humidity, lightLevel, gasLevel, coLevel, airQualityOptimal);
}

//*************** FUNCIÓN PARA MOSTRAR EN LCD ****************
void displayOnLCD(float temperature, float humidity, int lightLevel, int gasLevel, int coLevel, bool airQualityOptimal) {
  unsigned long currentMillis = millis();
  if (currentMillis - lastDisplayTime >= displayInterval) {
    lastDisplayTime = currentMillis;
    lcd.clear();

    switch (displayState) {
      case 0:
        lcd.setCursor(0, 0);
        lcd.print("Temp: ");
        lcd.print(temperature);
        lcd.print(" C");
        break;
      case 1:
        lcd.setCursor(0, 0);
        lcd.print("Humedad: ");
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
        lcd.print("Gas (MQ-2): ");
        lcd.print(gasLevel);
        break;
      case 4:
        lcd.setCursor(0, 0);
        lcd.print("CO (MQ-7): ");
        lcd.print(coLevel);
        break;
      case 5:
        lcd.setCursor(0, 0);
        lcd.print("Calidad Aire:");
        lcd.setCursor(0, 1);
        lcd.print(airQualityOptimal ? "OPTIMA" : "NO OPTIMA");
        break;
    }

    displayState = (displayState + 1) % 6; // Cambiar al siguiente estado en el ciclo
  }
}

//*************** LOOP PRINCIPAL ****************
void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastMeasurementTime >= measurementInterval) {
    lastMeasurementTime = currentMillis;
    monitorAirQuality();
  }
}

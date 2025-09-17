/********************************************************************************
 * *
 * PROYECTO LABORATORIO I - ESTACIÓN DE MONITOREO AMBIENTAL              *
 * Código Firmware v1.6 (Versión Final Corregida)                        *
 * *
 * Integra: RTC, DHT22, LDR, BMP280, LCD y Módulo SD.                       *
 * Mejoras: Manejo de errores en display y optimización de memoria.       *
 * *
 ********************************************************************************/

// --- LIBRERÍAS ---
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <RTClib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SD.h>

// --- CONFIGURACIÓN DE HARDWARE Y PINES ---
#define SD_CS_PIN 10
#define DHT_PIN 9
#define DHT_TYPE DHT22
#define LCD_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2
#define LDR_PIN A0
#define LED_PIN 4

DHT dht(DHT_PIN, DHT_TYPE);
Adafruit_BMP280 bmp;
RTC_DS3231 rtc;
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

const float ALTITUD_CORDOBA = 400.0;
const int UMBRAL_ADC_ENCENDER = 300;
const int UMBRAL_ADC_APAGAR   = 350;
bool ledOn = false;

// --- VARIABLES GLOBALES ---
float temperatura = NAN, humedad = NAN, presion = NAN, lux = 0.0, lux_filtrado = 0.0;

// --- GESTIÓN DE TIEMPOS ---
unsigned long previousMillis_sensores = 0;
const long INTERVALO_LECTURA_SENSORES = 5000;
unsigned long previousMillis_pantalla = 0;
const long INTERVALO_ACTUALIZACION_LCD = 3000;

int estado_lcd = 0;
const int NUM_ESTADOS_LCD = 4;

// --- PROTOTIPOS DE FUNCIONES ---
void leerSensores();
void actualizarPantalla();
float filtradoExponencial(float valor_actual, float nueva_lectura, float alfa);
void inicializarSensores();
float calcularLuxCalibrado(int valorADC);
void registrarDatosEnSD();

// =============================================================================
// --- SETUP ---
// =============================================================================
void setup() {
  Serial.begin(9600);
  Serial.println("Iniciando Estacion de Monitoreo v1.6...");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  lcd.init();
  lcd.backlight();
  lcd.print("Inicializando...");
  delay(500);

  Serial.print("Iniciando SD...");
  lcd.setCursor(0,1);
  lcd.print("Iniciando SD...");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Error!");
    lcd.clear(); lcd.print("Error Tarjeta SD");
    while (1);
  }
  Serial.println("OK");
  lcd.clear();

  inicializarSensores();
  leerSensores();
  Serial.println("Sistema listo.");
}

// =============================================================================
// --- LOOP ---
// =============================================================================
void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis_sensores >= INTERVALO_LECTURA_SENSORES) {
    previousMillis_sensores = currentMillis;
    leerSensores();
    registrarDatosEnSD();
  }
  if (currentMillis - previousMillis_pantalla >= INTERVALO_ACTUALIZACION_LCD) {
    previousMillis_pantalla = currentMillis;
    actualizarPantalla();
  }
}

// =============================================================================
// --- IMPLEMENTACIÓN DE FUNCIONES ---
// =============================================================================

void inicializarSensores() {
  dht.begin();
  if (!rtc.begin()) {
    Serial.println("Error: RTC no encontrado."); lcd.setCursor(0,1); lcd.print("Error RTC!"); while (1);
  }
  if (!bmp.begin(0x76)) {
    Serial.println("Error: BMP280 no encontrado."); lcd.setCursor(0,1); lcd.print("Error BMP280!"); while (1);
  }
}

void leerSensores() {
  temperatura = dht.readTemperature();
  humedad = dht.readHumidity();
  float presion_abs = bmp.readPressure() / 100.0F;
  presion = bmp.seaLevelForAltitude(ALTITUD_CORDOBA, presion_abs);
  int ldr_raw = analogRead(LDR_PIN);
  lux = calcularLuxCalibrado(ldr_raw);
  lux_filtrado = filtradoExponencial(lux_filtrado, lux, 0.4);

  if (!ledOn && ldr_raw < UMBRAL_ADC_ENCENDER) {
    digitalWrite(LED_PIN, HIGH); ledOn = true; Serial.println("LED Encendido");
  } else if (ledOn && ldr_raw > UMBRAL_ADC_APAGAR) {
    digitalWrite(LED_PIN, LOW); ledOn = false; Serial.println("LED Apagado");
  }
}

void registrarDatosEnSD() {
  DateTime now = rtc.now();
  char nombreArchivo[12];
  snprintf(nombreArchivo, sizeof(nombreArchivo), "%02d%02d%02d.csv", now.day(), now.month(), now.year() % 100);
  File dataFile = SD.open(nombreArchivo, FILE_WRITE);
  if (dataFile) {
    if (dataFile.size() == 0) {
      dataFile.println("Fecha,Hora,Temperatura,Humedad,Presion,Lux");
    }
    char dataString[100];
    snprintf(dataString, sizeof(dataString),
             "%04d-%02d-%02d,%02d:%02d:%02d,%.1f,%.1f,%.1f,%.0f",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second(),
             temperatura, humedad, presion, lux_filtrado);
    dataFile.println(dataString);
    dataFile.close();
    Serial.print("Dato guardado en SD: "); Serial.println(dataString);
  } else {
    Serial.print("Error abriendo "); Serial.println(nombreArchivo);
  }
}

void actualizarPantalla() {
  lcd.clear();
  lcd.setCursor(0, 0);
  char buffer[17]; // Buffer para formatear texto para el LCD

  switch (estado_lcd) {
    case 0: { // Fecha y Hora
      DateTime now = rtc.now();
      snprintf(buffer, sizeof(buffer), "%02d/%02d/%04d", now.day(), now.month(), now.year());
      lcd.print(buffer);
      lcd.setCursor(0, 1);
      snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
      lcd.print(buffer);
      break;
    }
    case 1: { // Temperatura y Humedad
      if (isnan(temperatura)) {
        snprintf(buffer, sizeof(buffer), "Temp: -- C");
      } else {
        dtostrf(temperatura, 4, 1, buffer); // Convierte float a char array
        snprintf(buffer, sizeof(buffer), "Temp: %s C", buffer);
      }
      lcd.print(buffer);
      
      lcd.setCursor(0, 1);
      if (isnan(humedad)) {
        snprintf(buffer, sizeof(buffer), "Humedad: -- %%");
      } else {
        dtostrf(humedad, 4, 1, buffer);
        snprintf(buffer, sizeof(buffer), "Humedad: %s %%", buffer);
      }
      lcd.print(buffer);
      break;
    }
    case 2: { // Presión
      lcd.print("Presion:");
      lcd.setCursor(0, 1);
      if (isnan(presion)) {
        snprintf(buffer, sizeof(buffer), "-- hPa");
      } else {
        dtostrf(presion, 6, 1, buffer);
        snprintf(buffer, sizeof(buffer), "%s hPa", buffer);
      }
      lcd.print(buffer);
      break;
    }
    case 3: { // Iluminancia
      lcd.print("Iluminancia:");
      lcd.setCursor(0, 1);
      snprintf(buffer, sizeof(buffer), "%.0f lux", lux_filtrado);
      lcd.print(buffer);
      break;
    }
  }
  estado_lcd = (estado_lcd + 1) % NUM_ESTADOS_LCD;
}

float filtradoExponencial(float valor_actual, float nueva_lectura, float alfa) {
  if (valor_actual == 0.0) { return nueva_lectura; }
  return (alfa * nueva_lectura) + (1 - alfa) * valor_actual;
}

float calcularLuxCalibrado(int valorADC) {
  // --- ¡AQUÍ VA TU CALIBRACIÓN REAL! ---
  const int ADC_OSCURO = 90;
  const int ADC_LUZ    = 680;
  long lux = map(valorADC, ADC_OSCURO, ADC_LUZ, 10, 300);
  if (lux < 0) { lux = 0; }
  return (float)lux;
}

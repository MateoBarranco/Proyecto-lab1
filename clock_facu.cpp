/********************************************************************************
 * *
 * PROYECTO LABORATORIO I - ESTACIÓN DE MONITOREO AMBIENTAL              *
 * Código Firmware v1.2 (con control de LED mejorado)                   *
 * *
 * Integra: RTC, DHT22, LDR, BMP280 y LCD.                                  *
 * Incluye lógica de control de LED con histéresis aportada por el equipo. *
 * *
 ********************************************************************************/

// --- LIBRERÍAS ---
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <RTClib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// --- CONFIGURACIÓN DE HARDWARE Y PINES ---
// Sensor de Temperatura y Humedad
#define DHT_PIN 9
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);

// Sensor de Presión Barométrica
Adafruit_BMP280 bmp; 
const float ALTITUD_CORDOBA = 400.0;

// Reloj de Tiempo Real (RTC)
RTC_DS3231 rtc;

// Pantalla LCD I2C 16x2
#define LCD_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

// Sensor de Luz (LDR) y LED de control
#define LDR_PIN A0
#define LED_PIN 4
// --- Lógica de histéresis aportada por el equipo ---
const int UMBRAL_ADC_ENCENDER = 300; 
const int UMBRAL_ADC_APAGAR   = 350;
bool ledOn = false;
// ---

// --- VARIABLES GLOBALES PARA DATOS DE SENSORES ---
float temperatura = 0,0;
float humedad = 0,0;
float presion = 0,0;
float lux = 0.0;
float lux_filtrado = 0.0;

// --- GESTIÓN DE TIEMPOS (CICLO NO BLOQUEANTE) ---
unsigned long previousMillis_sensores = 0;
unsigned long previousMillis_pantalla = 0;
const long INTERVALO_LECTURA_SENSORES = 5000;
const long INTERVALO_ACTUALIZACION_LCD = 3000;

// Máquina de estados para la pantalla LCD
int estado_lcd = 0;
const int NUM_ESTADOS_LCD = 4;

// --- PROTOTIPOS DE FUNCIONES ---
void leerSensores();
void actualizarPantalla();
float filtradoExponencial(float valor_actual, float nueva_lectura, float alfa);
void inicializarSensores();

// =============================================================================
// --- FUNCIÓN DE SETUP ---
// =============================================================================
void setup() {
  Serial.begin(9600);
  Serial.println("Iniciando Estacion de Monitoreo v1.2...");

  // Inicialización de periféricos
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Aseguramos que el LED inicie apagado
  
  lcd.init();
  lcd.backlight();
  lcd.print("Inicializando...");
  delay(1000);

  inicializarSensores();

  leerSensores(); 
  Serial.println("Sistema listo.");
  lcd.clear();
}

// =============================================================================
// --- FUNCIÓN DE LOOP PRINCIPAL ---
// =============================================================================
void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis_sensores >= INTERVALO_LECTURA_SENSORES) {
    previousMillis_sensores = currentMillis;
    leerSensores();
  }

  if (currentMillis - previousMillis_pantalla >= INTERVALO_ACTUALIZACION_LCD) {
    previousMillis_pantalla = currentMillis;
    actualizarPantalla();
  }
}

// =============================================================================
// --- IMPLEMENTACIÓN DE FUNCIONES AUXILIARES ---
// =============================================================================

void inicializarSensores() {
  dht.begin();
  if (!rtc.begin()) {
    Serial.println("Error: RTC no encontrado."); lcd.setCursor(0,1); lcd.print("Error RTC!"); while (1);
  }
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  if (!bmp.begin(0x76)) {
    Serial.println("Error: BMP280 no encontrado."); lcd.setCursor(0,1); lcd.print("Error BMP280!"); while (1);
  }
}

void leerSensores() {
  // 1. Lectura DHT22
  temperatura = dht.readTemperature();
  humedad = dht.readHumidity();

  // 2. Lectura BMP280
  float presion_abs = bmp.readPressure() / 100.0F;
  presion = bmp.seaLevelForAltitude(ALTITUD_CORDOBA, presion_abs);

  // 3. Lectura LDR
  int ldr_raw = analogRead(LDR_PIN);
  lux = map(ldr_raw, 0, 1023, 0, 1000);
  lux_filtrado = filtradoExponencial(lux_filtrado, lux, 0.4);

  // 4. Lógica de control de LED con histéresis (Aportada por el equipo)
  if (!ledOn && ldr_raw < UMBRAL_ADC_ENCENDER) {
    digitalWrite(LED_PIN, HIGH);
    ledOn = true;
    Serial.println("LED Encendido");
  } else if (ledOn && ldr_raw > UMBRAL_ADC_APAGAR) {
    digitalWrite(LED_PIN, LOW);
    ledOn = false;
    Serial.println("LED Apagado");
  }
}

void actualizarPantalla() {
  lcd.clear();
  lcd.setCursor(0, 0);

  switch (estado_lcd) {
    case 0: { // Fecha y Hora
      DateTime now = rtc.now();
      char buffer_fecha[11];
      snprintf(buffer_fecha, sizeof(buffer_fecha), "%02d/%02d/%04d", now.day(), now.month(), now.year());
      lcd.print(buffer_fecha);
      lcd.setCursor(0, 1);
      char buffer_hora[9];
      snprintf(buffer_hora, sizeof(buffer_hora), "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
      lcd.print(buffer_hora);
      break;
    }
    case 1: { // Temperatura y Humedad
      lcd.print("Temp: " + String(temperatura, 1) + " C");
      lcd.setCursor(0, 1);
      lcd.print("Humedad: " + String(humedad, 1) + " %");
      break;
    }
    case 2: { // Presión
      lcd.print("Presion:");
      lcd.setCursor(0, 1);
      lcd.print(String(presion, 1) + " hPa");
      break;
    }
    case 3: { // Iluminancia
      lcd.print("Iluminancia:");
      lcd.setCursor(0, 1);
      lcd.print(String(lux_filtrado, 0) + " lux");
      break;
    }
  }
  estado_lcd = (estado_lcd + 1) % NUM_ESTADOS_LCD;
}

float filtradoExponencial(float valor_actual, float nueva_lectura, float alfa) {
  if (valor_actual == 0.0) {
    return nueva_lectura;
  }
  return (alfa * nueva_lectura) + (1 - alfa) * valor_actual;
}

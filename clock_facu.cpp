/********************************************************************************
 * *
 * PROYECTO LABORATORIO I - ESTACIÓN DE MONITOREO AMBIENTAL              *
 * Código Firmware Unificado v1.0                          *
 * *
 * Integra: RTC, DHT22, LDR, BMP280 y LCD en una arquitectura no bloqueante.  *
 * Autor: PROYECTO LAB1 (Guía Gemini)                                        *
 * Fecha: 03/09/2025                                                         *
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

// Sensor de Temperatura y Humedad (Requisito 4)
#define DHT_PIN 9
#define DHT_TYPE DHT22 // Actualizado a DHT22 según especificaciones del proyecto
DHT dht(DHT_PIN, DHT_TYPE);

// Sensor de Presión Barométrica y Temperatura
Adafruit_BMP280 bmp; 
const float ALTITUD_CORDOBA = 400.0; // Altitud aprox. para calibrar la presión a nivel del mar

// Sensor de Luz (Fotorresistor) (Requisito 5)
#define LDR_PIN A0

// Reloj de Tiempo Real (RTC) (Requisito 6)
RTC_DS3231 rtc;

// Pantalla LCD I2C 16x2 (Requisito 12)
#define LCD_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

// LED de estado (vinculado al LDR)
#define LED_PIN 4
const int UMBRAL_LUZ = 500; // Umbral para encender el LED

// --- VARIABLES GLOBALES PARA DATOS DE SENSORES ---
float temperatura = NAN;
float humedad = NAN;
float presion = NAN;
float lux = 0.0;
float lux_filtrado = 0.0; // Variable para el valor filtrado

// --- GESTIÓN DE TIEMPOS (CICLO NO BLOQUEANTE) ---
unsigned long previousMillis_sensores = 0;
unsigned long previousMillis_pantalla = 0;

// Ciclo de muestreo configurable (Requisito 8)
const long INTERVALO_LECTURA_SENSORES = 5000;  // Leer sensores cada 5 segundos
const long INTERVALO_ACTUALIZACION_LCD = 3000; // Rotar la pantalla cada 3 segundos

// Máquina de estados para la pantalla LCD
int estado_lcd = 0;
const int NUM_ESTADOS_LCD = 4; // 0:Fecha/Hora, 1:Temp/Hum, 2:Presión, 3:Luz

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
  Serial.println("Iniciando Estacion de Monitoreo Ambiental...");

  // Inicialización de periféricos
  pinMode(LED_PIN, OUTPUT);
  
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Inicializando...");
  delay(1000);

  // Llamada a la función que inicializa todos los sensores
  inicializarSensores();

  // Tomar una lectura inicial para no mostrar valores vacíos
  leerSensores(); 
  
  Serial.println("Sistema listo.");
  lcd.clear();
}

// =============================================================================
// --- FUNCIÓN DE LOOP PRINCIPAL ---
// =============================================================================
void loop() {
  unsigned long currentMillis = millis();

  // TAREA 1: Leer todos los sensores a un intervalo fijo
  if (currentMillis - previousMillis_sensores >= INTERVALO_LECTURA_SENSORES) {
    previousMillis_sensores = currentMillis;
    leerSensores();
  }

  // TAREA 2: Actualizar la pantalla a otro intervalo
  if (currentMillis - previousMillis_pantalla >= INTERVALO_ACTUALIZACION_LCD) {
    previousMillis_pantalla = currentMillis;
    actualizarPantalla();
  }

  // Futura implementación:
  // - Comprobar si se ha pulsado el botón de marcador de eventos (Requisito 13)
  // - Entrar en modo de bajo consumo (sleep) si es necesario (Requisito 23)
}

// =============================================================================
// --- IMPLEMENTACIÓN DE FUNCIONES AUXILIARES ---
// =============================================================================

/**
 * @brief Inicializa todos los sensores y verifica su conexión.
 */
void inicializarSensores() {
  // Iniciar DHT22
  dht.begin();

  // Iniciar RTC
  if (!rtc.begin()) {
    Serial.println("Error: No se encuentra el RTC.");
    lcd.setCursor(0,1);
    lcd.print("Error RTC!");
    while (1);
  }
  // Descomentar esta línea UNA SOLA VEZ para ajustar la hora y luego volver a comentarla.
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // Iniciar BMP280
  if (!bmp.begin(0x76)) { // La dirección I2C puede ser 0x76 o 0x77
    Serial.println("Error: No se encuentra el BMP280.");
    lcd.setCursor(0,1);
    lcd.print("Error BMP280!");
    while (1);
  }
}

/**
 * @brief Lee los valores de todos los sensores y los almacena en variables globales.
 */
void leerSensores() {
  // 1. Leer Temperatura y Humedad del DHT22
  temperatura = dht.readTemperature();
  humedad = dht.readHumidity();

  if (isnan(temperatura) || isnan(humedad)) {
    Serial.println("Error en la lectura del DHT22");
  } else {
    Serial.print("DHT -> Temp: " + String(temperatura) + " C, Hum: " + String(humedad) + "% | ");
  }

  // 2. Leer Presión y Temperatura del BMP280
  float temp_bmp = bmp.readTemperature();
  float presion_abs = bmp.readPressure() / 100.0F; // Convertir de Pa a hPa
  presion = bmp.seaLevelForAltitude(ALTITUD_CORDOBA, presion_abs);

  Serial.print("BMP -> Presion: " + String(presion) + " hPa | ");

  // 3. Leer nivel de luz del LDR
  int ldr_raw = analogRead(LDR_PIN);
  lux = map(ldr_raw, 0, 1023, 0, 1000); // Mapeo simple a Lux (se puede mejorar con calibración)
  
  // Aplicar filtro exponencial (Requisito 9)
  lux_filtrado = filtradoExponencial(lux_filtrado, lux, 0.4);

  Serial.println("LDR -> Lux (filtrado): " + String(lux_filtrado));
  
  // Controlar LED de estado
  if (ldr_raw < UMBRAL_LUZ) { // Menor valor analógico = más luz
      digitalWrite(LED_PIN, HIGH); // Encender LED si hay luz
  } else {
      digitalWrite(LED_PIN, LOW);
  }
}

/**
 * @brief Actualiza la pantalla LCD según el estado actual.
 */
void actualizarPantalla() {
  lcd.clear();
  lcd.setCursor(0, 0);

  switch (estado_lcd) {
    case 0: { // Pantalla 1: Fecha y Hora
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
    case 1: { // Pantalla 2: Temperatura y Humedad
      lcd.print("Temp: " + String(temperatura, 1) + " C");
      lcd.setCursor(0, 1);
      lcd.print("Humedad: " + String(humedad, 1) + " %");
      break;
    }
    case 2: { // Pantalla 3: Presión Atmosférica
      lcd.print("Presion:");
      lcd.setCursor(0, 1);
      lcd.print(String(presion, 1) + " hPa");
      break;
    }
    case 3: { // Pantalla 4: Nivel de Iluminancia
      lcd.print("Iluminancia:");
      lcd.setCursor(0, 1);
      lcd.print(String(lux_filtrado, 0) + " lux");
      break;
    }
  }

  // Avanzar a la siguiente pantalla para la próxima actualización
  estado_lcd = (estado_lcd + 1) % NUM_ESTADOS_LCD;
}

/**
 * @brief Aplica un filtro exponencial para suavizar una señal.
 * @param valor_actual El último valor filtrado.
 * @param nueva_lectura El nuevo valor leído del sensor.
 * @param alfa El factor de suavizado (0 < alfa < 1). Un valor más bajo suaviza más.
 * @return El nuevo valor filtrado.
 */
float filtradoExponencial(float valor_actual, float nueva_lectura, float alfa) {
  // Si es la primera lectura, el valor filtrado es igual al leído
  if (valor_actual == 0.0) {
    return nueva_lectura;
  }
  return (alfa * nueva_lectura) + (1 - alfa) * valor_actual;
}
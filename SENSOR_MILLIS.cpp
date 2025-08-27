#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// --- CONFIGURACIÓN DE SENSORES Y PERIFÉRICOS ---
// Configuración del sensor DHT11
#define DHTPIN 9
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Configuración del sensor BMP280
Adafruit_BMP280 bmp;
const float ALTITUD_LOCAL = 250.0;

// Configuración de la pantalla LCD I2C
#define LCD_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

// --- GESTIÓN DE TIEMPOS ---
unsigned long previousMillis_dht = 0;
unsigned long previousMillis_bmp = 0;
unsigned long previousMillis_lcd = 0;
const long interval_dht = 2000; // Intervalo de lectura del DHT11 (mínimo 1000ms, usamos 2000 para seguridad)
const long interval_bmp = 2000; // Intervalo de lectura del BMP280
const long interval_lcd = 3000; // Intervalo de rotación del LCD

int lcd_state = 0; // 0: DHT, 1: BMP (Presión), 2: BMP (Temp)

// --- SETUP: INICIALIZACIÓN DEL SISTEMA ---
void setup() {
  Serial.begin(9600);
  
  // Inicialización del LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Iniciando...");
  delay(1500);
  lcd.clear();

  // Inicialización de sensores
  dht.begin();
  if (!bmp.begin(0x76)) {
    Serial.println("Error: BMP280 no detectado.");
    lcd.clear();
    lcd.print("Error BMP280");
    while (1);
  }
  Serial.println("Sensores inicializados.");
}

// --- LOOP: CICLO PRINCIPAL NO BLOQUEANTE ---
void loop() {
  unsigned long currentMillis = millis();

  // TAREA 1: Lectura del DHT11
  if (currentMillis - previousMillis_dht >= interval_dht) {
    previousMillis_dht = currentMillis;
    // Lógica de lectura y validación del DHT11
    float dht_temp = dht.readTemperature();
    float dht_hum = dht.readHumidity();
    if (isnan(dht_temp) || isnan(dht_hum)) {
      Serial.println("Error: Lectura del DHT11 fallida.");
    } else {
      Serial.print("DHT Temp: "); Serial.print(dht_temp); Serial.print("C | Hum: "); Serial.println(dht_hum);
    }
  }

  // TAREA 2: Lectura del BMP280
  if (currentMillis - previousMillis_bmp >= interval_bmp) {
    previousMillis_bmp = currentMillis;
    // Lógica de lectura y validación del BMP280
    float bmp_temp = bmp.readTemperature();
    float presion_absoluta = bmp.readPressure() / 100.0;
    float presion_nivel_mar = presion_absoluta / pow(1.0 - (ALTITUD_LOCAL / 44330.0), 5.255);
    Serial.print("BMP Temp: "); Serial.print(bmp_temp); Serial.print("C | Presion: "); Serial.print(presion_nivel_mar); Serial.println("hPa");
  }

  // TAREA 3: Rotación y visualización del LCD
  if (currentMillis - previousMillis_lcd >= interval_lcd) {
    previousMillis_lcd = currentMillis;

    lcd.clear();
    lcd.setCursor(0, 0);

    // Máquina de estados para la rotación del LCD
    switch (lcd_state) {
      case 0:
        // Muestra DHT
        lcd.print("Temp:");
        lcd.print(dht.readTemperature());
        lcd.print("C");
        lcd.setCursor(0, 1);
        lcd.print("Humedad:");
        lcd.print(dht.readHumidity());
        lcd.print("%");
        lcd_state = 1;
        break;
      case 1:
        // Muestra Presión
        lcd.print("Presion:");
        lcd.setCursor(0, 1);
        lcd.print(bmp.readPressure() / 100.0);
        lcd.print(" hPa");
        lcd_state = 2;
        break;
      case 2:
        // Muestra Temperatura del BMP (para validación)
        lcd.print("Temp BMP:");
        lcd.setCursor(0, 1);
        lcd.print(bmp.readTemperature());
        lcd.print(" C");
        lcd_state = 0; // Regresa al inicio
        break;
    }
  }

 
}
SENSOR DE TEMPERATURA  DHT22 FUNCIONAL CON INES ESPECIFICOS 


#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// Configuración del DHT22
#define DHTPIN 9       // Pin digital donde está conectado el DHT22
#define DHTTYPE DHT22  // Tipo de sensor

DHT dht(DHTPIN, DHTTYPE);

// Configuración del LCD I2C
#define LCD_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2

LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

void setup() {
  Serial.begin(9600);
  dht.begin();

  lcd.init();         // Inicializa el LCD
  lcd.backlight();    // Enciende la luz de fondo
  lcd.clear();        // Limpia la pantalla
  lcd.print("Iniciando...");
  delay(1500);
  lcd.clear();
}

void loop() {
  delay(2000);  // Espera entre lecturas (DHT22 necesita ~2 segundos)

  float temp = dht.readTemperature();
  float hum = dht.readHumidity();

  // Verifica si las lecturas son válidas
  if (isnan(temp) || isnan(hum)) {
    Serial.println("Error al leer el DHT22");
    lcd.clear();
    lcd.print("Error lectura");
    return;
  }

  // Mostrar en Serial Monitor
  Serial.print("Temperatura: ");
  Serial.print(temp);
  Serial.println(" °C");

  Serial.print("Humedad: ");
  Serial.print(hum);
  Serial.println(" %");

  // Mostrar en LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temp);
  lcd.print(" C");

  lcd.setCursor(0, 1);
  lcd.print("Humedad: ");
  lcd.print(hum);
  lcd.print(" %");

  delay(3000); // Tiempo para leer en pantalla
}
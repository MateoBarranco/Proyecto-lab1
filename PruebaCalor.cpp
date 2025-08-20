#include <DHT.h>
#include <LiquidCrystal_I2C.h>

#define DHTPIN 2
#define DHTTYPE DHT22

const int LCD_COLS = 16;
const int LCD_ROWS = 2;      // podemos cambiarlo por un valor mas alto ---> averiguar que es mejor. 
const int LCD_ADDR = 0x27; 

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

void setup() {              // se ejecuta una sola vez al inicio del programa
  Serial.begin(9600);       // nicia la comunicación serial
  Serial.println("Iniciando...");

  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcd.print("Iniciando LCD...");
  delay(1500);
  lcd.clear();

  dht.begin(); // ---> Le dice al microcontrolador que inicie la comunicación con el sensor DHT22.
// lcd.init(), lcd.backlight() Funciones del lcd que inicializan, encienden y limpian.

}


void loop() {
  delay(2000);        // ausa el programa por 2 segundos. Esto es necesario para que el sensor DHT22 tenga tiempo de estabilizarse entre lecturas.

  float humedad = dht.readHumidity();
  float temperatura = dht.readTemperature();

                       //dht.readHumidity dht.readTemperature leen los datos del sensor y los almacenan

  if (isnan(humedad) || isnan(temperatura)) {
    
    return;
  }

 
  delay(3000);

 
  delay(3000);


}

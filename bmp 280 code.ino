#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#define FLASH_BTN 0
Adafruit_BMP280 bmp; // I2C interface
float off = 91067.20 ;
void setup() {
  Serial.begin(115200);
  Serial.println(F("BMP280 Pressure & Temperature Test"));
  pinMode(FLASH_BTN, INPUT_PULLUP);
  // Try both possible I2C addresses
  if (!bmp.begin(0x76) && !bmp.begin(0x77)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  // Optional: set sampling parameters
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,   // temperature
    Adafruit_BMP280::SAMPLING_X16,  // pressure
    Adafruit_BMP280::FILTER_X16,
    Adafruit_BMP280::STANDBY_MS_500
  );
}

void loop() {
  float temperature = bmp.readTemperature();     // °C
  float pressure_Pa = bmp.readPressure();        // Pa
  float pressure_kPa = pressure_Pa / 1000.0;     // kPa
  if (digitalRead(FLASH_BTN) == LOW) {
    off = pressure_kPa *1000 ;   // simple debounce
  }
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  Serial.print("Pressure: ");
  Serial.print(pressure_Pa);
  Serial.println(" Pa");

  Serial.print("Pressure: ");
  Serial.print(pressure_kPa);
  Serial.println(" kPa");
  Serial.print("offset: ");
  Serial.print(off);
  Serial.println(" Pa");
  Serial.print("speed: ");
  Serial.print(sqrt(2*(pressure_Pa - off)));
  Serial.println("m/s");
  Serial.println();
  delay(100);
}

#include <Wire.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;  // I2C
#define DOUT 14   // GPIO12 = D6 on NodeMCU
#define SCK  13 
  // GPIO14 = D5 on NodeMCU
float spee =0.314159;
float ambpres = 90.9;
float off = 76.24;
void setup() {
  Serial.begin(115200);
  pinMode(DOUT, INPUT);
  pinMode(SCK, OUTPUT);
  digitalWrite(SCK, LOW);
  pinMode(FLASH_BTN, INPUT_PULLUP);
  Serial.println("HX710B Basic Test Start");
  
  Serial.println(F("BMP280 test starting..."));

  if (!bmp.begin(0x76)) {   // Most BMP280 boards use I2C address 0x76
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X16,
    Adafruit_BMP280::FILTER_X16,
    Adafruit_BMP280::STANDBY_MS_500);
}
long readHX710B() {
  while (digitalRead(DOUT));  // Wait until DOUT goes LOW

  long count = 0;
  for (int i = 0; i < 24; i++) {
    digitalWrite(SCK, HIGH);
    delayMicroseconds(1);
    count = count << 1;
    digitalWrite(SCK, LOW);
    delayMicroseconds(1);
    if (digitalRead(DOUT)) count++;
  }

  // 25th pulse to finish conversion cycle
  digitalWrite(SCK, HIGH);
  delayMicroseconds(1);
  digitalWrite(SCK, LOW);
  delayMicroseconds(1);

  // Convert from 24-bit two's complement
  if (count & 0x800000) count |= ~0xFFFFFF;

  return count;
}
void loop() {

  float temperature = bmp.readTemperature();  // °C
  float pressurePa = bmp.readPressure();      // Pa
  float pressurekPa = pressurePa / 1000.0;    // kPa
  long value = readHX710B();

  float y= ((((float)(value +8388608)/16777215)*40)+off) ; // banglore pressure is 91.5 and minnimum is that by 1.25 apprx
  if(y>ambpres){
     spee = sqrt((float)(y - ambpres)*2*1000) ;
    }
  else{
     spee =0;
  }
   // assume density as 1 kg/m3  cause at banglore its 1.03 and sea level it is 1.225  
  Serial.print(value);
  Serial.print("  ");
  Serial.print(spee);
  Serial.print("  ");
  Serial.print(ambpres);
  Serial.print("  ");
  Serial.print(off);
  Serial.print("  ");
  Serial.println(y);

  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" °C");

  Serial.print("Pressure = ");
  Serial.print(pressurePa);
  Serial.print(" Pa  (");
  Serial.print(pressurekPa);
  Serial.println(" kPa)");

  Serial.println();
  if (digitalRead(FLASH_BTN) == LOW) {
    ambpres= pressurekPa ;
    off = (pressurekPa - (((float)(value +8388608)/16777215)*40)) ;
    Serial.println(off);
    delay(300);  // simple debounce
  }

}

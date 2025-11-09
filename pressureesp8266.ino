// Basic HX710B test with ESP8266
// Reads raw 24-bit output and prints to Serial Monitor

#define DOUT 14   // GPIO12 = D6 on NodeMCU
#define SCK  13   // GPIO14 = D5 on NodeMCU


#define FLASH_BTN 0
float spee =0.314159;
float off = -2219613;
void setup() {
  Serial.begin(115200);
  pinMode(DOUT, INPUT);
  pinMode(SCK, OUTPUT);
  digitalWrite(SCK, LOW);
  delay(500);
  Serial.println("HX710B Basic Test Start");
  pinMode(FLASH_BTN, INPUT_PULLUP);
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
  long value = readHX710B();

  float y= ((((float)(value +8388608)/16777215)*40)+76.8) ; // banglore pressure is 91.5 and minnimum is that by 1.25 apprx
  if (digitalRead(FLASH_BTN) == LOW) {
    off = value;  // simple debounce
  }
  if(value > off){
     spee = sqrt((((float)(value - off)/16777215)*40)*2*1000) ;
    }
  else{
     spee =0;
  }
   // assume density as 1 kg/m3  cause at banglore its 1.03 and sea level it is 1.225  
  Serial.print(value);
  Serial.print("  ");
  Serial.print(spee);
  Serial.print("  ");
  Serial.println(off);
  delay(500);
}

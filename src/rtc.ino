#include <Wire.h>
//#include <Serial>
#include <String.h>
//#include <LuquidCrystal.h>


uint8_t const clockAddress = 0x68; // 1101000

bool checkReset();

void show(unsigned long);


void setup() {
  // We need an input pin to listen to the -RESET from the clock
  pinMode(A0, INPUT_PULLUP);
  for(int i =  0; i <= 13; ++i) {
     pinMode(i, OUTPUT);
     digitalWrite(i, LOW);
  }

  // Wait for RESET to clear
  while( checkReset() ) {
    delay(200);
  }

  Wire.begin();
  // Set up by writing to the RTC
  Wire.beginTransmission(clockAddress);
  for(int i = 0; i < 4; ++i)
    Wire.write(0);        // counter registers
  Wire.write(0);          // EOSC (bit 7) off -> counting
  Wire.write(0);          // Disable trickle charge
  Wire.endTransmission();
  delay(200);
}

void loop() {
  // Read value from RTC
  unsigned long val;
  if( !checkReset() ) {
    Wire.requestFrom(clockAddress, 4);
    while( Wire.available() ) {
      unsigned char b = Wire.read();
      val <<= 8;
      val |= b;
    }
    char buf[8];
    int g = snprintf(buf, 8, "%d\n", val);
    Serial.write(buf);
    show(buf);
  }
  delay(5000);
}


void
show( unsigned long x )
{
  for(int i = 0; i<13; ++i) {
    digitalWrite(i, (x & 1) ? HIGH : LOW);
    x >>= 1;
  }
}


bool
checkReset()
{
  // A HIGH signal is about 3.3 V, or around an input value of 676
  // Also, the -RESET signal we are reading is Active Low
  if( analogRead(0) > 500 ) {
    digitalWrite(13, HIGH);       // No reset - Board LED on
    return false;
  } else {
    digitalWrite(13, LOW);        // Reset asserted - board LED off
    return true;
  }
}

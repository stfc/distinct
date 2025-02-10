#include <EEPROM.h>

typedef union { float f; unsigned long u; } floatlong;

void hexd(int k)
{
  k &= 0xF;
  Serial.print("0123456789ABCDEF"[k]);
}

void hexb(int k)
{
  hexd(k >> 4);
  hexd(k);
}

void hexa(byte a[4])
{
  hexb(a[0]);
  hexb(a[1]);
  hexb(a[2]);
  hexb(a[3]);
  Serial.println("");
}


void
setup()
{
  Serial.begin(115200);
  const unsigned long int magic = 0xD15712C7UL;
  floatlong m;
  m.u = 0x803F00D5UL;
  Serial.println(m.f);
  m.u = 0x17803FFFUL;
  Serial.println(m.f);
  float *a, b = 0.01;
  float *c, d;
  a = &b;
  c = &d;
  EEPROM.put(12, *a);
  EEPROM.get(12, m.f);
  EEPROM.get(12, *c);
  Serial.println(m.f);
  Serial.println(d);
  hexa(m.u);
}

void loop() {

}

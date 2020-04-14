#include "Oasis_RarceD.h"

void softReset()
{
  wdt_enable(WDTO_15MS);
  while (1)
    ;
}

void ledBlink(uint8_t pin, long milli)
{
  digitalWrite(pin, HIGH);
  delay(milli);
  digitalWrite(pin, LOW);
}

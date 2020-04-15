#include "Oasis_RarceD.h"
#include <stdint.h>


void ledBlink(uint8_t pin, long milli)
{
  digitalWrite(pin, 1);
  delay(milli);
  digitalWrite(pin, 0);
}

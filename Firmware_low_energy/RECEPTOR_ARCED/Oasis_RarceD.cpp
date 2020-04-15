#include "Oasis_RarceD.h"
#include <stdint.h>


void ledBlinkk(uint8_t pin, uint64_t milli)
{
 int a = 12;
}

int freeRam()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}
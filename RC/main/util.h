#ifndef UTIL_H
#define UTIL_H

#define LN_SCALE_FACTOR -3.0
#define MAX_DUTY        0.95

#include <stdint.h>

/* Macros */
#define _digitalWrite(pin, val)   digitalWrite(pin, val)
#define _enableInterrupts()     sei()
#define _disableInterrupts()    cli()

/* Function prototypes */
uint8_t speedMapToInt(uint8_t input, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max, bool dir = true);

#endif // !UTIL_H
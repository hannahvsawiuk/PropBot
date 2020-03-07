#ifndef UTIL_H
#define UTIL_H

#define LN_SCALE_FACTOR -3.0
#define MAX_DUTY        0.95

/* Macros */
#define _digitalWrite(pin, val)   digitalWrite(pin, val)
#define _enableInterrupts()     sei()
#define _disableInterrupts()    cli()

/* Function prototypes */
float speedMapToFloat(float input, float in_min, float in_max, float out_min, float out_max, bool dir = true);

#endif // !UTIL_H
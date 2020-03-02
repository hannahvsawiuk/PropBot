#ifndef UTIL_H
#define UTIL_H

#include <math.h>

#define LN_SCALE_FACTOR -3.0
#define MAX_DUTY        0.95

/* Macros */
#define _digitalWrite(pin, val)   digitalWrite(pin, val)
#define _enableInterrupts()     sei()
#define _disableInterrupts()    cli()

/**
 * @brief Returns the input linearly mapped to [0, 1] based on the provided I/O minima and maxima.
 * 
 * @param x         input value
 * @param in_min    input minimum
 * @param in_max    input maximum
 * @param out_min   output minimum
 * @param out_max   output maximum
 * @return float    mapped value
 */
float linMapToFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return abs((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}


/**
 * @brief Returns the input logarithmically mapped to [0, 1]: -ln(1-x)/LN_SCALE_FACTOR
 * 
 * @param x         input value
 * @param in_min    input minimum
 * @param in_max    input maximum
 * @param out_min   output minimum
 * @param out_max   output maximum
 * @param bw        backwards flag. Inverts the value before exponentially scaling
 * @return float    mapped value
 */
float logMapToFloat(float x, float in_min, float in_max, float out_min, float out_max, bool bw = false)
{
    if (bw == true) {
        return min(
            - log(linMapToFloat(x, in_min, in_max, out_min, out_max)) / LN_SCALE_FACTOR,
            MAX_DUTY
        );

    }
    return min(
        - log(1 - linMapToFloat(x, in_min, in_max, out_min, out_max)) / LN_SCALE_FACTOR,
        MAX_DUTY
    );
}    

#endif // !UTIL_H
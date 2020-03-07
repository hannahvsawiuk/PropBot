#include "util.h"
#include <math.h>

/**
 * @brief Returns the input linearly mapped to [0, 1] based on the provided I/O minima and maxima.
 * 
 * @param input     input value
 * @param in_min    input minimum
 * @param in_max    input maximum
 * @param out_min   output minimum
 * @param out_max   output maximum
 * @param dir       direction flag. Forward = true, backward = false.
 * @return float    mapped value
 */
float linMapToFloat(float input, float in_min, float in_max, float out_min, float out_max, bool dir = true)
{
    if (dir)
        return fabs((input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
    return 1 - fabs((input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}
 
#ifdef LOG_SPEED
    /**
     * @brief Returns the input logarithmically mapped to [0, 1]: -ln(1-x)/LN_SCALE_FACTOR
     * 
     * @param input     input value
     * @param in_min    input minimum
     * @param in_max    input maximum
     * @param out_min   output minimum
     * @param out_max   output maximum
     * @param dir       direction flag. Forward = true, backward = false. Inverts the value before exponentially scaling
     * @return float    mapped value
     */
    float speedMapToFloat(float input, float in_min, float in_max, float out_min, float out_max, bool dir = true) {
        if (dir) {
            return min(
                - log(1 - linMapToFloat(input, in_min, in_max, out_min, out_max)) / LN_SCALE_FACTOR,
                MAX_DUTY
            );
        }
        return min(
            - log(linMapToFloat(input, in_min, in_max, out_min, out_max)) / LN_SCALE_FACTOR,
            MAX_DUTY
        );
    }
#else
    /**
     * @brief Returns the input linearly mapped to [0, 1] based on the provided I/O minima and maxima.
     * 
     * @param input     input value
     * @param in_min    input minimum
     * @param in_max    input maximum
     * @param out_min   output minimum
     * @param out_max   output maximum
     * @param dir       direction flag. Forward = true, backward = false.
     * @return float    mapped value
     */
    float speedMapToFloat(float input, float in_min, float in_max, float out_min, float out_max, bool dir = true) {
        if (dir) {
            return fabs((input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
        }
        return 1 - fabs((input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
    }
#endif // LOG_SPEED

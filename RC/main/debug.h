
/*
debug.h - More advanced debug logging

To use, define DEBUG before including this file
*/

#ifndef DEBUGUTILS_H
#define DEBUGUTILS_H


#ifdef DEBUG
#include <Arduino.h>
#define DEBUG_PRINT(str)                 \
    Serial.print(millis());              \
    Serial.print(": ");                  \
    Serial.print(__PRETTY_FUNCTION__);   \
    Serial.print(' ');                   \
    Serial.print(__FILE__);              \
    Serial.print(':');                   \
    Serial.print(__LINE__);              \
    Serial.print(' ');                   \
    Serial.println(str);
#define DEBUG_PRINTDEC(x)                \
    Serial.print(millis());              \
    Serial.print(": ");                  \
    Serial.print(__PRETTY_FUNCTION__);   \
    Serial.print(' ');                   \
    Serial.print(__FILE__);              \
    Serial.print(':');                   \
    Serial.print(__LINE__);              \
    Serial.print(' ');                   \
    Serial.print(x, DEC);
#define DEBUG_PRINT_DEC(str, x)          \
    Serial.print(millis());              \
    Serial.print(": ");                  \
    Serial.print(__PRETTY_FUNCTION__);   \
    Serial.print(' ');                   \
    Serial.print(__FILE__);              \
    Serial.print(':');                   \
    Serial.print(__LINE__);              \
    Serial.print(' ');                   \
    Serial.println(str)                  \
    Serial.print(' = ');                 \
    Serial.print(x, DEC);                 
#else
#define DEBUG_PRINT(str)
#define DEBUG_PRINTDEC(x)
#define DEBUG_PRINT_DEC(str, x)
#endif

#endif
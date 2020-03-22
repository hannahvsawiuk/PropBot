
/*
debug.h - More advanced debug logging

To use, define DEBUG before including this file
*/

#ifndef DEBUGUTILS_H
#define DEBUGUTILS_H

#include <string.h>
#define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)

#ifdef DEBUG_MODE
#include <Arduino.h>
#define DEBUG_PRINT(str)                 \
    Serial.print(millis());              \
    Serial.print(": ");                  \
    Serial.print(__FUNCTION__);   \
    Serial.print(' ');                   \
    Serial.print(__FILENAME__);              \
    Serial.print(':');                   \
    Serial.print(__LINE__);              \
    Serial.print(' ');                   \
    Serial.println(str);
#define DEBUG_PRINTDEC(x)                \
    Serial.print(millis());              \
    Serial.print(": ");                  \
    Serial.print(__FUNCTION__);   \
    Serial.print(' ');                   \
    Serial.print(__FILENAME__);              \
    Serial.print(':');                   \
    Serial.print(__LINE__);              \
    Serial.print(' ');                   \
    Serial.print(x, DEC);
#define DEBUG_PRINT_DEC(str, x)          \
    Serial.print(millis());              \
    Serial.print(": ");                  \
    Serial.print(__FUNCTION__);   \
    Serial.print(' ');                   \
    Serial.print(__FILENAME__);              \
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
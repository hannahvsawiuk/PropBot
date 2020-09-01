#ifndef PINOUT_H
#define PINOUT_H

// Pins defined by the UCMotor header
#include "UCMotor.h"

//Pin definitions for the ultrasonic sensor(MB1202)
#define SCL_PIN 5              //Default SDA is Pin5 PORTC for the UNO -- you can set this to any tristate pin
#define SCL_PORT PORTC 
#define SDA_PIN 4              //Default SCL is Pin4 PORTC for the UNO -- you can set this to any tristate pin
#define SDA_PORT PORTC

// Pin definitions for the RC receiver
#ifdef __AVR_ATmega328P__ 
    #define RC_RIGHT_CHANNEL_PIN    5
    #define RC_LEFT_CHANNEL_PIN     6
    #define RC_SWA_CHANNEL_PIN      9
    #define RC_SWB_CHANNEL_PIN      10

#elif defined(__AVR_ATmega2560__)
    #define RC_RIGHT_CHANNEL_PIN    44
    #define RC_LEFT_CHANNEL_PIN     45
    #define RC_SWA_CHANNEL_PIN      46
    #define RC_SWB_CHANNEL_PIN      47

#else
    #error "No RC pin configurations available"
#endif

#endif // !PINOUT_H

#ifndef PINOUT_H
#define PINOUT_H
/**
 * @brief Pin definitions for the wheels on Propbot
 * 
 * R=Right L=Left  | F=Front B=Back 
 *      
 * R/L_F/B_<pin_type>
 * 
 */
#ifdef PROPBOT
    // Right Front
    #define RF_controlPin          2    // Motor control pin (PWM)
    #define RF_brakeReleasePin     3    // Brake release pin
    #define RF_dirPin              4    // Direction pin

    // Right Back
    #define RB_controlPin          5
    #define RB_brakeReleasePin     8    
    #define RB_dirPin              9    

    // Left Front
    #define LF_controlPin          6    
    #define LF_brakeReleasePin     10   
    #define LF_dirPin              11   

    // Left Back
    #define LB_controlPin          7    
    #define LB_brakeReleasePin     12  
    #define LB_dirPin              13   
#else
    // Else, pins defined in the UCMotor header: <UCMotor.h>
    #include "UCMotor.h"
#endif

/**
 * @brief Pin definitions for the RC receiver
 * 
 */
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
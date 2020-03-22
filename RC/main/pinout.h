#ifndef PINOUT_H
#define PINOUT_H
/**
 * @brief Pin definitions for the wheels
 * 
 * R=Right L=Left  | F=Front B=Back 
 *      
 * R/L_F/B_<pin_type>
 * 
 */
#ifdef PROPBOT
    #define RF_controlPin          2    //(RF MP)
    #define RF_brakeReleasePin     3    //(RF BP)
    #define RF_dirPin              4    //(RF DP)

    #define RB_controlPin          5    //(RB MP)
    #define RB_brakeReleasePin     8    //(RB BP)
    #define RB_dirPin              9    //(RB DP)

    #define LF_controlPin          6    //(LF MP)
    #define LF_brakeReleasePin     10   //(LF BP)
    #define LF_dirPin              11   //(LF DP)

    #define LB_controlPin          7    //(LB MP)
    #define LB_brakeReleasePin     12   //(LB BP)
    #define LB_dirPin              13   //(LB DP)
#else
    // Else pins defined by the UCMotor header
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
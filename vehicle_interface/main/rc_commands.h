#ifndef RC_COMMANDS_H
#define RC_COMMANDS_H
#include "pinout.h"
#include "wheel.h"
#include "util.h"

/*  Define the max and min thresholds for the longitudinal wheel sets */
// CH2 - right longitudinal wheel set
#define RC_RIGHT_SET_FW_MAX         1985
#define RC_RIGHT_SET_FW_MIN         1638
#define RC_RIGHT_SET_BW_MAX         1310  
#define RC_RIGHT_SET_BW_MIN         980
// CH3 - left longitudinal wheel set
#define RC_LEFT_SET_FW_MAX          1972
#define RC_LEFT_SET_FW_MIN          1632 
#define RC_LEFT_SET_BW_MAX          1320
#define RC_LEFT_SET_BW_MIN          997//1007
/*  Define channels and thresholds for the switches */
// CH5/6 - SWA/B
#define RC_SWX_LOW_MAX              1000//994
#define RC_SWX_LOW_MIN              950//987
#define RC_SWX_HIGH_MAX             2000//1981
#define RC_SWX_HIGH_MIN             1900//1974

/*  Define the minimum distance that the ultrasonic sensor can detect*/
#define MIN_DISTANCE                25
/*  Function prototypes for rc commands */
Array<wheel_motor_command_t, NUM_WHEELS> fetch_rc_commands(void); 

#endif // !RC_COMMANDS_H

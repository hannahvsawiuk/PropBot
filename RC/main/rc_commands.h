#ifndef RC_COMMANDS_H
#define RC_COMMANDS_H
#include "pinout.h"
#include "wheel.h"
#include "util.h"
#include "types.h"

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
#define RC_LEFT_SET_BW_MIN          997 
/*  Define channels and thresholds for the switches */
// CH5/6 - SWA/B
#define RC_SWX_LOW_MAX              1000 
#define RC_SWX_LOW_MIN              950 
#define RC_SWX_HIGH_MAX             2000
#define RC_SWX_HIGH_MIN             1900

/*  Function prototype for rc commands */
void initialize_rc();
Array<wheel_motor_command_t, NUM_WHEELS> fetch_rc_commands(); 

#endif // !RC_COMMANDS_H
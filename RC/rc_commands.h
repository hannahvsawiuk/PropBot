#ifndef RC_COMMANDS_H
#define RC_COMMANDS_H

/*  Define the max and min thresholds for the longitudinal wheel sets */
// CH2 - right longitudinal wheel set
#define RC_RIGHT_SET_FW_MAX         1971
#define RC_RIGHT_SET_FW_MIN         1638
#define RC_RIGHT_SET_BW_MAX         1310  
#define RC_RIGHT_SET_BW_MIN         987
// CH3 - left longitudinal wheel set
#define RC_LEFT_SET_FW_MAX          1972
#define RC_LEFT_SET_FW_MIN          1632 
#define RC_LEFT_SET_BW_MAX          1320
#define RC_LEFT_SET_BW_MIN          1007
/*  Define channels and thresholds for the switches */
// CH5/6 - SWA/B
#define RC_SWX_LOW_MAX              994
#define RC_SWX_LOW_MIN              987
#define RC_SWX_HIGH_MAX             1981
#define RC_SWX_HIGH_MIN             1974

#endif // !RC_COMMANDS_H
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
#define R_F_controlPin          2  //(RF MP)
#define R_F_brakeReleasePin     3  //(RF BP)
#define R_F_dirPin              4  //(RF DP)


#define L_F_controlPin          5  //(LF MP)
#define L_F_brakeReleasePin     8  //(LF BP)
#define L_F_dirPin              9  //(LF DP)

#define L_B_controlPin          6  //(LB MP)
#define L_B_brakeReleasePin     10 //(LB BP)
#define L_B_dirPin              11 //(LB DP)

#define R_B_controlPin          7  //(RB MP)
#define R_B_brakeReleasePin     12 //(RB BP)
#define R_B_dirPin              16 //(RB DP)

/**
 * @brief Pin definitions for the RC receiver
 * 
 */
#define RC_RIGHT_CHANNEL_PIN    44
#define RC_LEFT_CHANNEL_PIN     45
#define RC_SWA_CHANNEL_PIN      46
#define RC_SWB_CHANNEL_PIN      13

#endif // !PINOUT_H
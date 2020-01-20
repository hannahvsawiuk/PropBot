#ifndef Wheel_h
#define Wheel_h


/**
 * @brief Pin definitions for the wheels
 * 
 * R=Right L=Left  | F=Front B=Back 
 *      
 * R/L_F/B_<pin_type>
 * 
 */
#define R_F_controlPin          3;  //(RF MP)
#define R_F_brakeReleasePin     ;  //(RF BP)
#define R_F_dirPin              ;  //(RF DP)


#define L_F_controlPin          9;  //(LF MP)
#define L_F_brakeReleasePin     ;  //(LF BP)
#define L_F_dirPin              ;  //(LF DP)

#define L_B_controlPin          10 ;  //(LB MP)
#define L_B_brakeReleasePin      ;  //(LB BP)
#define L_B_dirPin               ; //(LB DP)

#define R_B_controlPin          11 ;  //(RB MP)
#define R_B_brakeReleasePin      ;  //(RB BP)
#define R_B_dirPin               ;  //(RB DP)


/**
 * @brief Wheel index mapping
 * 
 *   Front
 * 
 *  0-----1
 * 
 *  Middle
 * 
 *  3-----2
 *   
 *   Back
 */
typedef enum 
{
    RF  = 0U,
    LF  = 1U,
    LB  = 2U,
    RB  = 3U
} wheel_index_t;


uint16_t* wheel_to_register[4] = {
    &OCR2B,     // RF: pin 3
    &OCR1A,     // LF: pin 9
    &OCR1B,     // LB: pin 10
    &OCR2A      // RB: pin 11
};

#define TOP 255
/**
 * @brief Enum of motor modes
 * 
 */
typedef enum
{
    MOTOR_MODE_COAST    = 0U,
    MOTOR_MODE_BRAKE    = 1U,
    MOTOR_MODE_FORWARD  = 2U,
    MOTOR_MODE_BACKWARD = 3U 

} motor_mode_t;

wheel_set(wheel_index_t index, motor_mode_t mode, uint8_t pwm);
wheel_brake(wheel_index_t index);
wheel_coast(wheel_index_t index);

#endif // !Wheel_h

#ifndef Wheel_h
#define Wheel_h

#include <vector>

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

typedef struct wheel_pins_t {
    uint8_t control,
    uint8_t brake_release,
    uint8_t dir
};

uint16_t* wheel_to_register[4] = {
    &OCR3A,     // RF: pin 5, digital 5
    &OCR3B,     // LF: pin 6, digital 2
    &OCR4A,     // LB: pin 15, digital 6
    &OCR4B      // RB: pin 16, digital 7 
};

/**
 * @brief Pin definitions for the wheels
 * 
 * R=Right L=Left  | F=Front B=Back 
 *      
 * R/L_F/B_<pin_type>
 * 
 */
#define R_F_controlPin          3;  //(RF MP)
#define R_F_brakeReleasePin     4;  //(RF BP)
#define R_F_dirPin              5;  //(RF DP)


#define L_F_controlPin          9;  //(LF MP)
#define L_F_brakeReleasePin     15; //(LF BP)
#define L_F_dirPin              16; //(LF DP)

#define L_B_controlPin          10; //(LB MP)
#define L_B_brakeReleasePin     17; //(LB BP)
#define L_B_dirPin              18; //(LB DP)

#define R_B_controlPin          11; //(RB MP)
#define R_B_brakeReleasePin     24; //(RB BP)
#define R_B_dirPin              25; //(RB DP)

wheel_pins_t wheel_to_pins[4] = {
    {R_F_controlPin, R_F_brakeReleasePin, R_F_dirPin},
    {L_F_controlPin, L_F_brakeReleasePin, L_F_dirPin},
    {L_B_controlPin, L_B_brakeReleasePin, L_B_dirPin},   
    {R_B_controlPin, R_B_brakeReleasePin, R_B_dirPin}
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

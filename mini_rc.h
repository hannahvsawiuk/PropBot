#ifndef MINI_RC_H_
#define MINI_RC_H_

#include "wheel.h"

// PWM output pins
#define PWM1_MOTOR_PIN 11   //OC2A
#define PWM2_MOTOR_PIN 3    //OC2B
#define PWM3_MOTOR_PIN 6    //OC0A
#define PWM4_MOTOR_PIN 5    //OC0B 

// RC pins for the mini car
#define MINI_RC_RIGHT_CHANNEL_PIN   0
#define MINI_RC_LEFT_CHANNEL_PIN    1
#define MINI_RC_SWA_CHANNEL_PIN     2
#define MINI_RC_SWB_CHANNEL_PIN     13

// Arduino pin names for interface to 74HCT595 latch
#define STORAGE_REG_CLK_PIN 12
#define SHIFT_REG_CLK_PIN 4
#define OUTPUT_ENABLE_PIN 7
#define SERIAL_DATA_PIN 8

// Bit positions in the 74HCT595 shift register output
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR4_A 0
#define MOTOR4_B 6
#define MOTOR3_A 5
#define MOTOR3_B 7

#define MINI_TOP 255

struct bit_shift_t 
{
    uint8_t high;
    uint8_t low;
}; 

const Array<bit_shift_t, 4> bit_shift_map = {{
    {MOTOR1_A, MOTOR1_B},
    {MOTOR2_A, MOTOR2_B},
    {MOTOR3_A, MOTOR3_B},
    {MOTOR4_A, MOTOR4_B}
}};

class DCMotor
{
public:
    DCMotor(uint8_t motor_index);
    ~DCMotor();
    void sendCommand(wheel_motor_command_t command);

private:
    uint8_t motor_index;
    volatile uint8_t* pwm_register;
    bit_shift_t bit_shift_vals;
    void enable();
    void latch_tx(uint8_t latch_state);
};

#endif // MINI_RC_H_
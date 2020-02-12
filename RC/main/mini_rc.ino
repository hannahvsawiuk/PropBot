#include "mini_rc.h"

DCMotor::DCMotor(uint8_t motor_index)
// : motor_index{motor_index}
// , pwm_register{wheel_to_register[motor_index]}
// , bit_shift_vals{bit_shift_map[motor_index]}
{
    motor_index = motor_index;
    pwm_register = wheel_to_register[motor_index];
    bit_shift_vals = bit_shift_map[motor_index];
    
    latch_tx(~_BV(bit_shift_vals.high) & ~_BV(bit_shift_vals.low));
}

DCMotor::~DCMotor()
{
    delete pwm_register;
}

void DCMotor::sendCommand(wheel_motor_command_t command)
{
    uint8_t latch_state = 0;
    
    // BRAKE condition
    if (command.brake_release != RELEASE_BRAKE) {   
        latch_state &= ~_BV(bit_shift_vals.high);    
        latch_state &= ~_BV(bit_shift_vals.low);
        latch_tx(latch_state); 
        return;
    }

    // Step 1: set rotation direction
    if (command.dir == FORWARD) {
        if (motor_index == 3 || motor_index == 4){
            latch_state |= _BV(bit_shift_vals.high);
            latch_state &= ~_BV(bit_shift_vals.low); 
        } else {
            latch_state &= ~_BV(bit_shift_vals.high);
            latch_state |= _BV(bit_shift_vals.low);    
        }
    } else {
        if (motor_index == 3 || motor_index == 4){
            latch_state &= ~_BV(bit_shift_vals.high);
            latch_state |= _BV(bit_shift_vals.low); 
        } else {
            latch_state |= _BV(bit_shift_vals.high);
            latch_state &= ~_BV(bit_shift_vals.low);    
        }
    }
    // latch shift state
    latch_tx(latch_state);

    // Step 2: set speed
    &pwm_register = command.duty_cycle * MINI_TOP;
}

void DCMotor::enable()
{
    latch_tx(0);
    digitalWrite(OUTPUT_ENABLE_PIN, LOW);
}

void DCMotor::latch_tx(uint8_t latch_state = 0)
{
    digitalWrite(STORAGE_REG_CLK_PIN, LOW);
    digitalWrite(SERIAL_DATA_PIN, LOW);

    for (uint8_t i = 0; i < 8; i++)
    {
        digitalWrite(SHIFT_REG_CLK_PIN, LOW);
        digitalWrite(SERIAL_DATA_PIN, (latch_state & _BV(7 - i))? HIGH : LOW);
        digitalWrite(SHIFT_REG_CLK_PIN, HIGH);
    }
    digitalWrite(STORAGE_REG_CLK_PIN, HIGH);
}

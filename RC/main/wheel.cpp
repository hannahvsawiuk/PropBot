#include "wheel.h"

#ifdef PROPBOT

    /**
     * @brief Parametrized Wheel object constructor 
     * 
     */
    Wheel::Wheel(uint8_t wheel_index)
        : 
        , wheel_index(wheel_index)
        , wheel_pins{wheel_to_pins[wheel_index]}
        , control_register{wheel_to_register[wheel_index]}
    {}
    
    /**
     * @brief Default Wheel object destructor
     * 
     */
    Wheel::~Wheel() 
    {
        delete control_register;
    }
    
    /**
     * @brief Sends commands to the motor drivers
     * 
     * @param command the structured command to be sent 
     */
    void Wheel::sendCommand(wheel_motor_command_t command)
    {       
        // disable interrupts
        _disableInterrupts();
        // update the brake state
        _digitalWrite(wheel_pins.brake_release, (command.brake_release == RELEASE_BRAKE) ? HIGH : LOW);
        // update the output compare register with the scaled duty value
        *control_register = command.duty_cycle * TOP;
        // update the direction
        _digitalWrite(wheel_pins.dir, (command.dir == DIR_FW) ? HIGH : LOW);
        // re-enable interrupts
        _enableInterrupts();
    }

    /**
     * @brief Retrieves wheel speeds from encoders
     * 
     */
    uint16_t Wheel::get_speed()
    {
        #error "Not yet implemented"
    }
#else
    /* Mini RC wheel class */

    /**
     * @brief Parametrized Wheel object constructor 
     * 
     */
    Wheel::Wheel(uint8_t wheel_index)
    : wheel_index(wheel_index)
    , uc_motor(UC_DCMotor(wheel_index))
    {
        Serial.println(String("Init motor: ") + wheel_index);
        uc_motor.run(STOP);
        uc_motor.setSpeed(0);
    }

    /**
     * @brief Default Wheel object destructor 
     * 
     */
    Wheel::~Wheel() {}

    /**
     * @brief Sends commands to the motor drivers
     * 
     * @param command the structured command to be sent
     */
    void Wheel::sendCommand(wheel_motor_command_t command)
    {
        // BRAKE condition
        if (command.brake_release == ENGAGE_BRAKE) {   
            // Serial.println("BRAKE");
            uc_motor.run(STOP);
            uc_motor.setSpeed(0);
            return;
        }

        // Step 1: set rotation direction
        if (command.dir == DIR_FW) {
            uc_motor.run(FORWARD);
        } else {
            uc_motor.run(BACKWARD);
        }
        // Step 2: set the speed
        uc_motor.setSpeed(command.duty_cycle * TOP);
    }

    /**
     * @brief Retrieves wheel speeds from encoders
     * 
     */
    uint16_t Wheel::getSpeed()
    {
        #error "Not yet implemented"
    }

#endif
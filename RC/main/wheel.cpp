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
    uint16_t Wheel::getSpeed()
    {
        Serial.print(String("ERROR: ") + __FUNCTION__ + String(" not yet implemented"));
        return 0;
    }

    /**
     * @brief Configures timers for PWM and initializes the wheel objects
     * 
     * @return
     */
    Wheel ** initializeWheels()
    {
        /*
        * Fast PWM configuration:
        *   - Clear OCxA/B on Compare Match, set OCxA at BOTTOM,(non-inverting mode)
        *   - TOP = 0x03FF (1023) for MEGA and 0x0FF (255) for UNO
        *   - Prescaler = 8
        *   - Output frequency = clk_io(RTC)/8 --> 8kHz (MEGA) and 4kHz (UNO)
        */ 
        #if defined(__AVR_ATmega328P__)
            // Timer2 Fast PWM setup
            TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM22) | (1 << WGM21) | (1 << WGM20);
            TCCR2B = (1 << CS21); // 8kHz
            // Set duty cycle initially to 0%
            OCR2A = 0;  
            OCR2B = 0;

            // Timer0 Fast PWM setup
            TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 < WGM02) | (1 << WGM01) | (1 << WGM00);
            TCCR0B = (1 << CS01); // 8KHz    
            // Set duty cycle initially to 0%
            OCR0A = 0;  
            OCR0B = 0;
        #elif (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
            // Timer3 Fast PWM setup
            TCCR3A = (1 << COM3A1) | (1 << COM3B1) | (1 << WGM32) | (1 << WGM31) | (1 << WGM30);
            TCCR3B = (1 << CS31);
            // Set duty cycle initially to 0%
            OCR3A = 0;  
            OCR3B = 0;

            // Timer4 Fast PWM setup
            TCCR4A = (1 << COM4A1) | (1 << COM4B1) | (1 < WGM42) | (1 << WGM41) | (1 << WGM40);
            TCCR4B = (1 << CS41);         
            // Set duty cycle initially to 0%
            OCR4A = 0;  
            OCR4B = 0;
        #else 
            #error "No timers configured for PWM"
        #endif

        Wheel * wheel_motors[NUM_WHEELS]; 
        // Pin setup for all wheel motors
        for (int i = wheel_indices::Start + 1; i < wheel_indices::End; i++ ) {
            pinMode(wheel_to_pins[i].control,       OUTPUT);
            pinMode(wheel_to_pins[i].brake_release, OUTPUT);
            pinMode(wheel_to_pins[i].dir,           OUTPUT);
            wheel_motors[i] = new Wheel(i);    // extern
        }
        return wheel_motors;
    }
#else
    /* Mini RC wheel class */

    /**
     * @brief Parametrized Wheel object constructor 
     * 
     */
    Wheel::Wheel(uint8_t wheel_index)
    : wheel_index(wheel_index)
    {
        uc_motor = new UC_DCMotor(wheel_index); 
        Serial.println(String("Init motor: ") + wheel_index);
        uc_motor->run(STOP);
        uc_motor->setSpeed(0);
    }

    /**
     * @brief Default Wheel object destructor 
     * 
     */
    Wheel::~Wheel() 
    {
        delete uc_motor;
    }

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
            uc_motor->run(STOP);
            uc_motor->setSpeed(0);
            return;
        }

        // Step 1: set rotation direction
        if (command.dir == DIR_FW) {
            uc_motor->run(FORWARD);
        } else {
            uc_motor->run(BACKWARD);
        }
        // Step 2: set the speed
        uc_motor->setSpeed(command.duty_cycle * TOP);
    }

    /**
     * @brief Retrieves wheel speeds from encoders
     * 
     */
    uint16_t Wheel::getSpeed()
    {
        Serial.print(String("ERROR: ") + __FUNCTION__ + String(" not yet implemented"));
    }

    /**
    * @brief Initializes the mini wheel objects
    * 
    * @return
    */
    Wheel ** initializeWheels() 
    {   
        Wheel * wheel_motors[NUM_WHEELS]; 
        for (int i = wheel_indices::Start + 1; i < wheel_indices::End; i++ ) {
            wheel_motors[i] = new Wheel(i + 1);
        }
        return wheel_motors;
    }

#endif
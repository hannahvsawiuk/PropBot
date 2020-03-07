#ifndef WHEEL_h
#define WHEEL_h

#include <Arduino.h>    // For architecture selection
#include "pinout.h"     // Pin mapping
#include "types.h"      // Custom data types

/* Define Important Values */
#if defined(__AVR_ATmega328P__)
        #define TOP 0XFF
    #elif (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
        #define TOP 0x03FF     // Top value of the registers (configuration done in MAIN) 
#endif


/**
 * @brief motor command structure that includes duty, brake, and direction
 * 
 */
typedef struct wheel_motor_command_t
{
    float duty_cycle; // float from 0 to 1
    bool brake_release;
    bool dir;
};

// Direction values
#define DIR_FW          true
#define DIR_BW          false
#define ENGAGE_BRAKE    false
#define RELEASE_BRAKE   true

/* Define structures and values for Propbot */
#ifdef PROPBOT

    #define NUM_WHEELS 4 
    /**
     * @brief Wheel index mapping
     * 
     *   Front
     * 
     *  3-----0
     * 
     *  Middle
     * 
     *  2-----1
     *   
     *   Back
     */
    typedef enum wheel_index_t
    {
        Start = -1,
        RF  = 0U,
        RB  = 1U,
        LF  = 2U,
        LB  = 3U,
        End = 4U
    } wheel_indices;

    // Pseudo map bc Arduino does not support STL
    const Array<String, NUM_WHEELS> wheel_index_name_map = {"RF", "RB", "LF", "LB"};

    /**
     * @brief wheel motor pin struct
     *
     */
    typedef struct wheel_pins_t {
        uint8_t control;
        uint8_t brake_release;
        uint8_t dir;
    };

    /* Default motor commands */
    // Brake: duty = 0, brake on
    const Array<wheel_motor_command_t, NUM_WHEELS> all_brake_command = {{
        {0.0, ENGAGE_BRAKE, FORWARD},
        {0.0, ENGAGE_BRAKE, FORWARD},
        {0.0, ENGAGE_BRAKE, FORWARD},
        {0.0, ENGAGE_BRAKE, FORWARD}
    }};
    // Coast: duty = 0, brake released
    const Array<wheel_motor_command_t, NUM_WHEELS> all_coast_command = {{
        {0.0, RELEASE_BRAKE, FORWARD},
        {0.0, RELEASE_BRAKE, FORWARD},
        {0.0, RELEASE_BRAKE, FORWARD},
        {0.0, RELEASE_BRAKE, FORWARD}
    }};

    /**
     * @brief maps wheel index to a struct that defines the wheel pins (digital pin index)
     * 
     */
    const Array<wheel_pins_t, 4> wheel_to_pins = {{
        {RF_controlPin, RF_brakeReleasePin, RF_dirPin},
        {RB_controlPin, RB_brakeReleasePin, RB_dirPin},
        {LF_controlPin, LF_brakeReleasePin, LF_dirPin},
        {LB_controlPin, LB_brakeReleasePin, LB_dirPin}   
    }};

    /**
     * @brief mapping of wheel index to the output compare associated with PWM generation
     *        update the register value with the duty wrt to the TOP value to alter the duty
     * 
     */
    #if defined(__AVR_ATmega328P__)
        uint8_t* wheel_to_register[4] = {
            &OCR2A,     // #1, pin 11
            &OCR2B,     // #2, pin 3
            &OCR0A,     // #3, pin 6
            &OCR0B      // #4, pin 5
        };
    #elif (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
        uint16_t* wheel_to_register[4] = {
            &OCR3B,     // RF: digital pin 2, PE4
            &OCR3A,     // RB: digital pin 5, PE3
            &OCR4A,     // LF: digital pin 6, PH3
            &OCR4B      // LB: digital pin 7, PH4 
        };
    #else
        #error "Architecture not supported"
    #endif // Architecture

    /**
     * @brief Wheel class - Propbot
     * 
     */
    class Wheel
    {
    private:
        wheel_pins_t wheel_pins;
        uint16_t* control_register;
        uint8_t wheel_index_;
    public:
        Wheel(uint8_t index);
        ~Wheel();
        void sendCommand(wheel_motor_command_t command);
        uint16_t getSpeed();
        uint8_t wheel_index();
    
    };
#else 
    // Else, define values for mini rc
    #include "UCMotor.h"    // Mini RC motors
    #define NUM_WHEELS 2 // driving with only two drive signals

    /**
     * @brief Mini RC car wheel index mapping
     *
     */
    typedef enum wheel_index_t
    {
        Start   = -1,
        L       = 0U,
        R       = 1U,
        End     = 2U,
    } wheel_indices;

    // Associated names for wheel indices
    const Array<String, NUM_WHEELS> wheel_index_name_map = {"L", "R"};
    
    // Command for engaging braking on all wheels
    const Array<wheel_motor_command_t, NUM_WHEELS> all_brake_command = {{
        {0.0, ENGAGE_BRAKE, FORWARD},
        {0.0, ENGAGE_BRAKE, FORWARD}
    }};

    /**
     * @brief Wheel class - Mini RC car
     * 
     */
    class Wheel
    {
    private:
        UC_DCMotor * uc_motor;
        uint8_t wheel_index_;
    public:
        Wheel(uint8_t index);
        ~Wheel();
        void sendCommand(wheel_motor_command_t command);
        uint8_t wheel_index();
        uint16_t getSpeed();
    };
#endif // PROPBOT

// Function prototype
Wheel ** initializeWheels();

#endif // !WHEEL_h

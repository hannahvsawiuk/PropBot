#ifndef WHEEL_h
#define WHEEL_h

#include <Arduino.h>    // For architecture selection
#include "pinout.h"     // Pin mapping

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
struct wheel_motor_command_t
{
    uint8_t duty_cycle; // uint8_tfrom 0 to 1
    bool brake_release;
    bool dir;

    wheel_motor_command_t(uint8_t duty_cycle_, bool brake_release_, bool dir_)
    : duty_cycle(duty_cycle_),
    brake_release(brake_release_),
    dir(dir_)
    {}

};


// Direction values
#define DIR_FW          true
#define DIR_BW          false
#define ENGAGE_BRAKE    false
#define RELEASE_BRAKE   true

/* Default commands */
const wheel_motor_command_t brake_command = {0, ENGAGE_BRAKE, FORWARD};
const wheel_motor_command_t coast_command = {0, RELEASE_BRAKE, FORWARD};

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
        Start   = 0U,
        RF      = 1U,
        RB      = 2U,
        LF      = 3U,
        LB      = 4U,
        End     = 5U
    } wheel_indices;

    /* Pseudo wheel name map */
    // const char* wheel_index_name_map[NUM_WHEELS] = {
    //     "RF", 
    //     "RB", 
    //     "LF", 
    //     "LB"
    // };

    /**
     * @brief wheel motor pin struct
     *
     */
    struct wheel_pins_t {
        uint8_t control;
        uint8_t brake_release;
        uint8_t dir;
    };

    /**
     * @brief maps wheel index to a struct that defines the wheel pins (digital pin index)
     * 
     */
    const wheel_pins_t wheel_to_pins[NUM_WHEELS] = {
        {RF_controlPin, RF_brakeReleasePin, RF_dirPin},
        {RB_controlPin, RB_brakeReleasePin, RB_dirPin},
        {LF_controlPin, LF_brakeReleasePin, LF_dirPin},
        {LB_controlPin, LB_brakeReleasePin, LB_dirPin}   
    };

    /**
     * @brief mapping of wheel index to the output compare associated with PWM generation
     *        update the register value with the duty wrt to the TOP value to alter the duty
     * 
     */
    #if defined(__AVR_ATmega328P__)
        volatile uint8_t* wheel_to_register[4] = {
            &OCR2A,     // #1, pin 11
            &OCR2B,     // #2, pin 3
            &OCR0A,     // #3, pin 6
            &OCR0B      // #4, pin 5
        };
    #elif (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
        volatile uint16_t* wheel_to_register[4] = {
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
        wheel_pins_t* wheel_pins;
        volatile uint16_t* control_register;
        uint8_t wheel_index_;
    public:
        Wheel(uint8_t index);
        ~Wheel();
        void sendCommand(wheel_motor_command_t* command);
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
        Start   = 0U,
        L       = 1U,
        R       = 2U,
        End     = 3U,
    } wheel_indices;

    /* Pseudo wheel name map */
    // const char* wheel_index_name_map[NUM_WHEELS] = {
    //     "L", 
    //     "R"
    // };
    
    /**
     * @brief Wheel class - Mini RC car
     * 
     */
    class Wheel
    {
    private:
        UC_DCMotor* uc_motor;
        uint8_t wheel_index_;
    public:
        Wheel(uint8_t index);
        ~Wheel();
        void sendCommand(wheel_motor_command_t* command);
        uint8_t wheel_index();
        uint16_t getSpeed();
    };
#endif // PROPBOT

/* Definition and initialization of global static vars */
static wheel_motor_command_t* commands[NUM_WHEELS] = {};
static Wheel* wheel_motors[NUM_WHEELS] = {};

/* Initialization function prototypes */
void initializeWheels();
void initializeWheelCommands();

/* Basic wheel command function prototypes */
void brake(wheel_motor_command_t* commands[NUM_WHEELS]);
void coast(wheel_motor_command_t* commands[NUM_WHEELS]);

#endif // !WHEEL_h
#include "wheel.h"
#include "pinout.h"
#include "rc_commands.h"
#include "mini_rc/mini_rc.h"
#include <string.h>
#define DEBUG_MODE
#define MINI_RC true
#include "debug.h"

volatile bool autonomy = false;
volatile Array<wheel_motor_command_t, 4> commands = all_brake_command;


// #ifdef MINI_RC
const Array<UC_DCMotor, NUM_WHEELS> mini_rc_motor_map = {{
    {4, MOTOR34_64KHZ},  // R1 --> 4
    {3, MOTOR34_64KHZ},  // L1 --> 3
    {2, MOTOR34_64KHZ},  // R2 --> 2
    {1, MOTOR34_64KHZ}   // L2 --> 1
}}; 
// #else
//     const Array<UC_DCMotor, NUM_WHEELS> mini_rc_motor_map = {{}};
// #endif

void setup()
{
    // Setup RC pins in input mode (input capture)
    pinMode(RC_RIGHT_CHANNEL_PIN,   INPUT);
    pinMode(RC_LEFT_CHANNEL_PIN,    INPUT);
    pinMode(RC_SWA_CHANNEL_PIN,     INPUT);
    pinMode(RC_SWB_CHANNEL_PIN,     INPUT);
    if (MINI_RC) {
        
    } else {
            
        
        // Pin setup for all wheel motors
        for (int i = wheel_indices::Start + 1; i < wheel_indices::End; i++ ) {
            pinMode(wheel_to_pins[i].control,       OUTPUT);
            pinMode(wheel_to_pins[i].brake_release, OUTPUT);
            pinMode(wheel_to_pins[i].dir,           OUTPUT);
        }
        // For debugging
        Serial.begin(9600);

        // if (SERIAL == true) {
        //     // code for setting up serial comms
        // } else {
        //     /*
        //     * Fast PWM configuration:
        //     *   - Clear OCxA/B on Compare Match, set OCxA at BOTTOM,(non-inverting mode)
        //     *   - TOP = 0x03FF (1023)
        //     *   - Prescaler = 64
        //     *   - Output frequency = clk_io(RTC)/8 --> 970Hz-980Hz
        //     */ 
        //     // Timer3 Fast PWM setup
        //     TCCR3A = (1 << COM3A1) | (1 << COM3B1) | (1 << WGM32) | (1 << WGM31) | (1 << WGM30);
        //     TCCR3B = (1 << CS31);
        //     // Set duty cycle initially to 0%
        //     OCR3A = 0;  
        //     OCR3B = 0;

        //     // Timer4 Fast PWM setup
        //     TCCR4A = (1 << COM4A1) | (1 << COM4B1) | (1 < WGM42) | (1 << WGM41) | (1 << WGM40);
        //     TCCR4B = (1 << CS41);         
        //     // Set duty cycle initially to 0%
        //     OCR4A = 0;  
        //     OCR4B = 0; 
        // }
    }
}

void loop()
{
    
    // UC_DCMotor leftMotor2(1, MOTOR34_64KHZ);
    // UC_DCMotor rightMotor2(2, MOTOR34_64KHZ);
    // UC_DCMotor leftMotor1(3, MOTOR34_64KHZ);
    // UC_DCMotor rightMotor1(4, MOTOR34_64KHZ);

    // const Array<UC_DCMotor, NUM_WHEELS> mini_rc_motor_map = {{
    //     {4, MOTOR34_64KHZ},  // R1 --> 4
    //     {3, MOTOR34_64KHZ},  // L1 --> 3
    //     {2, MOTOR34_64KHZ},  // R2 --> 2
    //     {1, MOTOR34_64KHZ}   // L2 --> 1
    // }} 
    
    if (autonomy) {
        // code for comms with the autonomy computer
    } else {
        commands = fetch_rc_commands(); // update wheel commands
        String debug_str = String();    // debug string

        for (int i = wheel_indices::Start + 1; i < wheel_indices::End; i++ ) {
            if (MINI_RC) {
                mini_wheel_set(&(mini_rc_motor_map[i]), commands[i]);
            } else {
                wheel_set(i, commands[i]);
            }
            debug_str += wheel_index_name_map[i] + String(": ") + \
                           String("duty - ") + commands[i].duty_cycle + String("\t") + \
                           String("brake - ") + ((commands[i].brake_release == 0)? String("on") : String("off")) + String("\t") + \
                           String("dir - ") + ((commands[i].dir == 1)? String("fw") : String("bw")) + \
                           String("\n");
        }
        Serial.println(debug_str);
    }
}
#include "pinout.h"
#include "rc_commands.h"
#include "UCMotor.h"
#include <string.h>
#include <stdio.h>
#include <SoftI2CMaster.h>     //You will need to install this library

volatile Array<wheel_motor_command_t, NUM_WHEELS> commands = all_brake_command;

// Initialize the UC motor objects
UC_DCMotor mL(1);
UC_DCMotor mR(2);
UC_DCMotor * minis[2] = {
    &mL,
    &mR
}; 

void setup()
{
    // For debugging
    Serial.begin(9600);
    Serial.println("Init");

    // Setup RC pins in input mode (input capture)
    pinMode(RC_RIGHT_CHANNEL_PIN,   INPUT);
    pinMode(RC_LEFT_CHANNEL_PIN,    INPUT);
    pinMode(RC_SWA_CHANNEL_PIN,     INPUT);
    pinMode(RC_SWB_CHANNEL_PIN,     INPUT);

    //Initialize I2C protocol for ultrasonic sensor
    i2c_init();
      
    // Indicate setup done
    Serial.println("Setup complete");
}

/**
 * @brief Sends commands to a UC_DC motor 
 * 
 * @param uc_motor reference to the UC_DC motor object
 * @param command the command to send
 * 
 * @return 
 */
void sendCommandMini(UC_DCMotor * uc_motor, wheel_motor_command_t command) {
    // BRAKE condition
    if (command.brake_release == ENGAGE_BRAKE) {  
        uc_motor->run(STOP);
        uc_motor->setSpeed(0);
        return;
    }

    // Step 1: set rotation direction
    String debug_str = String();
    if (command.dir == DIR_FW) {
        uc_motor->run(FORWARD);
    } else {
        uc_motor->run(BACKWARD);
    }

    // Step 2: set speed
    uc_motor->setSpeed(command.duty_cycle * TOP);
}

void loop()
{
    int distance = read_ultrasonic_sensor();
    if (distance > MIN_DISTANCE){
    // Update wheel commands
    commands = fetch_rc_commands(); 
    }

    else{
      commands = all_brake_command;
    }
    // Send commands to wheels
    for (int i = wheel_indices::Start + 1; i < wheel_indices::End; i++ ) {
        sendCommandMini(minis[i], commands[i]);
    }
}

//////////////////////////////////////////////////////////
//       Read the sensor at the default address         //
//////////////////////////////////////////////////////////
int read_ultrasonic_sensor(){
  boolean error = 0;  //Create a bit to check for catch errors as needed.
  int distance;
  
  //Take a range reading at the default address of 224
  error = start_sensor(224);    //Start the sensor and collect any error codes.
  if (!error){                  //If you had an error starting the sensor there is little point in reading it as you will get old data.
    delay(100);
    distance = read_sensor(224);   //reading the sensor will return an integer value -- if this value is 0 there was an error
    //Serial.print("Distance:");Serial.println(range);Serial.print("cm");
    Serial.println((String)"Distance:"+distance+" cm");
    return distance;
  }
}

///////////////////////////////////////////////////
// Function: Start a range reading on the sensor //
///////////////////////////////////////////////////
//Uses the I2C library to start a sensor at the given address
//Collects and reports an error bit where: 1 = there was an error or 0 = there was no error.
//INPUTS: byte bit8address = the address of the sensor that we want to command a range reading
//OUPUTS: bit  errorlevel = reports if the function was successful in taking a range reading: 1 = the function
//  had an error, 0 = the function was successful
boolean start_sensor(byte bit8address){
  boolean errorlevel = 0;
  bit8address = bit8address & B11111110;               //Do a bitwise 'and' operation to force the last bit to be zero -- we are writing to the address.
  errorlevel = !i2c_start(bit8address) | errorlevel;   //Run i2c_start(address) while doing so, collect any errors where 1 = there was an error.
  errorlevel = !i2c_write(81) | errorlevel;            //Send the 'take range reading' command. (notice how the library has error = 0 so I had to use "!" (not) to invert the error)
  i2c_stop();
  return errorlevel;
}



///////////////////////////////////////////////////////////////////////
// Function: Read the range from the sensor at the specified address //
///////////////////////////////////////////////////////////////////////
//Uses the I2C library to read a sensor at the given address
//Collects errors and reports an invalid range of "0" if there was a problem.
//INPUTS: byte  bit8address = the address of the sensor to read from
//OUPUTS: int   range = the distance in cm that the sensor reported; if "0" there was a communication error
int read_sensor(byte bit8address){
  boolean errorlevel = 0;
  int range = 0;
  byte range_highbyte = 0;
  byte range_lowbyte = 0;
  bit8address = bit8address | B00000001;  //Do a bitwise 'or' operation to force the last bit to be 'one' -- we are reading from the address.
  errorlevel = !i2c_start(bit8address) | errorlevel;
  range_highbyte = i2c_read(0);           //Read a byte and send an ACK (acknowledge)
  range_lowbyte  = i2c_read(1);           //Read a byte and send a NACK to terminate the transmission
  i2c_stop();
  range = (range_highbyte * 256) + range_lowbyte;  //compile the range integer from the two bytes received.
  if(errorlevel){
    return 0;
  }
  else{
    return range;
  }
}

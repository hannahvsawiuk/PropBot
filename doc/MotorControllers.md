# Motor Controller Introduction and Options

## The Current Harware

### Background
After reading through all existing documentation as well as extensive research, our team was unable to find datasheets and suppliers and manufacturer information for both the motors and the drivers. Due to this, it is our intention to replace the drivers.

Given the number of wires exiting the motors as well as the shape of the motor, we have concluded that they are in fact brushless DC motors. We came to this conclusion as we observed inputs for the three phases as well three cables for the sensors to enable electrical commutation. Our guess is the motors use Hall-effect sensors. Further, the shape indicates that the motors are BLDC as they lack depth but are rated up to 15A.

### Moving Forward

Given this information, we are now looking for either standalone motor controllers or a system of motor controllers that can be daisy-chained together to facilitate complete control of the robot.

### Determined Motor Specifications

| Parameter       | Value       |
|-----------------|-------------|
| Power           | 250 Watts   |
| Nominal Voltage | 24 Volts DC |
| Current limit   | 10 Amps     |

### Other Assumptions

Given the provided Arduino code, it seems that the current drivers are integrated three-phase BLDC drivers. There are three control signals:

| Pin Name                | Functionality              | Type                           |
|-------------------------|----------------------------|--------------------------------|
| L/R_F/B_motorPin        | Input PWM pin              | PWM, active high               |
| L/R_F/B_brakeReleasePin | Releases brake             | Digital, active high           |
| L/R_F/B_directionPin    | Sets direction of rotation | Digital, active high. High = F |

#### Current Motor Speed and Direction Control

The controlled used at the moment is an Arduino
`AnalogWrite()` is used to set the speed of the motors via PWM.

**PWM Specs:**

- The pins for the front wheels output PWM signals at 980Hz, and the motor pins for the back wheels generate PWM signals at 490 Hz.

- The duty cycle of the PWM waveforms is set by an integer between 0 and 255.

## Controller and Driver Options

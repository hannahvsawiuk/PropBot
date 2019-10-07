# Motor Controller Introduction and Options

## The Current Harware

### Background
After reading through all existing documentation as well as extensive research, our team was unable to find datasheets and suppliers and manufacturer information for both the motors and the drivers. Due to this, it is our intention to replace the drivers.

Given the number of wires exiting the motors as well as the shape of the motor, we have concluded that they are in fact brushless DC motors. We came to this conclusion as we observed inputs for the three phases as well three cables for the sensors to enable electrical commutation. Our guess is the motors use Hall-effect sensors. Further, the shape indicates that the motors are BLDC as they lack depth but are rated up to 15A.

### Moving Forward

Given this information, we are now looking for either standalone motor controllers or a system of motor controllers that can be daisy-chained together to facilitate complete control of the robot.


### Determined Motor Specifications

| Parameter       | Value |   |   |   |
|-----------------|-------|---|---|---|
| Nominal Voltage | 24    |   |   |   |
| Current limit   | 15    |   |   |   |

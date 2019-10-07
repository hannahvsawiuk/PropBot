# Motor Controller Introduction and Options

## The Current Harware

The robot possesses six motors, but only four DC/DC converters and four controllers.

### Background

After reading through all existing documentation as well as extensive research, our team was unable to find datasheets and suppliers and manufacturer information for both the motors and the drivers. Due to this, it is our intention to replace the drivers.

Given the number of wires exiting the motors as well as the shape of the motor, we have concluded that they are in fact brushless DC motors. We came to this conclusion as we observed inputs for the three phases as well three cables for the sensors to enable electrical commutation. Our guess is the motors use Hall-effect sensors. Further, the shape indicates that the motors are BLDC as they lack depth but are rated up to 10A.

### Moving Forward

Given this information, we are now looking for either standalone motor controllers or a system of motor controllers that can be daisy-chained together to facilitate complete control of the robot. Further, cheaper controllers could be integrated into the system for the non-motion motors to improve braking by either de-energizing or energizing the braking system.

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

**PWM Specs**

- The pins for the front wheels output PWM signals at 980Hz, and the motor pins for the back wheels generate PWM signals at 490 Hz.

- The duty cycle of the PWM waveforms is set by an integer between 0 and 255.

## Controller and Driver Options

### High Cost Range

### Medium Cost Range

#### Option 1 - RoboteQ FBL2360

**Product Information**

Member of the FBL23xx family of medium power dual channel, BLDC motor controllers.

- [Product page](https://www.roboteq.com/index.php/component/virtuemart/403/fbl2360b-detail?Itemid=971)
- [Datasheet](https://www.roboteq.com/index.php/docman/motor-controllers-documents-and-files/documentation/datasheets/fbl23xx-datasheet/203-fbl23xx-datasheet/file)

**Electrical Specs**

|Parameter                      | Value               |
|-------------------------------|---------------------|
| Max Voltage                   | 60V                 |
| # Channels                    | 2                   |
| Max current / channel         | 60A                 |
| Continuous current/ channel   | 40A                 |
| ON resistance                 | 2.5 mOhms           |
| Max power dissipation         | 3600 W              |
| Control loop frequency        | 1000 Hz             |
| Regenerative braking          | Yes                 |
| RC controllable               | Yes                 |
| Other supported communication | Analog, RS232, USB, CANbus |

**Mechanical Specs**

| Parameter     | Value                |
|---------------|----------------------|
| Weight        | 452g                 |
| Dimensions    | 140mm x 140mm x 25mm |

**Pricing and Availability**

- US$ 695.00 - but one can be used for two motors since there are two channels. Works out  to $US 346.50 per motor
- Readily available

**Feasibily Analysis and Intended Implementation**

Given that the controller has two channels and can be daisy chained together. From the product page:
>For mobile robot applications, the controller’s two motor channels can either be operated independently or mixed to move and steer a vehicle. Using CAN bus, up to 127 controllers can be networked at up to 1Mbit/s on a single twisted pair.

This means that two modules could be purchased and integrated into the system to control the four motors (2 x front, 2 x back). For the two central wheels, the existing drivers or cheaper drivers could be used to simply brake when necessary.

The price is fairly high, but cheaper than others when the number of channels is taken into account. But, the price is fair for the amount of functionality it provides, especially given the controller's ability to be daisy-chained to create a full robot motion controller.

**Analysis of Custom Motion Capabilities**

One main concern of off-the-shelf controllers is a lack of low-level control of turning radii and speed as well as visibility into the motion (speed, direction, etc.).

Firstly, the controllers can store and run up to 1,500 lines of MicroBasic source code. MicroBasic is a very simple language. It can be used to get insights into the motion of the motors. See the links below for more information

- [Overview of MicroBasic](https://www.roboteq.com/index.php/technology-menu/microbasic-technology)
- [Example of MicroBasic for a RoboteQ controller](https://github.com/gsisko/RoboteQ-Microbasic-Examples/blob/master/Motion%20Control/Motion_Control_in_CLOSED_LOOP_COUNT_POSITION.mbs)
- [Advanced Brushed and Brushless Digital Motor Controllers User Manual](https://www.roboteq.com/index.php/docman/motor-controllers-documents-and-files/documentation/user-manual/272-roboteq-controllers-user-manual-v17/file)

*NOTE: This controller is not compatible with the ROS RoboteQ driver.* See this link for information on which RoboteQ controllers are compatible with the ROS driver: [github repo](https://github.com/g/roboteq)

#### Option 2 - RoboteQ SBL1360A

**Product Information**

Member of the SBL13xx family of low power and compact size brushless DC motor controllers

- [Product page](https://www.roboteq.com/index.php/compoent/virtuemart/429/sbl1360-277-detail?Itemid=971)
- [Datasheet](https://www.roboteq.com/index.php/docman/motor-controllers-documents-and-files/documentation/datasheets/sbl13xx-datasheet/61-sbl13xx-datasheet/file)

**Electrical Specs**

|Parameter                      | Value               |
|-------------------------------|---------------------|
| Max Voltage                   | 60V                 |
| # Channels                    | 1                   |
| Max current / channel         | 30A                 |
| Continuous current/ channel   | 20A                 |
| ON resistance                 | 20 mOhms            |
| Max power dissipation         | 1200 W              |
| Control loop frequency        | 1000 Hz             |
| Regenerative braking          | Yes                 |
| RC controllable               | Yes                 |
| Other supported communication | Analog, RS232, USB, CANbus |

**Mechanical Specs**

| Parameter     | Value                |
|---------------|----------------------|
| Weight        | 60g                  |
| Dimensions    | 70mm x 70mm x 27mm   |

**Pricing and Availability**

- US$ 275
- Readily available

**Feasibily Analysis and Intended Implementation**

Given that the controller has two channels and can be daisy chained together. From the product page:
>The SBL1360A accepts commands received from an RC radio, Analog Joystick, wireless modem, or microcomputer. Using CAN bus, up to 127 controllers can be networked on a single twisted pair cable. Numerous safety features are incorporated into the controller to ensure reliable and safe operation.

In this case, a driver for each motor is required.

The price is in the medium range, but the functionality is extensive.

**Analysis of Custom Motion Capabilities**

Just like the other controllers from RoboteQ, the functionality and visibility of the system can be customized.

From the product page:
> The controller's operation can be extensively automated and customized using Basic Language scripts. The controller can be configured, monitored and tuned in realtime using a Roboteq's free PC utility. The controller can also be reprogrammed in the field with the latest features by downloading new operating software from Roboteq.

Unlike the FB23xx family of controllers, the SBL13xx family is compatible with the ROS driver. See link here: [github repo](https://github.com/g/roboteq).

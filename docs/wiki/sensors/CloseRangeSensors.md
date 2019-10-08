# Short distance sensors

Sparkfun has a excellent comparison site: https://www.sparkfun.com/distance_sensor_comparison_guide

## General Goals

- Inexpensive
- Decent Range and Arc
  - Long enough range to stop robot
  - Arc should be wide enough so we don't need an execessive number of them
- Weatherproof?
  - Might be difficult for raw components

## Summary Table

| Sensor   | Sensor Type | Cost   | Max Range | Accuracy | Arc       | Waterproof  |
|  ---     |     ---     |  ---   |    ---    | ---      | ---       |    ---      |
|Sharp GP2 | Infrared    | $12-18 | 80-550cm  | ?        | Thin      | No          |
|VL53L1X   | Laser       | $13-16 | 200-400cm | 4cm      | Very Thin | No          |
|tinyLiDAR | Laser       | $33    | 200cm     | 3%       | Very Thin | No          |
|HC-SR04   | Ultrasonic  | $3     | 500cm     | 0.3cm    | Small     | No          |
|MaxSonar  | UltraSonic  | $35-90 | 5m-7.6m   | 1cm      | Small     | Some Models |


## Infrared

According to this article https://www.makerguides.com/sharp-gp2y0a710k0f-ir-distance-sensor-arduino-tutorial/
a IR sensor is essentially a beam going out the sensor. So the detection angle would be fairly thin. 

### Sharp GP2*
Theres a bunch of variations of this IR distance sensor from sharp. The main difference being the range of detection.

Range: 10-80cm
https://www.robotshop.com/ca/en/sharp-gp2y0a21yk0f-ir-range-sensor.html

Range: 20-150cm
https://www.robotshop.com/ca/en/sharp-gp2y0a02yk0f-ir-range-sensor.html

Range: 100-550cm
https://www.robotshop.com/ca/en/sharp-gp2y0a710k0f-ir-range-sensor.html

Interface: Analog Voltage Out (Higher voltage = Longer distance, non-linear)

Pros:
- Cheap: $12-$18 per piece (Slightly less if bought in 5+ quantity)
- Up to 550cm of range

Cons:
- Analog output, requires separate ADC or connect to Arduino's builtin ADC.
- According to the datasheet it should not be exposed to "Direct light from sun" in order to guarentee accurate readings.
- Doesn't look very weatherproof

## Laser

Basically a fixed lidar...

### VL53L1X
400cm Range
https://www.robotshop.com/ca/en/tof-range-finder-sensor-breakout-board-voltage-regulator-vl53l1x.html

200cm Range
https://www.robotshop.com/ca/en/tof-range-finder-sensor-breakout-board-voltage-regulator-vl53l0x.html

Interface: I2C

Pros:
- Cheap: $13-16
- Range: up to 400cm. Supposedly 2-4cm accuracy as well.
- Not affected by ambient lighting conditions
- 50 Hz Sample Rate

Cons:
- Does't look waterproof at all (Bare PCB)

### tinyLiDAR
https://www.robotshop.com/ca/en/tinylidar-tof-range-finder-sensor.html

Pros:
- Not affected by ambient lighting conditions
- 60 Hz Sample Rate

Cons:
- Somewhat Expensive: $33
- Range: 4-200cm, barely good enough
- Does't look waterproof at all (Bare PCB)

## Ultrasonic

Ultrasonic generally has the advantage of low power consumption. IR/Laser tends to have currents of around 20-40 mA @ 3.3-5V but Ultrasonic
uses around 1-2 mA @ 5V.

### HC-SR04

The classic CPEN291 sensor... (Bad memories)

Interface: Timed GPIO

Pros:
- Cheap as dirt: $12 for 5 (https://www.amazon.ca/HC-SR04-Ultrasonic-Distance-Arduino-MEGA2560/dp/B01COSN7O6)
- Supposedly up to 500cm with 0.3cm accuracy
- Good software support

Cons:
- GPIO interface is annoying to work with if we have to write the library
- From experience its terrible on moderately slanted walls/objects
- Low polling frequency
- Not very waterproof

### MaxSonar
There are many variations of the MaxSonar
https://www.sparkfun.com/datasheets/Sensors/Proximity/Sensor_Selection_Guide.pdf

Model: 5 m range bare PCB $35
https://www.sparkfun.com/products/11309

There are more models inbetween (e.g. 5m, 6m range w/wo waterproof case)

Model: 7.6 m range with waterproof case $90
https://www.sparkfun.com/products/9496

Interface: Time GPIO or Analog or Serial/RS232

Pros:
- Longest range of all the sensors at 7.6 m for the top model
- Many ways to communicate with it
- Waterproof Casing available

Cons:
- Super Expensive: $35-$90 
- Low polling frequency as low as 8 Hz for the longest range version
- The waterproof casing seems kinda bulky


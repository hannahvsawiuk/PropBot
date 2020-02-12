#include "UCMotor.h"
#include "../wheel.h"

#define MINI_FW 0x01
#define MINI_BW 0x02
#define MINI_R  0x03
#define MINI_L  0x04
#define STOP    0x05

#define MINI_TOP 0xFF

int mini_wheel_set(UC_DCMotor* motor, wheel_motor_command_t command);

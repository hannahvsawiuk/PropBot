#include "EEPROM_address.h"
#include "pinout.h"
#include "array_function.h"
#include "threshold_function.h"
#include <EEPROM.h>
#include <Console.h>

char action = 'O';

//**********Right Joystick**********//
void get_right_threshold(Array *a){
	int rc2_max, rc2_min, rc2_central;

	//MAX Position
	Serial.println("Please move the right joystick to the most forward position.\n");
	Serial.println("Type in 'S' to start measurement or type in 'E' to end measurement.\n");
	action = Console.read();
	while(action == 'S'){
		insertArray(a, pulseIn(rc_channel2, HIGH));
	}

	//MIN Position
	Serial.println("Please move the right joystick to the most backward position.\n");
	Serial.println("Type in 'S' to start measurement or type in 'E' to end measurement.\n");
	action = Console.read();
	while (action == 'S') {
		insertArray(a, pulseIn(rc_channel2, HIGH));
	}

	//Central Position
	Serial.println("Please move the right joystick to the central position.\n");
	Serial.println("Type in 'S' to start measurement or type in 'E' to end measurement.\n");
	action = Console.read();
	while (action == 'S') {
		insertArray(a, pulseIn(rc_channel2, HIGH));
	}

	Array_sort(a->array, a->size);
	rc2_max = a->array[a->size];
	rc2_min = a->array[0];
	rc2_central = Find_median(a->array, a->size);

	EEPROM.put(rc2_addr_max, rc2_max);
	EEPROM.put(rc2_addr_min, rc2_min);
	EEPROM.put(rc2_addr_central, rc2_central);
	
	freeArray(a);
	return;
}

////**********Left Joystick**********//
void get_left_threshold(Array *a){
	int rc3_max, rc3_min, rc3_central;

	//MAX Position
	Serial.println("Please move the left joystick to the most forward position.\n");
	Serial.println("Type in 'S' to start measurement or type in 'E' to end measurement.\n");
	action = Console.read();
	while (action == 'S') {
		insertArray(a, pulseIn(rc_channel3, HIGH));
	}

	//MIN Position
	Serial.println("Please move the left joystick to the most backward position.\n");
	Serial.println("Type in 'S' to start measurement or type in 'E' to end measurement.\n");
	action = Console.read();
	while (action == 'S') {
		insertArray(a, pulseIn(rc_channel3, HIGH));
	}

	//Central Position
	Serial.println("Please move the left joystick to the central position.\n");
	Serial.println("Type in 'S' to start measurement or type in 'E' to end measurement.\n");
	action = Console.read();
	while (action == 'S') {
		insertArray(a, pulseIn(rc_channel3, HIGH));
	}

	Array_sort(a->array, a->size);
	rc3_max = a->array[a->size];
	rc3_min = a->array[0];
	rc3_central = Find_median(a->array, a->size);

	EEPROM.put(rc3_addr_max, rc3_max);
	EEPROM.put(rc3_addr_min, rc3_min);
	EEPROM.put(rc3_addr_central, rc3_central);

	freeArray(a);
	return;
}

//**********Switch A(Remote E-stop)**********//
void get_switchA_threshold(Array *a){
	int swa_max, swa_min;

	//Position 0
	Serial.println("Please move Switch A to 0\n");
	Serial.println("Type in 'S' to start measurement or type in 'E' to end measurement.\n");
	action = Console.read();
	while (action == 'S') {
		insertArray(a, pulseIn(switch_A, HIGH));
	}

	//Position 1
	Serial.println("Please move Switch A to 1\n");
	Serial.println("Type in 'S' to start measurement or type in 'E' to end measurement.\n");
	action = Console.read();
	while (action == 'S') {
		insertArray(a, pulseIn(switch_A, HIGH));
	}

	Array_sort(a->array, a->size);
	swa_max = a->array[a->size];
	swa_min = a->array[0];

	EEPROM.put(rc5_addr_zero, swa_max);
	EEPROM.put(rc5_addr_one, swa_min);  

	freeArray(a);
	return;
}

//**********Switch B(Mode Switch)**********//
void get_switchB_threshold(Array *a){
	int swb_max, swb_min;

	//Position 0
	Serial.println("Please move Switch B to 0\n");
	Serial.println("Type in 'S' to start measurement or type in 'E' to end measurement.\n");
	action = Console.read();
	while (action == 'S') {
		insertArray(a, pulseIn(switch_B, HIGH));
	}

	//Position 1
	Serial.println("Please move Switch B to 1\n");
	Serial.println("Type in 'S' to start measurement or type in 'E' to end measurement.\n");
	action = Console.read();
	while (action == 'S') {
		insertArray(a, pulseIn(switch_B, HIGH));
	}

	Array_sort(a->array, a->size);
	swb_max = a->array[a->size];
	swb_min = a->array[0];

	EEPROM.put(rc6_addr_zero, swb_max);
	EEPROM.put(rc6_addr_one, swb_min);

	freeArray(a);
	return;
}

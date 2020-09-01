#include "pinout.h"
#include "array_function.h"
#include "threshold_function.h"
#include <stdlib.h>
#include <stdio.h>
#include <Console.h>

void setup() {
  Console.begin();    // Initialize Console
  while (!Console);   // Wait for the Console port to connect

  pinMode(rc_channel2, INPUT);
  pinMode(rc_channel3, INPUT);
  pinMode(switch_A, INPUT);
  pinMode(switch_B, INPUT); 
  Serial.begin(9600);
}

void loop() {
   int rc2;          //Right Joystick
   int rc3;          //Left Joystick
   int swa;          //Channel 5 (Remote E-stop)
   int swb;          //Channel 6 (Mode Switch)

   Array channel2;  //Temporary array to store values from Channel 2
   Array channel3;  //Temporary array to store values from Channel 3
   Array switchA;   //Temporary array to store values from Switch A
   Array switchB;   //Temporary array to store values from Switch B
 
//Initilize the arrays above with initial size of 5
  initArray(&channel2, 5);
  initArray(&channel3, 5);
  initArray(&switchA, 5);
  initArray(&switchB, 5);

  Serial.println("Please put both joysticks at the central position.\n");
  Serial.println("Please put all the switchs at 0 position.\n");

 //Measure and store the threshold value in EEPROM
   get_right_threshold(&channel2);
   get_left_threshold(&channel3);
   get_switchA_threshold(&switchA);
   get_switchB_threshold(&switchB);

 //clear all the temporary array
   freeArray(&channel2);
   freeArray(&channel3);
   freeArray(&switchA);
   freeArray(&switchB);

   for(;;){} //stop the loop function
}

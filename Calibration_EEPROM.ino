#include<EEPROM.h>
int rc_channel2 = 3;
int rc_channel3 = 4;
int switchA = 6;  //Channel 5
int switchB = 7;  //Channel 6

//Address in EEPROM
const int rc2_addr_max = 0;
const int rc2_addr_min = 1;
const int rc2_addr_central = 2;

const int rc3_addr_max = 3;
const int rc3_addr_min = 4;
const int rc3_addr_central = 5;

const int rc5_addr_zero = 6;
const int rc5_addr_one = 7;

const int rc6_addr_zero = 8;
const int rc6_addr_one = 9;

void setup() {
  // put your setup code here, to run once:
  pinMode(rc_channel2, INPUT);
  pinMode(rc_channel3, INPUT);
  pinMode(switchA, INPUT);
  pinMode(switchB, INPUT); 
  Serial.begin(9600);
}

void loop() {
 int rc2;          //Right Joystick
   int rc3;        //Left Joystick
     int swa;      //Channel 5 (Remote E-stop)
       int swb;    //Channel 6 (Mode Switch)
       
//**********Right Joystick**********//
  //MAX Position
  Serial.println("Please move the right joystick to the maximum position\n");
  delay(2000);
  rc2 = pulseIn(rc_channel2, HIGH);
  EEPROM.put(rc2_addr_max, rc2);  
  //MIN Position
  Serial.println("Please move the right joystick to the minimum position\n");
  delay(2000);
  rc2 = pulseIn(rc_channel2, HIGH);
  EEPROM.put(rc2_addr_min, rc2);  

  //Central Position
  Serial.println("Please move the right joystick to the central position\n");
  delay(2000);
  rc2 = pulseIn(rc_channel2, HIGH);
  EEPROM.put(rc2_addr_central, rc2);  

//**********Left Joystick**********//
  //MAX Position
  Serial.println("Please move the left joystick to the maximum position\n");
  delay(2000);
  rc3 = pulseIn(rc_channel3, HIGH);
  EEPROM.put(rc3_addr_max, rc3);  
  
  //MIN Position
  Serial.println("Please move the left joystick to the minimum position\n");
  delay(2000);
  rc3 = pulseIn(rc_channel3, HIGH);
  EEPROM.put(rc3_addr_min, rc3);  

  //Central Position
  Serial.println("Please move the left joystick to the central position\n");
  delay(2000);
  rc3 = pulseIn(rc_channel3, HIGH);
  EEPROM.put(rc3_addr_central, rc3);  

//**********Switch A(Remote E-stop)**********//
  //Position 0
  Serial.println("Please move Switch A to 0\n");
  delay(2000);
  swa = pulseIn(switchA, HIGH);
  EEPROM.put(rc5_addr_zero, swa);  
  //Position 1
  Serial.println("Please move Switch A to 1\n");
  delay(2000);
  swa = pulseIn(switchA, HIGH);
  EEPROM.put(rc5_addr_one, swa);  

//**********Switch B(Mode Switch)**********//
  //Position 0
  Serial.println("Please move Switch B to 0\n");
  delay(2000);
  swb = pulseIn(switchB, HIGH);
  EEPROM.put(rc6_addr_zero, swb);  
  
  //Position 1
  Serial.println("Please move Switch B to 1\n");
  delay(2000);
  swb = pulseIn(switchB, HIGH);
  EEPROM.put(rc6_addr_one, swb);  
  
  for(;;){} //stop the loop function
}

#include <FileIO.h>

int rc_channel2 = 3;
int rc_channel3 = 4;
int switchA = 6;  //Channel 5
int switchB = 7;  //Channel 6
File script;

void setup() {
  // put your setup code here, to run once:
  pinMode(rc_channel2, INPUT);
  pinMode(rc_channel3, INPUT);
  pinMode(switchA, INPUT);
  pinMode(switchB, INPUT); 
  FileSystem.begin();
  File script = FileSystem.open("RC_Threshold.txt", FILE_WRITE);
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
  script.print("Righ Joystick\n");
  delay(2000);
  rc2 = pulseIn(rc_channel2, HIGH);
  script.print("MAX: "); script.println(rc2); script.print("\n");
  
  //MIN Position
  Serial.println("Please move the right joystick to the minimum position\n");
  delay(2000);
  rc2 = pulseIn(rc_channel2, HIGH);
  script.print("MIN: "); script.println(rc2); script.print("\n");

  //Central Position
  Serial.println("Please move the right joystick to the central position\n");
  delay(2000);
  rc2 = pulseIn(rc_channel2, HIGH);
  script.print("Central: "); script.println(rc2); script.print("\n");

//**********Left Joystick**********//
  //MAX Position
  Serial.println("Please move the left joystick to the maximum position\n");
  script.print("Left Joystick\n");
  delay(2000);
  rc3 = pulseIn(rc_channel3, HIGH);
  script.print("MAX: "); script.println(rc3); script.print("\n");
  
  //MIN Position
  Serial.println("Please move the left joystick to the minimum position\n");
  delay(2000);
  rc3 = pulseIn(rc_channel3, HIGH);
  script.print("MIN: "); script.println(rc3); script.print("\n");

  //Central Position
  Serial.println("Please move the left joystick to the central position\n");
  delay(2000);
  rc3 = pulseIn(rc_channel3, HIGH);
  script.print("Central: "); script.println(rc3); script.print("\n");

//**********Switch A(Remote E-stop)**********//
  //Position 0
  Serial.println("Please move Switch A to 0\n");
  script.print("Switch A\n");
  delay(2000);
  swa = pulseIn(switchA, HIGH);
  script.print("MAX: "); script.println(swa); script.print("\n");
  
  //Position 1
  Serial.println("Please move Switch A to 1\n");
  delay(2000);
  swa = pulseIn(switchA, HIGH);
  script.print("MIN: "); script.println(swa); script.print("\n");

//**********Switch B(Mode Switch)**********//
  //Position 0
  Serial.println("Please move Switch B to 0\n");
  script.print("Switch B\n");
  delay(2000);
  swb = pulseIn(switchB, HIGH);
  script.print("MAX: "); script.println(swb); script.print("\n");
  
  //Position 1
  Serial.println("Please move Switch B to 1\n");
  delay(2000);
  swb = pulseIn(switchB, HIGH);
  script.print("MIN: "); script.println(swb); script.print("\n");
  
  script.close();
  for(;;){} //stop the loop function
}

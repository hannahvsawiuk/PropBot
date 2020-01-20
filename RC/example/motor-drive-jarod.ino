#include <Console.h>


// Pin Definitions  R= Right L=Left  F= Front M= Middle B=Back
int R_F_motorPin         = 3;  //(RF MP)
int R_F_brakeReleasePin  = 4;  //(RF BP)
int R_F_DirectionPin     = 2;  //(RF DP)

int R_B_motorPin         = 5 ;  //(RB MP)
int R_B_brakeReleasePin  = 6 ;  //(RB BP)
int R_B_DirectionPin     = 7 ;  //(RB DP)

int L_F_motorPin         = 11;  //(LF MP)
int L_F_brakeReleasePin  = 12;  //(LF BP)
int L_F_DirectionPin     = 13;  //(LF DP)

int L_B_motorPin         = 9 ;  //(LB MP)
int L_B_brakeReleasePin  = 8 ;  //(LB BP)
int L_B_DirectionPin     = 10 ; //(LB DP)




// Variable Definations
int ForwardSpeed       = 150;
int BackSpeed          = 200;
int LeftTurnSpeed      = 200;
int RightTurnSpeed     = 200;

int DelayTimeForward   = 250;           // Movement time after pressing button forward each time
int DelayTimeBack      = 350;      // Movement time after pressing button backward each time
int DelayTimeLeft      = 350;           // Movement time after pressing button backward each time
int DelayTimeRight     = 350;           // Movement time after pressing button backward each time

int DelayTimeForwardLong    = 1000;           // Movement time after pressing button forward each time
int DelayTimeBackLong       = 1500;      // Movement time after pressing button backward each time
int DelayTimeLeftLong       = 1300;           // Movement time after pressing button backward each time
int DelayTimeRightLong      = 1300;           // Movement time after pressing button backward each time


int Movement           = 'O';

// START
void setup() {

  Bridge.begin();   // Initialize Bridge
  Console.begin();  // Initialize ConsolE

  // Wait for the Console port to connect
  while (!Console){
    ;
  }

  Console.println("You're connected to the Console!!!!");
  // initialize the LED pin as an output:

  pinMode(R_F_motorPin , OUTPUT);
  pinMode(R_B_motorPin , OUTPUT);
  pinMode(L_F_motorPin , OUTPUT);
  pinMode(L_B_motorPin , OUTPUT);

  pinMode(R_F_DirectionPin , OUTPUT);
  pinMode(R_B_DirectionPin , OUTPUT);
  pinMode(L_F_DirectionPin , OUTPUT);
  pinMode(L_B_DirectionPin , OUTPUT);

  pinMode(R_F_brakeReleasePin , OUTPUT);
  pinMode(R_B_brakeReleasePin , OUTPUT);
  pinMode(L_F_brakeReleasePin , OUTPUT);
  pinMode(L_B_brakeReleasePin , OUTPUT);
}

void loop() {
  Movement = Console.read();
  switch (Movement) {
    case 'F':
      MoveForward();
      Brake();
      delay(50);
      ReleaseBrake () ;
      break;
    case 'B':
      MoveBackward();
      Brake();
      delay(50);
      ReleaseBrake () ;
      break;
    case 'L':

      TurnLeft();
      Brake();
      delay(50);
      ReleaseBrake () ;
      break;
    case 'R':
      TurnRight();
      Brake();
      delay(50);
      ReleaseBrake () ;
      break;



    case 'W':
      MoveForwardLong();
      Brake();
      delay(50);
      ReleaseBrake () ;
      break;
    case 'S':
      MoveBackwardLong();

      Brake();
      delay(50);
      ReleaseBrake () ;
      break;
    case 'A':
      TurnLeftLong();
      Brake();
      delay(50);
      ReleaseBrake () ;
      break;
    case 'D':
      TurnRightLong();
      Brake();
      delay(50);
      ReleaseBrake () ;
      break;
  }
}



void MoveForwardLong() {

  digitalWrite(R_F_DirectionPin, HIGH);
  digitalWrite(R_B_DirectionPin, HIGH);
  digitalWrite(L_F_DirectionPin, HIGH);
  digitalWrite(L_B_DirectionPin, HIGH);
  ReleaseBrake ();

  for (int j = 40 ; j < 80 ; j=j+1) {
    analogWrite(R_F_motorPin , j);
    analogWrite(R_B_motorPin , j);
    analogWrite(L_F_motorPin , j);
    analogWrite(L_B_motorPin , j);
    delay(10);
  }
  delay(DelayTimeForwardLong);   // Move for DelayTime milliseconds
  ReleaseBrake () ;
}



void MoveForward() {

  digitalWrite(R_F_DirectionPin, HIGH);
  digitalWrite(R_B_DirectionPin, HIGH);
  digitalWrite(L_F_DirectionPin, HIGH);
  digitalWrite(L_B_DirectionPin, HIGH);
  ReleaseBrake ();

 for (int j = 75 ; j < 82 ; j=j+5) {
    analogWrite(R_F_motorPin , j);
    analogWrite(R_B_motorPin , j);
    analogWrite(L_F_motorPin , j);
    analogWrite(L_B_motorPin , j);
    delay(50);
  }
// delay(DelayTimeForward);   // Move for DelayTime milliseconds
  delay(400);   
  Brake();
  delay(50);

  ReleaseBrake () ;
}

void MoveBackwardLong() {

  digitalWrite(R_F_DirectionPin, LOW);
  digitalWrite(R_B_DirectionPin, LOW);
  digitalWrite(L_F_DirectionPin, LOW);
  digitalWrite(L_B_DirectionPin, LOW);

  ReleaseBrake ();

  for (int j = 40 ; j < 90 ; j=j+3) {
    analogWrite(R_F_motorPin , j);
    analogWrite(R_B_motorPin , j);
    analogWrite(L_F_motorPin , j);
    analogWrite(L_B_motorPin , j);
    delay(10);
  }
  delay(DelayTimeBackLong);
  ReleaseBrake () ;
}

void MoveBackward() {

  digitalWrite(R_F_DirectionPin, LOW);
  digitalWrite(R_B_DirectionPin, LOW);
  digitalWrite(L_F_DirectionPin, LOW);
  digitalWrite(L_B_DirectionPin, LOW);

  ReleaseBrake ();

  for (int j = 40 ; j < 90 ; j=j+3) {
    analogWrite(R_F_motorPin , j);
    analogWrite(R_B_motorPin , j);
    analogWrite(L_F_motorPin , j);
    analogWrite(L_B_motorPin , j);
    delay(10);
  }
  delay(DelayTimeBack);   // Move for DelayTime milliseconds
  Brake();
  delay(50);
  ReleaseBrake () ;
}

void TurnLeft() {
  digitalWrite(R_F_DirectionPin, HIGH);
  digitalWrite(R_B_DirectionPin, HIGH);
  digitalWrite(L_F_DirectionPin, LOW);
  digitalWrite(L_B_DirectionPin, LOW);

  ReleaseBrake ();
  for (int j = 40 ; j < 90 ; j=j+3) {
    analogWrite(R_F_motorPin , j);
    analogWrite(R_B_motorPin , j);
    analogWrite(L_F_motorPin , j);
    analogWrite(L_B_motorPin , j);
    delay(20);
  }

  delay(DelayTimeLeft);   // Move for DelayTime milliseconds
  Brake();
  delay(50);
  ReleaseBrake () ;
}
void TurnLeftLong() {
  digitalWrite(R_F_DirectionPin, HIGH);
  digitalWrite(R_B_DirectionPin, HIGH);
  digitalWrite(L_F_DirectionPin, LOW);
  digitalWrite(L_B_DirectionPin, LOW);

  ReleaseBrake ();
  for (int j = 40 ; j < 90 ; j=j+3) {
    analogWrite(R_F_motorPin , j);
    analogWrite(R_B_motorPin , j);
    analogWrite(L_F_motorPin , j);
    analogWrite(L_B_motorPin , j);
    delay(10);
  }
  delay(DelayTimeLeftLong);
  ReleaseBrake () ;
}
void TurnRight() {

  digitalWrite(R_F_DirectionPin, LOW);
  digitalWrite(R_B_DirectionPin, LOW);
  digitalWrite(L_F_DirectionPin, HIGH);
  digitalWrite(L_B_DirectionPin, HIGH);

  ReleaseBrake ();
  for (int j = 40 ; j < 90 ; j=j=j+1) {
    analogWrite(R_F_motorPin , j);
    analogWrite(R_B_motorPin , j);
    analogWrite(L_F_motorPin , j);
    analogWrite(L_B_motorPin , j);
    delay(10);
  }

  delay(DelayTimeRight);   // Move for DelayTime milliseconds
  Brake();
  delay(50);
  ReleaseBrake () ;
}
void TurnRightLong() {

  digitalWrite(R_F_DirectionPin, LOW);
  digitalWrite(R_B_DirectionPin, LOW);
  digitalWrite(L_F_DirectionPin, HIGH);
  digitalWrite(L_B_DirectionPin, HIGH);

  ReleaseBrake ();
  for (int j = 40 ; j < 90 ; j=j+3) {
    analogWrite(R_F_motorPin , j);
    analogWrite(R_B_motorPin , j);
    analogWrite(L_F_motorPin , j);
    analogWrite(L_B_motorPin , j);
    delay(20);
  }
  delay(DelayTimeRightLong);
  ReleaseBrake () ;
}

void ReleaseBrake () {
  digitalWrite(R_F_brakeReleasePin, HIGH);
  digitalWrite(R_B_brakeReleasePin, HIGH);
  digitalWrite(L_F_brakeReleasePin, HIGH);
  digitalWrite(L_B_brakeReleasePin, HIGH);

}
void Brake() {
  digitalWrite(R_F_brakeReleasePin, LOW);
  digitalWrite(R_B_brakeReleasePin, LOW);
  digitalWrite(L_F_brakeReleasePin, LOW);
  digitalWrite(L_B_brakeReleasePin, LOW);

  analogWrite(R_F_motorPin , 0);
  analogWrite(R_B_motorPin , 0);
  analogWrite(L_F_motorPin , 0);
  analogWrite(L_B_motorPin , 0);

}

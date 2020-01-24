int rc_channel1 = 2;
int rc_channel2 = 3;
int rc_channel3 = 4;
int rc_channel4 = 5;
int switchA = 6;
int switchB = 7;

void setup() {
  // put your setup code here, to run once:
  pinMode(rc_channel1, INPUT);
  pinMode(rc_channel2, INPUT);
  pinMode(rc_channel3, INPUT);
  pinMode(rc_channel4, INPUT);
  pinMode(switchA, INPUT);
  pinMode(switchB, INPUT);  
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int pwm1 = 0;
  int pwm2 = 0;
  int pwm3 = 0;
  int pwm4 = 0;
  int pwma = 0;
  int pwmb = 0;
  int swa = pulseIn(switchA, HIGH);
   int swb = pulseIn(switchB, HIGH);
    int rc1 = pulseIn(rc_channel1, HIGH);
     int rc2 = pulseIn(rc_channel2, HIGH);
      int rc3 = pulseIn(rc_channel3, HIGH);
       int rc4 = pulseIn(rc_channel4, HIGH);

if(rc1==0){
      Serial.println(" CH1 no signal");
  }
  else if(rc1 > 1620){ //RIGHT right stick
      pwm1 = map(rc1, 1620, 1977, 0, 255); //map our speed to 0-255 range
    
      Serial.print(" CH1 right stick speed: ");
      Serial.println(pwm1);
  }
  else if(rc1 < 1370){ //RIGHT left stick
      pwm1 = map(rc1, 1370, 990, 0, 255); //map our speed to 0-255 range
     
      Serial.print(" CH1 left stick speed: ");
      Serial.println(pwm1);
  }else{
      Serial.println(" CH1 stick centered");
  }

 if(rc2==0){
      Serial.println(" CH2 no signal");
  }
  else if(rc2 > 1638){ //RIGHT up stick
      pwm2 = map(rc2, 1638, 1971, 0, 255); //map our speed to 0-255 range

      Serial.print(" CH2 up stick speed: ");
      Serial.println(pwm2);
  }
  else if(rc2 < 1310){ // RIGHT down stick
      pwm2 = map(rc2, 1310, 987, 0, 255); //map our speed to 0-255 range

      Serial.print(" CH2 down stick speed: ");
      Serial.println(pwm2);
  }else{
      Serial.println(" CH2 stick centered");
  }
  
 if(rc3==0){
      Serial.println(" CH3 no signal");
  }
  else if(rc3 > 1632){ //LEFT up stick
      pwm3 = map(rc3, 1632, 1972, 0, 255); //map our speed to 0-255 range

      Serial.print(" CH3 up stick speed: ");
      Serial.println(pwm3);
  }
  else if(rc3 < 1320){ // LEFT down stick
      pwm3 = map(rc3, 1320, 1007, 0, 255); //map our speed to 0-255 range

      Serial.print(" CH3 down stick speed: ");
      Serial.println(pwm3);
  }else{
      Serial.println(" CH3 stick centered");
  }
  
  if(rc4==0){
      Serial.println(" CH4 no signal");
  }
  else if(rc4 > 1560){ //LEFT right stick
      pwm4 = map(rc4, 1560, 1975, 0, 255); //map our speed to 0-255 range

      Serial.print(" CH4 right stick speed: ");
      Serial.println(pwm4);
  }
  else if(rc4 < 1380){ //LEFT left stick
      pwm4 = map(rc4, 1380, 987, 0, 255); //map our speed to 0-255 range

      Serial.print(" CH4 left stick speed: ");
      Serial.println(pwm4);
  }else{
      Serial.println(" CH4 stick centered");

  }

//Programmable Switches
if(swa > 1960){ //SWA = 1
      pwma = 255; 
      
      Serial.print(" SWA = 1, pwma: ");
      Serial.println(pwma);
  }
else{
      pwma = 0; 
      
      Serial.print(" SWA = 0, pwma: ");
      Serial.println(pwma);
  }

 if(swb > 1960){ //SWB = 1
      pwmb = 255; 
      
      Serial.print(" SWB = 1, pwmb: ");
      Serial.println(pwmb);
  }
else{
      pwmb = 0;
      
      Serial.print(" SWB = 0, pwmb: ");
      Serial.println(pwmb);
  }
  delay(1000); 
}

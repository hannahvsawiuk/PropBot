#include "rc_commands.h"
#include "pinout.h"

fetch_rc_command() 
{
    sw_a       = pulseIn(RC_SWA_CHANNEL_PIN, HIGH);
    sw_b       = pulseIn(RC_SWB_CHANNEL_PIN, HIGH);
    rc_right   = pulseIn(RC_RIGHT_CHANNEL_PIN, HIGH);
    rc_left    = pulseIn(RC_LEFT_CHANNEL_PIN, HIGH);

    if (SW_A)

}

bool stop (uint16_t switch_command)
{
    if (switch_command < RC_SWX_HIGH_MAX && switch_command > RC_SWX_HIGH_MIN) {
        return true
    } else if (switch_command < RC_SWX_LOW_)
    {
        
    }
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

/*  Hot Compost Bin Control Software
 *  James Fotherby 15th May 2022
 *  
 *  This software runs on an Arduino. It monitors the temperature and controls the air pump.
 *  
 *  We PWM the pump with 10 minute cycles. Minium duty is 5% and maximum is 100%. 
 *  At 60C the PWM is 5% and at 65C the PWM is 100%
 * 
 * Calibrations:
 * room temp 734
 * 76.0C = 264
 * 65.0C = 340
 * 58.0C = 387
 * 51.5C = 437
 * 47.0C = 480
 */

#include        "Average.h"

#define         RELAY_PIN       11
#define         TEMP_SENSE_PIN  A0
#define         TEMP_60C        372
#define         TEMP_65C        336
#define         PERIOD_TIME     600000

Average         Temperature_Filter(32);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  //Serial.println("Starting...");

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(13, OUTPUT);
}

void loop() {
  static int T_Filtered; 
  static unsigned long TimeStamp, On_Time;

  TimeStamp = millis();
  digitalWrite(RELAY_PIN, HIGH);
  digitalWrite(13, HIGH);

  while(1)
  {
    // Take 32 Temperature readings and take an average
    for(int i = 0; i < 32; i++) {
      int Temp_Reading = analogRead(TEMP_SENSE_PIN); 
      T_Filtered = Temperature_Filter.Rolling_Average(Temp_Reading);
      delay(30);
    }

    T_Filtered = constrain(T_Filtered, TEMP_65C, TEMP_60C);
    On_Time = map(T_Filtered, TEMP_60C, TEMP_65C, 30, 600);

    long Time_Remaining = (On_Time*1000) + TimeStamp - millis();
    if(Time_Remaining < 0)
      break; 

    Serial.print(T_Filtered); Serial.print(", Remaining Time: "); Serial.println(Time_Remaining/1000); 
  }

  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(13, LOW);
 
  while(1)  {
    long Time_Remaining = TimeStamp + PERIOD_TIME - millis();    
    if(Time_Remaining < 0)
      break;

    delay(1000);
    Serial.print("Remaining Time: "); Serial.println(Time_Remaining/1000); 
  }
}

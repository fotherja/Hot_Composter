/*  Hot Compost Bin Control Software
 *  James Fotherby 15th May 2022
 *  
 *  This software runs on an Arduino. It monitors the temperature of the compost 
 *
 *  1) A periodic function takes readings of the compost and surrounding air temps and keeps a 32 value average
 *
 *  2) The pumps switch on in turn driving water back and forth between two sealed containers. 
 *     One-Way valves ensure that air is always sucked from the compost and blown out of a vent
 *     Float switches are triggered when the water level reaches a max or min in the containers
 *     
 * 
 *  To Do:
 *  - Adjust airflow to maintain appropiate temperature of 60C
 *  - At lower temperatures, some airflow is still needed but less due to the slower rate of metabolism. 
 *  - A metabolic rate of 350W needs approximately 1L of O2 a minute which is an aiflow of 5L/minute. 
 *  - A metabolic rate of 50W needs approximately 1L of O2 every 7 minutes
 *  - 1 pump stroke is ~2.5L of air moved. At max pumping speed, air is moved at ~6L/min 
 *  - Assuming our composter can operate between about 350W down to 50W we need our airflow to vary between Max (@6L/min) and a minimum of ~1L/Min
 *
 * I think the best way to do this is for each 5 minute interval. Start by doing all the pumping needed for that interval and then pause.
 * 
 */

#include        "Average.h"

#define         PUMP_FILL_PIN             3
#define         PUMP_DRAIN_PIN            4

#define         FLOAT_MAX_PIN             5
#define         FLOAT_MIN_PIN             6

#define         COMPOST_TEMP_SENSE_PIN    A2
#define         ENCLOSURE_TEMP_SENSE_PIN  A3

#define         CS_PIN                    10
#define         LED_PIN                   13
//------------------------------------------------------------------
#define         PUMP_TIMEOUT              23000
#define         MIN_PUMP_TIME_TO_REGISTER 5000

int             Convert_to_Temp(int ADC_Reading);

Average         Compost_Temperature_Filter(32), Enclosure_Temperature_Filter(32);
int             Compost_Temperature, Enclosure_Temperature;

//---------------------------------------------------------------------------------------------------------------------------------------
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(FLOAT_MAX_PIN, INPUT_PULLUP);
  pinMode(FLOAT_MIN_PIN, INPUT_PULLUP);

  pinMode(PUMP_FILL_PIN, OUTPUT);
  pinMode(PUMP_DRAIN_PIN, OUTPUT);

  noInterrupts();           // disable all interrupts
  TCCR1A = (1<<WGM11);
  TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS12);   
  ICR1 = 7812;
  TIMSK1 = (1<<TOIE1); 
  interrupts();             // enable all interrupts
}

//---------------------------------------------------------------------------------------------------------------------------------------
ISR(TIMER1_OVF_vect)        // interrupt service routine to take temperature measurments 
{
  Compost_Temperature_Filter.Rolling_Average(analogRead(COMPOST_TEMP_SENSE_PIN));
  Enclosure_Temperature_Filter.Rolling_Average(analogRead(ENCLOSURE_TEMP_SENSE_PIN));
}

//---------------------------------------------------------------------------------------------------------------------------------------
void loop() 
{
  static long Pump_Strokes = 0;
  unsigned int Pump_On_Timer = 0;

  // Fill Container A
  Pump_On_Timer = 0;
  digitalWrite(PUMP_FILL_PIN, HIGH);
  while(digitalRead(FLOAT_MIN_PIN) == 0 && Pump_On_Timer < PUMP_TIMEOUT )
  {
    delay(10);
    Pump_On_Timer += 10;
  }

  digitalWrite(PUMP_FILL_PIN, LOW);

  if(Pump_On_Timer < MIN_PUMP_TIME_TO_REGISTER)
    delay(MIN_PUMP_TIME_TO_REGISTER);
  else
    Pump_Strokes++;
  
  delay(250);

  // Display and record temperatures 
  Compost_Temperature = Convert_to_Temp(Compost_Temperature_Filter.Rolling_Avg);
  Enclosure_Temperature = Convert_to_Temp(Enclosure_Temperature_Filter.Rolling_Avg);
  Serial.print(Compost_Temperature); Serial.print(","); Serial.print(Enclosure_Temperature); Serial.print(","); Serial.println(Pump_Strokes);
  
  // Fill container B
  Pump_On_Timer = 0;
  digitalWrite(PUMP_DRAIN_PIN, HIGH);
  while(digitalRead(FLOAT_MAX_PIN) == 0 && Pump_On_Timer < PUMP_TIMEOUT)
  {
    delay(10);
    Pump_On_Timer += 10;
  }

  digitalWrite(PUMP_DRAIN_PIN, LOW);

  if(Pump_On_Timer < MIN_PUMP_TIME_TO_REGISTER)
    delay(MIN_PUMP_TIME_TO_REGISTER);
  else
    Pump_Strokes++;

  delay(250);

  // Display and record temperatures 
  Compost_Temperature = Convert_to_Temp(Compost_Temperature_Filter.Rolling_Avg);
  Enclosure_Temperature = Convert_to_Temp(Enclosure_Temperature_Filter.Rolling_Avg);
  Serial.print(Compost_Temperature); Serial.print(","); Serial.print(Enclosure_Temperature); Serial.print(","); Serial.println(Pump_Strokes);
}

//---------------------------------------------------------------------------------------------------------------------------------------
int Convert_to_Temp(int ADC_Reading)
{
  const float A = 118.0;
  const float B = -4.16e-1;
  const float C = 7.56e-4;
  const float D = -5.67e-7;

  float X = (float)ADC_Reading;

  float Y = A + B*X + C*X*X + D*X*X*X;
  Y *= 10.0;
  return((int)round(Y));
}






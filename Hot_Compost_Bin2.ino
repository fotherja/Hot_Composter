/*  Hot Compost Bin Control Software
 *  James Fotherby 15th May 2022
 *  
 *  This software runs on an Arduino. It monitors the temperature of the compost and adjusts the airflow to maintain appropiate temperatures
 *
 *  1) A periodic function takes readings of the compost and surrounding air temps and keeps a 32 value average
 *  2)
 * 
 * 
 * Calibrations:
 * room temp 734
 * 76.0C = 264
 * 65.0C = 340
 * 58.0C = 387
 * 51.5C = 437
 * 47.0C = 480
 * 
 * 
 * 10K thermostat: 19C = 756
 */

#include        "Average.h"
#include        <SPI.h>
#include        <SD.h>

#define         PUMP_FILL_PIN             3
#define         PUMP_DRAIN_PIN            4

#define         FLOAT_MAX_PIN             5
#define         FLOAT_MIN_PIN             6

#define         COMPOST_TEMP_SENSE_PIN    A2
#define         ENCLOSURE_TEMP_SENSE_PIN  A3

#define         CS_PIN                    10
#define         LED_PIN                   13
//------------------------------------------------------------------
#define         TEMP_60C                  400     //372
#define         TEMP_65C                  336
#define         MIN_PUMP_PERIOD           6
#define         PERIOD_TIME               600
#define         PERIOD_TIME_MS            600000
#define         LOG_PERIOD                300000


void            Log_SD_Card(int Compost_Temp, int Enclosure_Temp, int Pump_Strokes);
int             Convert_to_Temp(int ADC_Reading);

Average         Compost_Temperature_Filter(32), Enclosure_Temperature_Filter(32);
int             Compost_Temperature, Enclosure_Temperature;

int             SD_Card_Usable_Flag;

//---------------------------------------------------------------------------------------------------------------------------------------
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(FLOAT_MAX_PIN, INPUT_PULLUP);
  pinMode(FLOAT_MIN_PIN, INPUT_PULLUP);

  pinMode(PUMP_FILL_PIN, OUTPUT);
  pinMode(PUMP_DRAIN_PIN, OUTPUT);

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(CS_PIN)) {
    Serial.println("Card failed, or not present");
    SD_Card_Usable_Flag = 0;
  }
  else  {
    Serial.println("card initialized.");
    SD_Card_Usable_Flag = 1;
  }

  noInterrupts();           // disable all interrupts
  TCCR1A = (1<<WGM11);
  TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS12);   
  ICR1 = 7812;
  TIMSK1 = (1<<TOIE1); 
  interrupts();             // enable all interrupts
}

//---------------------------------------------------------------------------------------------------------------------------------------
ISR(TIMER1_OVF_vect)        // interrupt service routine 
{
  Compost_Temperature_Filter.Rolling_Average(analogRead(COMPOST_TEMP_SENSE_PIN));
  Enclosure_Temperature_Filter.Rolling_Average(analogRead(ENCLOSURE_TEMP_SENSE_PIN));
}

//---------------------------------------------------------------------------------------------------------------------------------------
void loop() 
{
  static long Pump_Strokes = 0;

  // Fill Container A
  Serial.println("Filling A");
  digitalWrite(PUMP_FILL_PIN, HIGH);
  while(FLOAT_MAX_PIN == 0)
  {
    delay(250);
  }

  digitalWrite(PUMP_FILL_PIN, LOW);
  Pump_Strokes++;
  delay(250);

  // Fill container B
  Serial.println("Filling B");
  digitalWrite(PUMP_DRAIN_PIN, HIGH);
  while(FLOAT_MIN_PIN == 0)
  {
    delay(250);
  }

  digitalWrite(PUMP_DRAIN_PIN, LOW);
  Pump_Strokes++;
  delay(250);

  // Display and record temperatures 
  Compost_Temperature = Convert_to_Temp(Compost_Temperature_Filter.Rolling_Avg);
  Enclosure_Temperature = Convert_to_Temp(Enclosure_Temperature_Filter.Rolling_Avg);
  Serial.print(Compost_Temperature); Serial.print(", "); Serial.print(Enclosure_Temperature); Serial.print(", "); Serial.println(Pump_Strokes);

  Log_SD_Card(Compost_Temperature, Enclosure_Temperature, Pump_Strokes);  
}


//---------------------------------------------------------------------------------------------------------------------------------------
void Log_SD_Card(int Compost_Temp, int Enclosure_Temp, int Pump_Strokes)
{  
  if(SD_Card_Usable_Flag == 0)  {
    return;
  }

  static unsigned long Minutes_Since_Reboot = 0;
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= LOG_PERIOD) {
    previousMillis = currentMillis;

    String dataString = "";
    dataString += String(Minutes_Since_Reboot);
    dataString += ",";
    dataString += String(Compost_Temp);
    dataString += ",";
    dataString += String(Enclosure_Temp);
    dataString += ",";
    dataString += String(Pump_Strokes);
      
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
  
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
      // print to the serial port too:
      Serial.println(dataString);
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening datalog.txt");
    }

    Minutes_Since_Reboot + 5;
  }    
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
  return((int)round(Y));
}





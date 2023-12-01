/*  Hot Compost Bin Control Software
 *  James Fotherby 9th November 2023
 *  
 *  This software runs on an Arduino. It:
 *  - monitors the temperature of the compost and outdoor environment and transmits this data over LoRa to an ESP32 connected to wifi for storage in InfluxDB.
 *  - Runs stepper motors that drive the volumetric air circulation pump
 *  - Adjusts the air intake valve which is driven by a servo motor - Adjusts % of fresh vs recircualted air.
 *
 *  2 Stepper motors drive pistons to circulate air through the compost media. 200 steps pre revolution. 1/16 microstepping enabled, 500ml air pumped per revolution
 *  A differential pressure sensor measures the flow of air pumped into the compost
 *  A servo motor adjusts the percentage of vented to recirculated air. All vented air will obviously draw the equivalent volume of fresh air into the composter
 *  We have 1-Wire temperature sensors measuring:
 *  1) outdoor temperature, 2) Heat exchanger flow temperature, 3) Heat exchanger return temperature,  4) Central compost temperature 
 *
 *  We have a liquid flow sensor measuring the flow of heated water out of the composter
 *  We have a servo motor that adjusts the flow of water circulated out of the composter relative to the flow recirculating through the heat exchanger
 *  
 *  A vacuum pump switches on from time to time to remove effluent liquid from the base of the composter
 *
 * General operation:
 * The pumps are run to circulate air at a continuous rate. (Currently set to 11520 steps/second with 1/16 microstepping and 200 steps/rev and gear ratio of (15/36)^2 -> 1600ms per revolution with 35L/min max recirculation rate)
 * Air is vented at about 5L/min (Metabolically 1L of 100% O2 is needed per 20kJ of heat made) So 5L/min is enough for 160W assuming a 10% O2 extraction rate from the air
 * Air venting is under PI control to adjust a 3-way valve based on the measured flow rates
 * We send data points to LoRa   

Non[] = {0x28, 0x84, 0x15, 0xEE, 0xB0, 0x22, 0x07, 0xB5};+
Gre[] = {0x28, 0xF8, 0xB3, 0xD9, 0xB0, 0x22, 0x07, 0x31};
Red[] = {0x28, 0xEF, 0x04, 0x01, 0xB1, 0x22, 0x07, 0x47};
Blu[] = {0x28, 0x07, 0xE9, 0xE3, 0xB0, 0x22, 0x07, 0x1C};
Sil[] = {0x28, 0x79, 0x3A, 0xDF, 0x0D, 0x00, 0x00, 0xA2};

*/

#include <SPI.h>
#include <LoRa.h>
#include <DS18B20.h>
#include "J_Stepper.h"
#include "DFRobot_LWLP_J.h"
#include <ServoTimer2.h>
#include <MovingAverage.h>
#include <PID_v1.h>

//--- Pin Definitions ---------------------------------------------------------------
#define         INT0_ONE_WIRE                 2
#define         INT1_FLOW_SENSOR              3
#define         LORA_NSS                      4       
#define         LORA_NRESET                   5
#define         LORA_DIO                      6

#define         OCR1A_STEPPER_1               9       
#define         OCR1A_STEPPER_2               10
#define         MOSI_LORA                     11       
#define         MISO_LORA                     12
#define         SCK_LORA                      13

#define         STEPPER_ENABLE                A0
#define         SERVO1_PIN                    A1               
#define         SERVO2_PIN                    A2
#define         SERVO3_PIN                    A3
#define         SDA                           A4
#define         SCK                           A5

//--- Other Defines -------------------------------------------------------------------
#define         DATA_TX_INTERVAL              60000
#define         READ_SENSORS_INTERVAL         6400 
#define         PRESSURE_INTERVAL             16                                                  
#define         NUMBER_OF_DS18B20             5

#define         AIR_FLOW_SETPOINT             28000

uint8_t         DS18B20_Adr[NUMBER_OF_DS18B20][8] = { 
                                              {0x28, 0x84, 0x15, 0xEE, 0xB0, 0x22, 0x07, 0xB5}, 
                                              {0x28, 0xF8, 0xB3, 0xD9, 0xB0, 0x22, 0x07, 0x31},
                                              {0x28, 0xEF, 0x04, 0x01, 0xB1, 0x22, 0x07, 0x47},
                                              {0x28, 0x07, 0xE9, 0xE3, 0xB0, 0x22, 0x07, 0x1C},
                                              {0x28, 0x79, 0x3A, 0xDF, 0x0D, 0x00, 0x00, 0xA2}
};

uint8_t         LoRa_localAddress             = 0xA;                                                            // address of this device
uint8_t         LoRa_destination              = 0xB;                                                            // destination to send to

DS18B20         ds(2);
J_Stepper       Steppers;                                                                                       // Uses Timer1 for square wave output
ServoTimer2     Servo1, Servo2, Servo3;
DFRobot_LWLP_J  lwlp;
uint16_t        Servo3_Position = 850;                                                                          // Front = 1850, Rear = 850, 1000 = slight vent
double          AirSetpoint = AIR_FLOW_SETPOINT, AirInput, AirOutput;
double          AirKp=8.0e-4, AirKi=4.0e-4, AirKd=0.0;
PID             AirPID(&AirInput, &AirOutput, &AirSetpoint, AirKp, AirKi, AirKd, REVERSE);

struct Sensor_Data_t {
                int16_t Temp[NUMBER_OF_DS18B20];
                uint16_t Air_Flow_ml_per_min;
} Sensor_Data;

MovingAverage   <int16_t, 8> filter[NUMBER_OF_DS18B20];
MovingAverage   <uint32_t, 8> Pressure_Filter;

void sendMessage();
uint8_t calculateChecksum(const uint8_t* data, size_t length);

//---------------------------------------------------------------------------------------------------------------------------------------
void setup() {
  // ---------- Configure stepper motor driver
  Steppers.Configure(OCR1A_STEPPER_1, OCR1A_STEPPER_2, STEPPER_ENABLE);   

  // ---------- Configure Serial for debugging
  Serial.begin(115200);                                                                                               
  while (!Serial);

  // ---------- Configure Pressure sensor: 
  while (lwlp.begin() != 0) {
    Serial.println("Failed to initialize the chip, please confirm the chip connection");
    delay(1000);
  }  

  // ---------- Configure LoRa:  
  LoRa.setPins(LORA_NSS, LORA_NRESET, LORA_DIO);                                                                // Configure LoRa for 433MHz
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                   
  }
  sendMessage();
  Serial.println("LoRa module initialised");

  // ---------- Servo Driver and PID controller:
  Servo3.attach(SERVO3_PIN);                                                                                   
  Servo3.write(Servo3_Position);  
  AirPID.SetMode(AUTOMATIC);
  AirPID.SetOutputLimits(850, 1850);
  AirPID.SetSampleTime(READ_SENSORS_INTERVAL);

  // ---------- Start steppers:
  Steppers.Enable_H_Bridge();
  Steppers.Ramp_to_Speed(11520);                                                                                //11520 results in smoothest operation and 1600ms pre rev
  while(Steppers.Current_Steps_per_Second() != 11520)   {
    Steppers.Service();
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------
void loop() 
{
  static unsigned long PrevMillis_A = millis();
  static unsigned long PrevMillis_B = millis();
  static unsigned long PrevMillis_C = millis();
  
  //--- Send Data over LoRa --------------------------------
  if(millis() - PrevMillis_A >= DATA_TX_INTERVAL)  {
    PrevMillis_A += DATA_TX_INTERVAL;
    sendMessage();
    //Serial.println("LoRa packet sent");
  }

  //--- Read Air flow and temperature sensors ---------------------------------------
  if(millis() - PrevMillis_B >= READ_SENSORS_INTERVAL)  {
    PrevMillis_B += READ_SENSORS_INTERVAL;

    Servo3.detach();

    // Read 2 revolutions worth of air flow readings
    PrevMillis_C = millis();
    float Pressure_Average = 0;  
    DFRobot_LWLP_J::sLwlp_t data;  
    uint8_t i = 0;

    while(i < 200) {         
      if(millis() - PrevMillis_C >= PRESSURE_INTERVAL)  {
        PrevMillis_C += PRESSURE_INTERVAL;
        
        data = lwlp.getData();                                                                                     // Takes 11.5ms to do a reading  (when delay set to 10ms but it's set to 12ms)
        Pressure_Average += data.presure;
        i++;
      }      
    }

    Pressure_Average *= 2.179;                                                                                     // / 200 and also k=435.8 calibration constant
    Sensor_Data.Air_Flow_ml_per_min = (uint16_t)Pressure_Filter.add((uint32_t)Pressure_Average);

    // Now read our temperature sensors  
    for(uint8_t i = 0; i < NUMBER_OF_DS18B20; i++)  {
      if(ds.select(DS18B20_Adr[i]))
        Sensor_Data.Temp[i] = filter[i].add((int16_t)(ds.getTempC() * 128.0)); 
      else 
        Sensor_Data.Temp[i] = 0;          
    }

    //Serial.println("Sensors read");
    Serial.println(Sensor_Data.Air_Flow_ml_per_min);  

    // Control our servo to introduce fresh air
    AirInput = (float)Sensor_Data.Air_Flow_ml_per_min;
    AirPID.Compute();
    Servo3_Position = (uint16_t)AirOutput;

    Servo3.attach(SERVO3_PIN);                                                                                   
    Servo3.write(Servo3_Position);     
  }
} 

//---------------------------------------------------------------------------------------------------------------------------------------
void sendMessage() 
{
  uint8_t checksum = calculateChecksum((uint8_t*)&Sensor_Data, sizeof(Sensor_Data));

  LoRa.beginPacket();                                                                                           // start packet
  LoRa.write(LoRa_destination);                                                                                 // add destination address
  LoRa.write(LoRa_localAddress);                                                                                // add sender address
  LoRa.write(sizeof(Sensor_Data));
  LoRa.write((uint8_t*)&Sensor_Data, sizeof(Sensor_Data));                                                      // add payload data
  LoRa.write(checksum);                                                                                         // Checksum of data
  LoRa.endPacket();                                                                                             // finish packet and send it
}

//---------------------------------------------------------------------------------------------------------------------------------------
uint8_t calculateChecksum(const uint8_t* data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}


















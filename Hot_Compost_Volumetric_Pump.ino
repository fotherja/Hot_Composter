/*  Hot Compost Bin Control Software
 *  James Fotherby 9th November 2023
 *  
 *  This software runs on an Arduino. It monitors the temperature of the compost and outdoor environment and transmits this data to a InfluxDB database.
 *
 *  2 Stepper motors drive pistons to circulate air through the compost media. 180 steps pre revolution. 1/16 microstepping enabled, 500ml air pumped per revolution
 *  A differential pressure sensor measures the flow of air pumped into the compost
 *  A servo motor adjusts the percentage of vented to recirculated air. All vented air will obviously draw the equivalent volume of fresh air into the composter
 *  We have 1-Wire temperature sensors measuring:
 *  1) outdoor temperature
 *  2) Heat exchanger flow temperature
 *  3) Heat exchanger return temperature
 *  4) Central compost temperature 
 *
 *  We have a liquid flow sensor measuring the flow of heated water out of the composter
 *  We have a servo motor that adjusts the flow of water circulated out of the composter relative to the flow recirculating through the heat exchanger
 *  
 *  We have a LoRa module connectedfor data transmission 
 *  A vacuum pump switches on from time to time to remove effluent liquid from the base of the composter
 *
 * General operation:
 * The pumps are run to circulate air at a continuous rate. Say 25L/Min. They do a rotation then sleep for a few seconds until the next rotation
 * Air is vented to introduce 1L O2 per 20kJ of heat made
 * We send data points to LoRa
 * we adjust cooling using a very slow PI controller and keep track of heat energy    

To Do
 - Set up timer 1 for stepper motor driving on Timer 1. Ramp up, steady then down in speed over a given number of steps.

Stepper_Steps(), Stepper_Ramp_To_Speed()
  - Sets stepper start frequency and ramp intervals
  - Each ramp interval increment stepper speed until target speed reached then set interval until time to down ramp
  - Do same for down ramp

Stepper_Sync
  We need a way to synchronise the steppers so they are 180 degrees out of phase. Hopefully they never desync. 
  I reckon we rotate 1 stepper and look at the flow waveform on the pressures and find the maximum flow rate and corrosponding step count.

Servo_Position
 - Set up timer 2 for servo driving
 - Interface with all respective sensors and LoRa

----------Pins-------------
D0  - 
D1  - 
D2  - INT0 - 1-wire
D3  - INT1 - Flow sensor
D4  - LORA NSS
D5  - LORA NRESET
D6  - LORA DIO0
D7  -
D8  - 
D9  - OCR1A Stepper 1
D10 - OCR1B Stepper 2
D11 - MOSI - LORA
D12 - MISO - LORA
D13 - SCK - LORA

A0  - Servo 1
A1  - Servo 2
A2  - Servo 3
A3  - Stepper Sleep 
A4  - SDA - Pressure Sensor
A5  - SCK - Pressure Sensor
*/

#include <SPI.h>
#include <LoRa.h>

#include <DS18B20.h>
#include "Wire.h"
#include "SHT31.h"

//--- Pin Definitions ---------------------------------------------------------------
#define         INT0_ONE_WIRE             2
#define         INT1_FLOW_SENSOR          3
#define         LORA_NSS                  4       
#define         LORA_NRESET               5

#define         OCR1A_STEPPER_1           9       
#define         OCR1A_STEPPER_2           10
#define         MOSI_LORA                 11       
#define         MISO_LORA                 12
#define         SCK_LORA                  13

#define         SERVO1                    A0               
#define         SERVO2                    A1
#define         SERVO3                    A2
#define         STEPPER_SLP               A3
#define         SDA                       A4
#define         SCK                       A5

#define         SHT31_ADDRESS             0x44

uint8_t         DS18B20_Adr[]             = {40, 250, 31, 218, 4, 0, 0, 52};

SHT31           sht;
DS18B20         ds(2);

const long frequency = 915E6;  // LoRa Frequency

const int csPin = 10;          // LoRa radio chip select
const int resetPin = 9;        // LoRa radio reset
const int irqPin = 2;          // change for your board; must be a hardware interrupt pin
//---------------------------------------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);                                                                                 // Configure Serial                
  while (!Serial);

  LoRa.setPins(csPin, resetPin, irqPin);                                                                // Configure LoRa
  if (!LoRa.begin(frequency)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                   
  }

  //ds.select(DS18B20_Adr);
  //ds.setResolution(12);
  Serial.println("DS configured");

  //Wire.begin();
  //sht.begin(SHT31_ADDRESS);
  //Wire.setClock(100000);

}

//---------------------------------------------------------------------------------------------------------------------------------------
void loop() 
{

}







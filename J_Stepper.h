#include "Arduino.h"

class J_Stepper 
{
  public:
    J_Stepper(void);

    void Configure(uint8_t Stepper_1_Pin, uint8_t Stepper_2_Pin, uint8_t Stepper_Enable_Pin);
    void Ramp_to_Speed(int Steps_per_Second);

    void Disable_H_Bridge();
    void Enable_H_Bridge();
    void Hold();
    void Unhold();

    void Set_Speed(int Steps_per_Second);

    int Current_Steps_per_Second();

    void Service();

    const int MIN_STEP_RATE         = 125;  
    const int MAX_STEP_RATE         = 20000;   

    int STEPPER_SPEED_RAMP_INTERVAL = 5;
    int MAX_DELTA_V                 = 400;
    int MIN_DELTA_V                 = -400;

    int Current_Steps_per_second   = MIN_STEP_RATE;
    int Desired_Steps_per_second   = MIN_STEP_RATE;
  
  private:
    unsigned long PreviousMillis_Stepper_Ramp = 0;
    int Stepper_Enable_Pin_;
};

J_Stepper::J_Stepper(void)
{
}

void J_Stepper::Configure(uint8_t Stepper_1_Pin, uint8_t Stepper_2_Pin, uint8_t Stepper_Enable_Pin)
{
  ICR1 = 64000; 
  TCCR1A = 0b01010000;
  TCCR1B = 0b00011001;                                                        // 16MHz XO, No prescaler, Toggle OCR1x on matched with ICR1 as our max count. 1/16 microstepping ICR1 = 1000 == 500 full steps/second                                                
  OCR1A = 0; OCR1B = 0;

  // Configure Pins
  digitalWrite(Stepper_Enable_Pin, HIGH);

  pinMode(Stepper_1_Pin, OUTPUT);
  pinMode(Stepper_2_Pin, OUTPUT);
  pinMode(Stepper_Enable_Pin, OUTPUT);

  Stepper_Enable_Pin_ = Stepper_Enable_Pin;
}


void J_Stepper::Ramp_to_Speed(int Steps_per_Second)
{
    Desired_Steps_per_second = constrain(Steps_per_Second, 0, MAX_STEP_RATE);
}

void J_Stepper::Disable_H_Bridge()
{
    digitalWrite(Stepper_Enable_Pin_, HIGH);
}

void J_Stepper::Enable_H_Bridge()
{
    digitalWrite(Stepper_Enable_Pin_, LOW);
}

void J_Stepper::Hold()
{
    TCCR1A = 0b00000000;
}

void J_Stepper::Unhold()
{
    TCCR1A = 0b01010000;
}

void J_Stepper::Set_Speed(int Steps_per_Second)
{
  Unhold();
  Current_Steps_per_second = constrain(Steps_per_Second, MIN_STEP_RATE, MAX_STEP_RATE);

  long X = 8000000 / Current_Steps_per_second;
  ICR1 = (uint16_t)X; 
}

int J_Stepper::Current_Steps_per_Second()
{
  return(Current_Steps_per_second);
}

void J_Stepper::Service()
{
  unsigned long CurrentMillis_Stepper_Ramp = millis();
  if((unsigned long)(CurrentMillis_Stepper_Ramp - PreviousMillis_Stepper_Ramp) >= STEPPER_SPEED_RAMP_INTERVAL)
  {
    PreviousMillis_Stepper_Ramp = CurrentMillis_Stepper_Ramp;

    int Delta_v = Desired_Steps_per_second - Current_Steps_per_second;

    if(Delta_v != 0)
    {
      Delta_v = constrain(Delta_v, MIN_DELTA_V, MAX_DELTA_V);
      Current_Steps_per_second += Delta_v;  

      if(Current_Steps_per_second < MIN_STEP_RATE)
      {
        Hold();
      }
      else
      {
        Unhold();
        long X = 8000000 / Current_Steps_per_second;
        ICR1 = (uint16_t)X;          
      }    
    } 
  }
}































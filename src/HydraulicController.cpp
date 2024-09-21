#include <Arduino.h>

#include "HydraulicController.h"
#include "PinDefinitions.h"
#include "Debug.h"


HydraulicController::HydraulicController()
{
   
}

HydraulicController::~HydraulicController()
{
}

void HydraulicController::Init()
{
    //Variable Valve Pin config
    pinMode(Valve_Enable, OUTPUT);
    pinMode(Valve_Dir, OUTPUT);
    pinMode(Valve_PWM, OUTPUT);
    pinMode(Valve_Fault, INPUT_PULLUP);
    pinMode(Valve_CurrentMonitor, INPUT); //Analog sensor

    //PWM
    pinMode(M1EN, OUTPUT);
    pinMode(M1DIR, OUTPUT);
    pinMode(M1PWM, OUTPUT);
    pinMode(M1DIAG, INPUT_PULLUP);
    pinMode(M1OCM, INPUT); //Analog sensor

    //Pump Control
    pinMode(PumpMotor_Relay, OUTPUT);//Relay to control hydraulic pump
    pinMode(PumpMotor_VoltagePresent, INPUT_PULLUP); //Confirms Pump Has power
    pinMode(PumpMotor_Presure, INPUT_PULLUP); //Sensor for Overpresure
    pinMode(PumpMotor_FluidLow, INPUT_PULLUP); //Sensor for low fluid
    pinMode(PumpMotor_Temp, INPUT); //Analog sensor
    
    //Locking Solenoid
    pinMode(Solenoid_Relay, OUTPUT);

    //24V Power Supply Control
    pinMode(PowerSuppy_Relay, OUTPUT);
    pinMode(PowerSuppy_VoltagePresent, INPUT);

    //Limit Switches
    pinMode(LimitSwitch_UpperMax, INPUT_PULLUP);
    pinMode(LimitSwitch_LowerMax, INPUT_PULLUP);

    ConfigPWMTimer();

    //Set the DIR pin until it can be optimized out
    digitalWrite(Valve_Dir, true);

    //Set enabled Pin to proper state
    mPWMEnabled = false;
    digitalWrite(Valve_Enable, mPWMEnabled); 

    mValvePercent = 0;
   
    mPumpOn = false;
    digitalWrite(PumpMotor_Relay, mPumpOn);
    mPumpStartTime = 0;

    mFaults = 0;
    
    SetFaultValue(HCFaults::VALVE_PWM_FAULT, !digitalRead(Valve_Fault));

    digitalWrite(M1EN, true);
    digitalWrite(M1DIR, true);
    digitalWrite(Valve_Enable, true);
    
}

void HydraulicController::SetFaultValue(HCFaults type, bool value){
    if(value){
        //Set Fault
        mFaults |= (1 << type);
    }else{
        //Clear Fault
        mFaults &= ~(1 << type);
    }
    
}

void HydraulicController::ConfigPWMTimer()
{

 //Clear default values from Config Registers
  TCCR4A = 0x00;
  TCCR4B = 0x00;

  //Configure Pin 8 to use Timer 4, setup fast PWM 
  //Prescaler = 8 = 0b010 = Max freq is 3921.16 Hz
  //Fast PWM(Top = ICR4) = WGM4 = 0b1110
  //Clear on compare of Pin 7 (OCR4B) and set on bottom = COM4B = 0b10
  //Clear on compare of Pin 8 (OCR4C) and set on bottom = COM4C = 0b10
  
  //TCCR4A = COM4A1 COM4A0 COM4B1 COM4B0 COM4C1 COM4C0 WGM41 WGM40
  TCCR4A |= (1<<COM4B1) | (1<<COM4C1) | (1<<WGM41); 
  
  //TCCR4B = ICNC4 ICES4 – WGM43 WGM42 CS42 CS41 CS40
  TCCR4B |= (1<<WGM43) | (1<<WGM42) | (1<<CS41); //Set timer4, fast PWM, prescaler to 8
  
  ICR4 = 9900; //(16,000,000Hz/200Hz/8)-1 = 9,999

  OCR4B = 0; //Set pin 7 Duty Cycle to 0% - Off
  OCR4C = 0; //Set pin 8 Duty Cycle to 0% - Off

  //if PRINT_DEBUG_PWM is defined print out registers set
  #ifdef PRINT_DEBUG_PWM
    Serial.print("\n\nTimer 4 Configured:\n");

    Serial.println("\nTCCR4A = [COM4A1, COM4A0, COM4B1, COM4B0, COM4C1, COM4C0, WGM41, WGM40]");
    Serial.print("TCCR4A: ");
    Serial.print(TCCR4A, BIN);

    
    Serial.println("\n\nTCCR4B = [ICNC4, ICES4, - , WGM43, WGM42, CS42, CS41, CS40]");
    Serial.print("TCCR4B: ");
    Serial.print(TCCR4B, BIN);

    Serial.println("\n\nTCCR4C = [FOC4A, FOC4B, FOC4C, - , - , - , - , -]");
    Serial.print("TCCR4C: ");
    Serial.print(TCCR4C, BIN);
    
    Serial.print("\n\nICR4: ");
    Serial.print(ICR4, DEC);
    Serial.print(";\tICR4 = top of timer = period\n");

    Serial.print("OCR4B: ");
    Serial.print(OCR4B, DEC);
    Serial.print(";\tOCR4B = Counter Value when pin should turn off\n");

    Serial.print("OCR4C: ");
    Serial.print(OCR4C, DEC);
    Serial.print(";\tOCR4C = Counter Value when pin should turn off\n");

  #endif //PRINT_DEBUG_PWM
  
}

int HydraulicController::GetVarValveCurrent_mA()
{
  #ifdef PRINT_DEBUG_MOTOR_CURRENT
    Serial.print("\nVariable Valve Current Consumtion = ");
    Serial.print(analogRead(Valve_CurrentMonitor) * 10, DEC);
    Serial.print("mA (raw = ");
    Serial.print(analogRead(Valve_CurrentMonitor), DEC);
    Serial.print(")\n");
  #endif //PRINT_DEBUG_MOTOR_CURRENT
  
  // 5V / 1024 ADC counts / 500 mV per A = 10 mA per count
  return analogRead(Valve_CurrentMonitor) * 10;
}


void HydraulicController::EnableValve(bool enable){
    mPWMEnabled = enable;
    digitalWrite(Valve_Enable, mPWMEnabled);

    #ifdef PRINT_DEBUG_MOTOR
        Serial.print("\nValve Enable set to: ");
        
        if(digitalRead(Valve_Enable)==1){
        Serial.print("Enabled (pin = ");
        }else{
        Serial.print("Disabled (pin = ");
        }

        Serial.print(digitalRead(Valve_Enable), DEC);
        Serial.print(")\n");

    #endif //PRINT_DEBUG_MOTOR
}

void HydraulicController::SetValve(uint16_t percent){
    //Check limits
    if(percent>100){
        percent = 100;
    }

    if(percent == mValvePercent){
        return; //no change, do nothing
    }

    mValvePercent = percent;
    
/*
    if(!mPWMEnabled){
        EnableValve(true);
    }
*/

    //Set compare register to the value to turn off
    uint16_t totalTicks = ICR4;
    uint16_t onTicks = (uint16_t)(float)totalTicks*((float)mValvePercent/100.0);
    Valve_CompareReg = onTicks;

    //Print all details of changing PWM on Pin 7 if debugging
    #ifdef PRINT_DEBUG_PWM
        Serial.print("\nValve Duty Cycle Changed to:");
        Serial.print(mValvePercent, DEC);

        Serial.print("%\nICR4: ");
        Serial.print(ICR4, DEC);
        Serial.print(";\tICR4 = top of timer = period");

        Serial.print("\nValve Compare Register: ");
        Serial.print(Valve_CompareReg, DEC);
        Serial.print(";\tOCR4n = Counter Value when pin should turn off\n");
    #endif //PRINT_DEBUG_PWM
    

}

void HydraulicController::SetPWM1(uint16_t percent){
    //Check limits
    if(percent>100){
        percent = 100;
    }

    if(percent == mPWM1Percent){
        return; //no change, do nothing
    }

    mPWM1Percent = percent;
    
/*
    if(!mPWMEnabled){
        EnableValve(true);
    }
*/

    //Set compare register to the value to turn off
    uint16_t totalTicks = ICR4;
    uint16_t onTicks = (uint16_t)(float)totalTicks*((float)mPWM1Percent/100.0);
    PWM1_CompareReg = onTicks;

    //Print all details of changing PWM on Pin 7 if debugging
    #ifdef PRINT_DEBUG_PWM
        Serial.print("\nPWM1 Duty Cycle Changed to:");
        Serial.print(mPWM1Percent, DEC);

        Serial.print("%\nICR4: ");
        Serial.print(ICR4, DEC);
        Serial.print(";\tICR4 = top of timer = period");

        Serial.print("\nPWM1 Compare Register: ");
        Serial.print(PWM1_CompareReg, DEC);
        Serial.print(";\tOCR4n = Counter Value when pin should turn off\n");
    #endif //PRINT_DEBUG_PWM
    

}

void HydraulicController::Stop(){
    
    digitalWrite(PumpMotor_Relay, false); //Turn Off Pump
    
    digitalWrite(Solenoid_Relay, false);  //Shut Off Solenoid 
    
    SetValve(0);                          //Cut Off Valve
    EnableValve(false);

}

//This function controls the Pump relay.  
//Need to consider if a timer is needed for presurization
void HydraulicController::TurnOnPump(bool on) {

    digitalWrite(PumpMotor_Relay, on);  // Set relay state regardless of change

    if (mPumpOn != on) {  // State is changing
        mPumpStartTime = millis(); // Record start time
        mPumpOn = on;  // Update state
    } 
}

unsigned long HydraulicController::GetPumpOnTimeMillis(){
    
    //Pump not on, so time on is 0
    if(!digitalRead(PumpMotor_Relay)){
        return 0; 
    }

    //current time - start time, this works in roll over too
    return millis() - mPumpStartTime; 
}

bool HydraulicController::BuildInTest(){

    uint8_t faults = mFaults;

    //ADD CODE HERE TO DO BIT 

    if(faults!=0){
        return false;
    }else{
        return true;
    }
    
}

bool HydraulicController::IsFaultPresent(HCFaults type){
    
    return false;

}

void HydraulicController::PWMFaultISR(bool state){
    
}

void HydraulicController::PumpOverPresureISR(bool state){

}

void HydraulicController::PumpPowerISR(bool state){

}

void HydraulicController::PumpFluidISR(bool state){

}

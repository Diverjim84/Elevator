#include <Arduino.h>

#include "HydraulicController.h"
#include "PinDefinitions.h"
#include "Debug.h"
#include "Globals.h"
#include "EStops.h"

HydraulicController::HydraulicController(EStops* es)
{
   Init(es);
}

HydraulicController::~HydraulicController()
{
}

void HydraulicController::Init(EStops* es)
{
    _es = es;
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
    //pinMode(PumpMotor_FluidLow, INPUT_PULLUP); //Sensor for low fluid
    pinMode(PumpMotor_Temp, INPUT); //Analog sensor
    
    //Locking Solenoid
    pinMode(Solenoid_Relay, OUTPUT);

    mPositionControlEnabled = !digitalRead(36);
    mPositionTarget = 0;    

    //24V Power Supply Control
    //pinMode(PowerSuppy_Relay, OUTPUT);
    //pinMode(PowerSuppy_VoltagePresent, INPUT);

    
    ConfigPWMTimer();

    //Set the DIR pin until it can be optimized out
    digitalWrite(Valve_Dir, true);

    //Set enabled Pin to proper state
    mPWMEnabled = false;
    digitalWrite(Valve_Enable, mPWMEnabled); 

    mValvePercent = 0;
   
    //Pump relay is active HIGH
    mPumpOn = false;
    digitalWrite(PumpMotor_Relay, mPumpOn);
    mPumpStartTime = 0;
    mElevatorState = MotionState::STOPPED;
    mTimeInMotionState = 0;

    mFaults = 0;
    
    SetFaultValue(HCFaults::VALVE_PWM_FAULT, !digitalRead(Valve_Fault));

    digitalWrite(M1EN, true);
    digitalWrite(M1DIR, true);
    digitalWrite(Valve_Enable, true);

    _lowerDoorOpen = digitalRead(DoorLock_Lower_LimitSwitch);
    _upperDoorOpen = digitalRead(DoorLock_Upper_LimitSwitch);
    _lowerTravelLS = digitalRead(LimitSwitch_LowerMax);
    _upperTravelLS = digitalRead(LimitSwitch_UpperMax);
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

void HydraulicController::UpdateValve(){
    if(!Globals::es->eStopped()){
        SetValve(mValveTarget);
    }else{
        Stop();
    }
    
}

void HydraulicController::SetValve(uint16_t percent){
    if(Globals::es->eStopped()){
        Stop();
        return;
    }
    
    //Check limits
    if(percent>100){
        Serial.print("\n***Value for Valve Exceeded: "); Serial.print(percent);Serial.print("***\n");
        percent = 100;
    }

    if(percent == mValvePercent){
        return; //no change, do nothing
    }

    mValveTarget = percent;

    if(!mPositionControlEnabled){
        uint8_t ramp = mValveRampRate;
        if(mElevatorState>MotionState::STOPPED){
            ramp = ramp*2;
        }

        if(mValvePercent < mValveTarget){
            mValvePercent += ramp;
            if(mValvePercent > mValveTarget){
                mValvePercent = mValveTarget;
            }
        }else if(mValvePercent > mValveTarget){
            mValvePercent -= ramp;
            if(mValvePercent < mValveTarget){
                mValvePercent = mValveTarget;
            }
        }
    }else{
        mValvePercent = mValveTarget;
    }

    if(!digitalRead(Valve_Enable)){
        EnableValve(true);
    }


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

void HydraulicController::SetPositionControl(bool enable){
    if(enable && !mPositionControlEnabled){
        mPositionTarget = Globals::EncoderPosition;
    }
    mPositionControlEnabled = enable;
}

void HydraulicController::SetPWM1(uint16_t percent){
    //Check limits
    if(Globals::es->eStopped()){
        Stop();
        return;
    }

    if(percent>100){
        percent = 100;
    }

    if(percent == mPWM1Percent){
        return; //no change, do nothing
    }

    mPWM1Percent = percent;
    

    if(!digitalRead(M1EN)){
        digitalWrite(M1EN, true);
    }


    //Set compare register to the value to turn off
    uint16_t totalTicks = ICR4;
    uint16_t onTicks = (uint16_t)(float)totalTicks*((float)mPWM1Percent/100.0);
    M1OCR = onTicks;

    //Print all details of changing PWM on Pin 7 if debugging
    #ifdef PRINT_DEBUG_PWM
        Serial.print("\nPWM1 Duty Cycle Changed to:");
        Serial.print(mPWM1Percent, DEC);

        Serial.print("%\nICR4: ");
        Serial.print(ICR4, DEC);
        Serial.print(";\tICR4 = top of timer = period");

        Serial.print("\nPWM1 Compare Register: ");
        Serial.print(M1OCR, DEC);
        Serial.print(";\tOCR4n = Counter Value when pin should turn off\n");
    #endif //PRINT_DEBUG_PWM
    

}

void HydraulicController::Stop(){
    
    TurnOnPump(false); //Turn Off Pump
    
    digitalWrite(M1EN, false);
    digitalWrite(Valve_Enable, false);
    SetPWM1(0);  //Shut Off Solenoid 
    SetValve(0);                          //Cut Off Valve
    mElevatorState = MotionState::STOPPED;
    EnableValve(false);
    //EnablePWM1(false);
}

//This function controls the Pump relay.  
//Need to consider if a timer is needed for presurization
void HydraulicController::TurnOnPump(bool on) {

    if(on != mPumpOn){
        if(on && !mPumpOn){ //turn on
                mPumpStartTime = millis(); //start timer
        }else{//turn off
            mPumpStartTime = 0;//reset timer
        }

        mPumpOn = on;
        digitalWrite(PumpMotor_Relay, on);   // Set relay state regardless of change
    }
    //digitalWrite(PumpMotor_Relay, on);  // Set relay state regardless of change

    //add logic to keep track of pump run time
     
}

unsigned long HydraulicController::GetPumpOnTimeMillis(){
    
    //Pump not on, so time on is 0
    if(!mPumpOn){
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

void HydraulicController::PrintMotionState(){
    switch (mElevatorState)
    {
    case MotionState::STOPPED:
        Serial.print("STOPPED");
        break;
    case MotionState::UP_PRIME:
        Serial.print("UP_PRIME");
        break;
    case MotionState::UP_INIT:
        Serial.print("UP_INIT");
        break;
    case MotionState::UP:
        Serial.print("UP");
        break;
    case MotionState::DOWN_PRIME:
        Serial.print("DOWN_PRIME");
        break;
    case MotionState::DOWN_INIT:
        Serial.print("DOWN_INIT");
        break;
    case MotionState::DOWN:
        Serial.print("DOWN");
        break;
    case MotionState::DOWN_STOP:
        Serial.print("DOWN_STOP");
        break;
    case MotionState::UP_STOP:
        Serial.print("UP_STOP");
        break;
    default:
        Serial.print("UNKNOWN");
        break;
    }
}

HydraulicController::MotionState HydraulicController::GetNewState(int8_t speed){
    
    //EStop Case
    if(_es->eStopped()){
        mElevatorState = MotionState::STOPPED;
        _es->printEstopStatus();
        Serial.println("ESTOP - STOPPED in new state");
        return MotionState::STOPPED;
    }
    
    //Over travel Protection
    if(_lowerTravelLS && speed <0){
        speed = 0;
        Serial.println("Lower Travel LS Triggered - STOPPED in new state");
    }
    if(_upperTravelLS && speed >0){
        speed = 0;
        Serial.println("Upper Travel LS Triggered - STOPPED in new state");
    }

    //Stopped
    if(speed == 0){
        //already stopped? do nothing
        if(mElevatorState == MotionState::STOPPED){
            return MotionState::STOPPED;
        }
        
        //were we going down?
        if(mElevatorState < MotionState::DOWN_STOP){
            //we were going down
            if(mElevatorState != MotionState::DOWN_STOP){
                mTimeInMotionState = millis();
                mElevatorState = MotionState::DOWN_STOP;
                return MotionState::DOWN_STOP;
            }else if(millis() - mTimeInMotionState > 500){
                mElevatorState = MotionState::STOPPED;
                mTimeInMotionState = millis();
                return MotionState::STOPPED;
            }else{
                return MotionState::DOWN_STOP;
            }
        }
        //were we going up?
        if(mElevatorState > MotionState::UP_STOP){
            //we were going up
            if(mElevatorState != MotionState::UP_STOP){
                mTimeInMotionState = millis();
                mElevatorState = MotionState::UP_STOP;
                return MotionState::UP_STOP;
            }else if(millis() - mTimeInMotionState > 700){
                mElevatorState = MotionState::STOPPED;
                mTimeInMotionState = millis();
                return MotionState::STOPPED;
            }else{
                return MotionState::UP_STOP;
            }
        }
        /*
        //if transitioning to stop, reset timer, update state
        if(mElevatorState != MotionState::STOPPED){
            mTimeInMotionState = millis();
            mElevatorState = MotionState::STOPPED;
            return MotionState::STOPPED;
        }else{//if already in stopped state, do nothing

            return MotionState::STOPPED;
        }
        */
    }
    //Go Up states
    if(speed > 0){
        
        //if transitioning to up from stop or a down state, enter up prime
        if(mElevatorState == MotionState::UP){
            //if already in up state, do nothing
            return MotionState::UP;
        }else if(mElevatorState <= MotionState::STOPPED){
            mTimeInMotionState = millis();
            mElevatorState = MotionState::UP_PRIME;
            return MotionState::UP_PRIME;
        }else if(mElevatorState == MotionState::UP_PRIME){
            //transition from up prime to up after motor is on for 1s
            if(GetPumpOnTimeMillis() > PUMP_PRIME_TIME){
                mTimeInMotionState = millis();
                mElevatorState  = MotionState::UP_INIT;
                return MotionState::UP_INIT;
            }else{ //if already in up prime state, do nothing
                return MotionState::UP_PRIME;
            }
        }else if(mElevatorState == MotionState::UP_INIT){
            if(millis()-mTimeInMotionState>0){ //delay of zero
                mTimeInMotionState = millis();
                mElevatorState   = MotionState::UP;
                return MotionState::UP;
            }else{ //do nothing while solino
                return MotionState::UP_INIT;
            }
        }
    }
    
    //Go down states
    if(speed < 0){
        
        if(mElevatorState == MotionState::DOWN){
            //do nothing, already in down state
            return MotionState::DOWN;
        }else if(mElevatorState >= MotionState::STOPPED){
            //either stopped or changing directions
            mTimeInMotionState = millis();
            mElevatorState = MotionState::DOWN_PRIME;
            return MotionState::DOWN_PRIME;
        }else if(mElevatorState == MotionState::DOWN_PRIME){
            //transition from down prime to up after motor is on for prime period
            if(GetPumpOnTimeMillis() > PUMP_PRIME_TIME){ //millis()-mTimeInMotionState > 500){
                mTimeInMotionState = millis();
                mElevatorState  = MotionState::DOWN_INIT;
                return MotionState::DOWN_INIT;
            }else{ //if already in up prime state, do nothing
                return MotionState::DOWN_PRIME;
            }
        }else if(mElevatorState == MotionState::DOWN_INIT){
            if(millis()-mTimeInMotionState>500){ //delay of zero
                mTimeInMotionState = millis();
                mElevatorState   = MotionState::DOWN;
                return MotionState::DOWN;
            }else{ //do nothing while solino
                return MotionState::DOWN_INIT;
            }
        }


    }
    return MotionState::STOPPED;//unknown State. Should never get here
}

void HydraulicController::DriveElevator(int8_t percent, uint16_t pumpPressure =0, uint16_t cylinderPressure=0){  

    MotionState state = GetNewState(percent);

    switch (state)
    {
    case MotionState::STOPPED:
        SetPWM1(0);
        SetValve(0);
        TurnOnPump(false);
        break;
    
    case MotionState::UP_PRIME:
        SetPWM1(0);
        SetValve(0);
        TurnOnPump(true);
        break;
    
    case MotionState::UP_INIT:
        SetPWM1(0);
        SetValve(percent);
        TurnOnPump(true);
        break;
    
    case MotionState::UP:
        SetPWM1(0);
        SetValve(percent);
        TurnOnPump(true);
        break;
    
    case MotionState::DOWN_PRIME:
        SetPWM1(0);
        SetValve(0);
        TurnOnPump(true);
        break;
    
    case MotionState::DOWN_INIT:
        percent = abs(percent);//get rid of negative  
        //make sure not to back drive pump
        if(mPumpOn ){
            percent = map(percent, 0, 100, 50, 0);
        }else{
            percent = map(percent, 0, 100, 100, 50);
        }
        SetPWM1(0);
        SetValve(40);
        TurnOnPump(true);
        break;

    case MotionState::DOWN:
        percent = abs(percent);//get rid of negative  
        //make sure not to back drive pump
        if(mPumpOn ){
            percent = map(percent, 0, 100, 20, 0);
        }else{
            percent = map(percent, 0, 100, 100, 0);
        }
        SetPWM1(100);
        SetValve(percent);
        TurnOnPump(true);
        break;
    
    case MotionState::DOWN_STOP:
        if(mValvePercent>30){
            SetPWM1(0);
        }else{
            SetPWM1(100);
        }
        SetValve(35);
        //no change to Valve state
        TurnOnPump(true);
        break;
    case MotionState::UP_STOP:
        SetPWM1(0);
        SetValve(0);
        //no change to Valve state
        TurnOnPump(true);
        break;
    default:
        SetPWM1(0);
        SetValve(0);
        TurnOnPump(false);
        break;
    }

}

void HydraulicController::DrivePosition(){
    if(!mPositionControlEnabled || Globals::es->eStopped()){

        return;
    }

    //do we have position?
    if(Globals::EncoderPosition < -0.5 ){
        return;
    }

    
    float pRange = 300.0;
    float tolerance = 30.0;
    float error = mPositionTarget - Globals::EncoderPosition;

    mPositionError = error;
    mPositionLastTime = millis();

    if(mElevatorState == MotionState::STOPPED ){
        if(abs(error)<tolerance){
            //we are stopped and at the target, do nothing
            TurnOnPump(false);
            SetPWM1(0);
            SetValve(0);
            return;
        }else if(error>0){
            //go up
            if(GetUpperLimitSwitch()){
                //we are at the top, do nothing
                Serial.print("\n****Can't Go up, LS active.****\n");
                return; 
            }
            mElevatorState = MotionState::UP_PRIME;
            mTimeInMotionState = millis();
            TurnOnPump(true);
            SetPWM1(0);
            SetValve(0);
            return;
        }else{
            if(GetLowerLimitSwitch()){
                //We are at the bottom, do nothing
                Serial.print("\n****Can't Go down, LS active.****\n");
                return;
            }
            mElevatorState = MotionState::DOWN_PRIME;
            mTimeInMotionState = millis();
            TurnOnPump(true);
            SetPWM1(0);
            SetValve(0);
            return;
        }
    }

    if(mElevatorState == MotionState::UP_STOP){
        if(millis()-mTimeInMotionState >500){
            mElevatorState = MotionState::STOPPED;
            mTimeInMotionState = millis();
            TurnOnPump(false);
            SetValve(0);
            SetPWM1(0);
            return;
        }else{
            return;
        }
    }
    
    if(mElevatorState == MotionState::DOWN_STOP){
        if(millis()-mTimeInMotionState >500){
            mElevatorState = MotionState::STOPPED;
            mTimeInMotionState = millis();
            TurnOnPump(false);
            SetValve(100);
            SetPWM1(0);
            return;
        }else{
            return;
        }
    }

    if(mElevatorState == MotionState::UP_PRIME){
        if(millis()-mTimeInMotionState>500){
            mElevatorState = MotionState::UP;
            mTimeInMotionState = millis();
            TurnOnPump(true);
            SetValve(0);
            SetPWM1(0);
            return;
        }else{
            return;
        }
    }

    if(mElevatorState == MotionState::UP_INIT){
        mElevatorState = MotionState::UP;
        mTimeInMotionState = millis();
        TurnOnPump(true);
        SetValve(0);
        SetPWM1(0);
        return;
    }

    if(mElevatorState == MotionState::UP){
        if(error<tolerance){
            mElevatorState = MotionState::UP_STOP;
            mTimeInMotionState = millis();
            TurnOnPump(true);
            SetValve(35);
            SetPWM1(0);
            return;
        }
        if(error<pRange){
            float speed = (error/pRange)*100.0;
            //speed = map((long)speed, 0,100, 35, 100);
            speed += 40;
            SetValve((uint16_t)speed);
            return;
        }else{
            SetValve(100);
            return;
        }
    }

    if(mElevatorState == MotionState::DOWN_PRIME){
        if(millis()-mTimeInMotionState > 500){
            mElevatorState = MotionState::DOWN_INIT;
            mTimeInMotionState = millis();
            TurnOnPump(true);
            SetValve(40);
            SetPWM1(0);
            return;
        }else{
            return;
        }
    }

    if(mElevatorState == MotionState::DOWN_INIT){
        if(millis()-mTimeInMotionState > 250){
            mElevatorState = MotionState::DOWN;
            mTimeInMotionState = millis();
            TurnOnPump(true);
            SetValve(40);
            SetPWM1(100);
            return;
        }else{
            return;
        }
    }

    if(mElevatorState == MotionState::DOWN){
        if(abs(error)<tolerance){
            mElevatorState = MotionState::DOWN_STOP;
            mTimeInMotionState = millis();
            TurnOnPump(false);
            SetValve(100);
            SetPWM1(100);
            return;
        }
        if(abs(error)<pRange){
            float speed = ((error)/pRange)*35.0;
            /*
            if(mPumpOn){
                speed = map(speed, 0, 100, 50, 0);
            }else{
                speed = map(speed, 0, 100, 100, 0);
            }
            */
            //speed = map((long)speed, 0, 100, 40, 0);
            speed = 35-speed;
            SetPWM1(100);
            SetValve((uint16_t)speed);
            return;
        }else{
            SetPWM1(100);
            SetValve(0);
            return;
        }
    }

   

}

void HydraulicController::SetElevatorSpeed(int8_t percent, uint16_t pumpPressure, uint16_t cylinderPressure){
    //to go up, solenoid is off and valve 0->100% is speed
    //to go down, solenoid is on and valve 100->0% is speed

    if(percent == 0){
        //stop
        SetPWM1(00);
        SetValve(00);
        TurnOnPump(false);
        mElevatorState = MotionState::STOPPED;
        //float dP = cylinderPressure - pumpPressure;//find the difference between the two pressures
        
    }else{
        if(percent < 0 && GetPumpOnTimeMillis()<1000){
            //Prime pump
            TurnOnPump(true);
        }else{
            //go down

            percent = abs(percent);//get rid of negative
            
            //make sure not to back drive pump
            if(mPumpOn ){
                percent = map(percent, 0, 100, 50, 0);
            }else{
                percent = map(percent, 0, 100, 100, 0);
            }

            SetPWM1(100);
            
            //percent = 100 - percent;//invert speed
            SetValve(percent);
        }

        if(percent > 0 && GetPumpOnTimeMillis()<1000){
            //prime pump
            TurnOnPump(true);
            
        }else{
            SetPWM1(0);
            SetValve(abs(percent));
        }
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

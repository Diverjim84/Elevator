#pragma once


//Globals are defined in seperate Header
#include "Globals.h"
//Put all pin definitions in this file
#include "PinDefinitions.h"

namespace PinChange{

volatile uint8_t PC0_PrevState;
volatile uint8_t PC1_PrevState;
volatile uint8_t PC2_PrevState;

void initPinChangeInterrupts(){

  //Functions on PCINT1
  pinMode(PumpMotor_OverPresure, INPUT_PULLUP);
  pinMode(PumpMotor_VoltagePresent, INPUT_PULLUP);

  //Functions on PCINT2
  pinMode(CallButton_UpperBTN, INPUT_PULLUP);
  pinMode(CallButton_LowerBTN, INPUT_PULLUP);
  
  pinMode(DoorLock_Lower_LimitSwitch, INPUT_PULLUP);
  pinMode(DoorLock_Upper_LimitSwitch, INPUT_PULLUP);

  pinMode(LimitSwitch_LowerMax, INPUT_PULLUP);
  pinMode(LimitSwitch_UpperMax, INPUT_PULLUP);

  
  PCMSK2 = PCINT_Mask2;  // Enable interrupts for pins A8-15
  PCMSK1 = PCINT_Mask1;   // Enable interrupts for pins 14-15
  PCICR  = PCINT_En_Mask;  // Enable PCINT2 interrupt

  PC2_PrevState = PINK;
  PC1_PrevState = PINJ;

}

ISR(PCINT1_vect){
  //find all changed pins with XOR operation
  byte pinStates = PINJ & PCINT_Mask1; //Only check pins that are enabled for pin interrupts
  byte changed_pins = PC1_PrevState ^ pinStates; 
  PC1_PrevState = pinStates;  // Update pin states

  //Pump Motor Over Pressure Changed  
  if(changed_pins & 0x02){
    //rising edge = door open
    if(pinStates  & (0x02)) {
      Globals::hc->SetPumpPresureExceeded(true);

      #ifdef PRINT_DEBUG_INTERRUPT
        Serial.println("*** ISR: Pump Motor Pressure Exceeded ***");
      #endif
    }else{ //falling edge = door closed    
      Globals::hc->SetPumpPresureExceeded(false);
      #ifdef PRINT_DEBUG_INTERRUPT
        Serial.println("*** ISR: Pump Motor Pressure Cleared ***");
      #endif
    }
  }

  //Pump Power Changed
  if(changed_pins & 0x01){
    //rising edge = power on
    if(pinStates  & (0x01)) {
      Globals::hc->SetPumpPowerFailure(false);
      #ifdef PRINT_DEBUG_INTERRUPT
        Serial.println("*** ISR: Pump Motor Has Power ***");
      #endif
    }else{ //falling edge = no power   
      Globals::hc->SetPumpPowerFailure(true);
      #ifdef PRINT_DEBUG_INTERRUPT
        Serial.println("*** ISR: Pump Motor Power Lost ***");
      #endif
    }
  }

}

ISR(PCINT2_vect){
  //find all changed pins with XOR operation
  byte pinStates = PINK & PCINT_Mask2; //Only check pins that are enabled for pin interrupts
  byte changed_pins = PC2_PrevState ^ pinStates; 
  PC2_PrevState = pinStates;  // Update pin states

  // Check for falling edges on each pin
  //Call Button Upper Rising edge
  if ((changed_pins & (1 << (CallButton_UpperBTN - (uint8_t)62U))))
  {
    if(pinStates & (1 << (CallButton_UpperBTN - (uint8_t)62U))) {
      // Call Button Pressed
      //digitalWrite(CallButton_UpperButtonLED, !digitalRead(CallButton_UpperButtonLED));
      //digitalWrite(CallButton_UpperButtonLED, LOW);
      Globals::upperCallBtn.set(false);
      
      //globals::hc.TurnOnPump(LOW);
      #ifdef PRINT_DEBUG_INTERRUPT
        Serial.println("\n\n*** ISR: Upper Call Btn Rising edge ***\n");
      #endif
    }else
    {
      // Call Button Not Pressed
      Globals::upperCallBtn.set(true);
      Globals::hc->SetTargetPosition(1500);
      
      //digitalWrite(CallButton_UpperButtonLED, HIGH);
      //globals::hc.TurnOnPump(HIGH);
      #ifdef PRINT_DEBUG_INTERRUPT
        Serial.println("\n\n*** ISR: Upper Call Btn Falling edge ***\n");
      #endif
    }
    
  }

  //Call Button Lower Rising edge
  if (changed_pins & (1 << (CallButton_LowerBTN - (uint8_t)62U))) 
  {
    if(pinStates & (1 << (CallButton_LowerBTN - (uint8_t)62U))) {
      // Call Button Pressed 
      //digitalWrite(CallButton_LowerButtonLED, LOW);
      Globals::lowerCallBtn.set(false);
      
      //globals::hc.SetPWM1(00);
      #ifdef PRINT_DEBUG_INTERRUPT
        Serial.println("\n\n*** ISR: Lower Call Btn Rising edge ***\n");
      #endif
    }else
    {
      // Call Button Not Pressed
      Globals::lowerCallBtn.set(true);
      Globals::hc->SetTargetPosition(15);
      
      //digitalWrite(CallButton_LowerButtonLED, HIGH);
      //globals::hc.SetPWM1(100);
      #ifdef PRINT_DEBUG_INTERRUPT
        Serial.println("\n\n*** ISR: Lower Call Btn Falling edge ***\n");
      #endif
    }
    
  }

  // Lower Door Close Switch changed
  if (changed_pins & (1 << (DoorLock_Lower_LimitSwitch - (uint8_t)62U))){
    //rising edge = door open
    if(pinStates & (1 << (DoorLock_Lower_LimitSwitch - (uint8_t)62U))) {
      Globals::hc->SetLowerDoorOpen(true);
      #ifdef PRINT_DEBUG_INTERRUPT
        Serial.println("*** ISR: Lower Door Switch Open ***");
      #endif
    }else //falling edge = door closed
    {
      Globals::hc->SetLowerDoorOpen(false);
      #ifdef PRINT_DEBUG_INTERRUPT
        Serial.println("*** ISR: Lower Door Switch Closed ***");
      #endif
    }
  }

  //Upper Door Close Switch changed
  if(changed_pins & (1 << (DoorLock_Upper_LimitSwitch - (uint8_t)62U))){
    //rising edge = door open
    if(pinStates  & (1 << (DoorLock_Upper_LimitSwitch - (uint8_t)62U))) {
      Globals::hc->SetUpperDoorOpen(true);

      #ifdef PRINT_DEBUG_INTERRUPT
        Serial.println("*** ISR: Upper Door Switch Open ***");
      #endif
    }else{ //falling edge = door closed    
      Globals::hc->SetUpperDoorOpen(false);

      #ifdef PRINT_DEBUG_INTERRUPT
        Serial.println("*** ISR: Upper Door Switch Closed ***");
      #endif
    }
  }

  //Upper Travel Limit Switch changed
  if(changed_pins & (1 << (LimitSwitch_UpperMax - (uint8_t)62U))){
    //rising edge = Limit Reached
    if(pinStates   &  (1  <<  (LimitSwitch_UpperMax - (uint8_t)62U))) {
      Globals::hc->SetUpperLimitSwitch(true);
      //Globals::hc->Stop();
      #ifdef PRINT_DEBUG_INTERRUPT
        Serial.println("*** ISR: Upper Limit Triggered ***");
      #endif
    }else{   //falling edge = not triggered
      Globals::hc->SetUpperLimitSwitch(false);

      #ifdef PRINT_DEBUG_INTERRUPT
        Serial.println("*** ISR: Upper Limit Cleared ***");
      #endif
    }
  }

  //Lower Travel Limit Switch changed
  if(changed_pins & (1 << (LimitSwitch_LowerMax - (uint8_t)62U))){
    //rising edge  = Limit Reached
    if(pinStates    &   (1   <<   (LimitSwitch_LowerMax - (uint8_t)62U))) {
      Globals::hc->SetLowerLimitSwitch(true);
      //Globals::hc->Stop();
      #ifdef PRINT_DEBUG_INTERRUPT
        Serial.println("*** ISR: Lower Limit Triggered ***");
      #endif
    }else{ //falling edge  = not triggered
      Globals::hc->SetLowerLimitSwitch(false);
      #ifdef PRINT_DEBUG_INTERRUPT
        Serial.println("*** ISR: Lower Limit Cleared ***");
      #endif
    }
  }
  // ... similar checks for other pins on Port K
}


}

#pragma once


//Globals are defined in seperate Header
#include "Globals.h"
//Put all pin definitions in this file
#include "PinDefinitions.h"

namespace PinChange{

volatile uint8_t PC0_PrevState;
volatile uint8_t PC1_PrevState;
volatile uint8_t PC2_PrevState;

void initCallButtons(){

  pinMode(CallButton_UpperButtonBTN, INPUT_PULLUP);
  pinMode(CallButton_UpperButtonLED, OUTPUT);

  pinMode(CallButton_LowerButtonBTN, INPUT_PULLUP);
  pinMode(CallButton_LowerButtonLED, OUTPUT);

  pinMode(CallButton_ClosetUpperBTN, INPUT_PULLUP);
  pinMode(CallButton_ClosetUpperLED, OUTPUT);

  pinMode(CallButton_ClosetLowerBTN, INPUT_PULLUP);
  pinMode(CallButton_ClosetLowerLED, OUTPUT);


  uint8_t mask = 0x00;
  mask |= 1<<(CallButton_ClosetUpperBTN-(uint8_t)62U);

  PCMSK2 = mask;  // Enable interrupts for pins 10-13, 50-53
  PCICR |= (1 << PCIE2);  // Enable PCINT2 interrupt

  PC2_PrevState = PINK;

}

ISR(PCINT2_vect){
  byte changed_pins = PC2_PrevState ^ PINK;  // XOR with current pin states to detect changes
  PC2_PrevState = PINK;  // Update pin states

  // Check for falling edges on each pin
  if ((changed_pins & (1 << (CallButton_UpperButtonBTN - (uint8_t)62U)))&&
      (PINK & (1 << (CallButton_UpperButtonBTN - (uint8_t)62U)))) {
    digitalWrite(CallButton_UpperButtonLED, !digitalRead(CallButton_UpperButtonLED));
  }
  if ((changed_pins & (1 << (CallButton_LowerButtonBTN - (uint8_t)62U)))&&
      (PINK & (1 << (CallButton_LowerButtonBTN - (uint8_t)62U)))) {
    digitalWrite(CallButton_LowerButtonLED, !digitalRead(CallButton_LowerButtonLED));
  }
  if ((changed_pins & (1 << (CallButton_ClosetUpperBTN - (uint8_t)62U)))&&
      (PINK & (1 << (CallButton_ClosetUpperBTN - (uint8_t)62U)))) {
    digitalWrite(CallButton_ClosetUpperLED, !digitalRead(CallButton_ClosetUpperLED));
  }
  if ((changed_pins & (1 << (CallButton_ClosetLowerBTN - (uint8_t)62U)))&&
      (PINK & (1 << (CallButton_ClosetLowerBTN - (uint8_t)62U)))) {
    digitalWrite(CallButton_ClosetLowerLED, !digitalRead(CallButton_ClosetLowerLED));
  }
  // ... similar checks for other pins on Port K
}


}

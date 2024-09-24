#include <Arduino.h>

//Globals are defined in seperate Header
#include "Globals.h"

//Moved defines that control debugging to this header file
#include "Debug.h"

//Put all pin definitions in this file
#include "PinDefinitions.h"

#include "EStops.h"
#include "PinChangeManager.h"
#include "LEDControl.h"




/*
//This is the class for the Hydraulics Control code.
#include "HydraulicController.h"

HydraulicController hc;
*/








void setup() {
  //This only enables Serial if debugging
  #ifdef DEBUG_ON
    Serial.begin(115200);
  #endif


  //Use pin 13 to know board is on and working.
  //pinMode(LED_BUILTIN, OUTPUT);
  


  pinMode(A7, INPUT);
  pinMode(A6, INPUT);
  pinMode(A5, INPUT);
  pinMode(A4, INPUT);
  pinMode(A3, INPUT);
  pinMode(A2, INPUT);


  //pinMode(PumpMotor_Relay, OUTPUT);

  //pinMode(Solenoid_Relay, OUTPUT);
  pinMode(DoorLock_Lower_Relay, OUTPUT);
  pinMode(DoorLock_Upper_Relay, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(29, OUTPUT);
  pinMode(Vent_Relay, OUTPUT);
  

  EStop::initESTOP();
  PinChange::initPinChangeInterrupts();
  LEDControl::init();
  LEDControl::LEDs_Off();
  globals::hc.Init();
  globals::hc.SetValve(50);

}

float analogToVoltage(int pin) {
  float voltage = (float)analogRead(pin) * (5.0/1023.0);
  return voltage;

}
uint16_t analogToPSI(int pin) {
  //.5V-4.5V = 1000psi; 4V/1000psi = .004V/psi; 1023ticks/5v = .00489V/tick
  //x*(5/2033)*(1000/4) = x*1.22=psi
  float psi = (float)(analogRead(pin)-90) * (5.0/1023.0) * (1000.0/4.0);
  return (uint16_t)psi;
}
void loop() {

  //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));//Toggle Built in LED
  delay(250);//2hz blink

  //digitalWrite(CallButton_UpperButtonLED, !digitalRead(CallButton_UpperButtonLED));//Toggle Built in LED


  //LEDControl::update();
  EStop::estopLEDUpdate();

  float x = ((float)analogRead(A7)/ 1023.0)*100.0;
  int valvePercent = (int)x;
  globals::hc.SetValve(valvePercent);
  ///*
  Serial.print("\n\nAnalog 7: "); Serial.print(valvePercent);
  Serial.print("; Valve PWM: "); Serial.print(globals::hc.GetValvePercent());
  Serial.print("; M1 PWM: "); Serial.print(globals::hc.GetPWM1Percent());
  //Serial.print("; Temp 3: "); Serial.print(analogRead(A6));
  //Serial.print("; Temp 2: "); Serial.print(analogRead(A5));
  Serial.print("; Pump Pressure: "); Serial.print(analogToPSI(A4)); Serial.print("psi("); Serial.print(analogRead(A4));Serial.print(" ticks)");
  Serial.print("; Cylinder Presure: "); Serial.print(analogToPSI(A3)); Serial.print("psi("); Serial.print(analogRead(A3));Serial.print(" ticks)");
  //Serial.print("; Pump Temp: "); Serial.println(analogRead(A2));
  //*/

  /*
  if(!EStop::eStopped()){
    globals::hc.SetValve((int)(2.0*x*10.0));
    if(digitalRead(CallButton_ClosetUpperLED)){
      globals::hc.SetPWM1(100);
    }else{
      globals::hc.SetPWM1(0);
    }
    //globals::hc.EnableValve(digitalRead(CallButton_UpperButtonLED));
  }
  */

/*
  if(hc.BuildInTest()){
    #ifdef PRINT_DEBUG_MOTOR_FAULTS
      Serial.print("\nValve Fault Detected\n");
    #endif //PRINT_DEBUG_MOTOR_FAULTS
  }

  #ifdef PRINT_DEBUG_MOTOR_CURRENT
    hc.GetVarValveCurrent_mA();//use built in debug print statements
  #endif //PRINT_DEBUG_MOTOR_CURRENT
 */

}

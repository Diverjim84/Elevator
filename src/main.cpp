#include <Arduino.h>

//Globals are defined in seperate Header
#include "Globals.h"

//Moved defines that control debugging to this header file
#include "Debug.h"

//Put all pin definitions in this file
#include "PinDefinitions.h"

#include "EStops.h"
#include "PinChangeManager.h"




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
  pinMode(LED_BUILTIN, OUTPUT);
  


  pinMode(A7, INPUT);

  EStop::initESTOP();
  PinChange::initCallButtons();

  globals::hc.Init();
  globals::hc.SetValve(0);

}

void loop() {

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));//Toggle Built in LED
  delay(250);//2hz blink

  float x = ((float)analogRead(A7)) * (5.0 / 1023.0);
  if(x<.25){
    x = 0.0;
  }
  Serial.print("\nAnalog 7: ");
  Serial.println((int)(2.0*x*10.0));

  if(!EStop::eStopped()){
    globals::hc.SetValve((int)(2.0*x*10.0));
    if(digitalRead(CallButton_ClosetUpperLED)){
      globals::hc.SetPWM1(100);
    }else{
      globals::hc.SetPWM1(0);
    }
    //globals::hc.EnableValve(digitalRead(CallButton_UpperButtonLED));
  }

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

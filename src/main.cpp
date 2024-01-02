#include <Arduino.h>

//Moved defines that control debugging to this header file
#include "Debug.h"

//Put all pin definitions in this file
#include "PinDefinitions.h"

//This is the class for the Hydraulics Control code.
#include "HydraulicController.h"

HydraulicController hc;



void setup() {
  //This only enables Serial if debugging
  #ifdef DEBUG_ON
    Serial.begin(115200);
  #endif


  //Use pin 13 to know board is on and working.
  pinMode(LED_BUILTIN, OUTPUT);

}

void loop() {

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));//Toggle Built in LED
  delay(250);//2hz blink

  if(hc.GetValveHasFault()){
    #ifdef PRINT_DEBUG_MOTOR_FAULTS
      Serial.print("\nValve Fault Detected\n");
    #endif //PRINT_DEBUG_MOTOR_FAULTS
  }


  #ifdef PRINT_DEBUG_MOTOR_CURRENT
    hc.GetVarValveCurrent_mA();//use built in debug print statements
  #endif //PRINT_DEBUG_MOTOR_CURRENT
  
}

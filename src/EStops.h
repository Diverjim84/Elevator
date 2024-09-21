#pragma once

//Globals are defined in seperate Header
#include "Globals.h"
//Put all pin definitions in this file
#include "PinDefinitions.h"


namespace EStop {

    volatile bool upperState;
    volatile bool lowerState;
    volatile bool carState;
    volatile bool closetState;

    //code to estop system
    void ESTOP(){
        globals::hc.Stop();
    }


        //check estop status
    bool eStopped(){
        bool estopState;

        noInterrupts(); //No interupts while checking this
        
        if(     upperState  == LOW 
            //&&  lowerState  == LOW 
            //&&  carState    == LOW 
            //&&  closetState == LOW 
          ){
            estopState = false;
        }else{
            estopState = true;
        }
        interrupts(); //allow interupts

        return estopState;
    }

    void estopLEDUpdate(){
        if(eStopped()){
            //if Estop button is triggered, then it should be solid, others blink
            if(upperState){//on
                digitalWrite(ESTOP_UpperLED, HIGH);
            }else{//blink
                digitalWrite(ESTOP_UpperLED, !digitalRead(ESTOP_UpperLED));
            }
            if(lowerState){//on
                digitalWrite(ESTOP_LowerLED, HIGH);
            }else{//blink
                digitalWrite(ESTOP_LowerLED, !digitalRead(ESTOP_LowerLED));
            }
            if(carState){//on
                digitalWrite(ESTOP_CarLED, HIGH);
            }else{//blink
                digitalWrite(ESTOP_CarLED, !digitalRead(ESTOP_CarLED));
            }
            if(closetState){//on
                digitalWrite(ESTOP_ClosetLED, HIGH);
            }else{//blink
                digitalWrite(ESTOP_ClosetLED, !digitalRead(ESTOP_ClosetLED));
            }
        }else{ //turn off all LEDs
            digitalWrite(ESTOP_UpperLED, LOW);
            digitalWrite(ESTOP_LowerLED, LOW);
            digitalWrite(ESTOP_CarLED, LOW);
            digitalWrite(ESTOP_ClosetLED, LOW);
        }
    }


    void isrESTOPUpper(){
        bool currentState = digitalRead(ESTOP_UpperBTN);

        if(upperState != currentState){
            upperState = currentState;

            if(upperState == HIGH){ //ESTOP SET
                digitalWrite(ESTOP_UpperLED, HIGH);//Turn on Estop Light
                ESTOP();
            }else{  //ESTOP CLEAR
                digitalWrite(ESTOP_UpperLED, LOW);
            }

        }

    }

    void isrESTOPLower(){

    }

    void isrESTOPCar(){

    }

    void isrESTOPCloset(){

    }

    void initESTOP(){
  
        //Upper Estop
        pinMode(ESTOP_UpperLED, OUTPUT);//LED
        pinMode(ESTOP_UpperBTN, INPUT_PULLUP); // Set the pin as input with internal pullup resistor
        attachInterrupt(digitalPinToInterrupt(ESTOP_UpperBTN), isrESTOPUpper, CHANGE); // Attach the interrupt with RISING mode

        //Lower Estop
        pinMode(ESTOP_LowerLED, OUTPUT);//LED
        pinMode(ESTOP_LowerBTN, INPUT_PULLUP); // Set the pin as input with internal pullup resistor
        attachInterrupt(digitalPinToInterrupt(ESTOP_LowerBTN), isrESTOPLower, CHANGE); // Attach the interrupt with RISING mode

        //Car Estop
        pinMode(ESTOP_CarLED, OUTPUT);//LED
        pinMode(ESTOP_CarBTN, INPUT_PULLUP); // Set the pin as input with internal pullup resistor
        attachInterrupt(digitalPinToInterrupt(ESTOP_CarBTN), isrESTOPCar, CHANGE); // Attach the interrupt with RISING mode

        //Closet Estop
        pinMode(ESTOP_ClosetLED, OUTPUT);//LED
        pinMode(ESTOP_ClosetBTN, INPUT_PULLUP); // Set the pin as input with internal pullup resistor
        attachInterrupt(digitalPinToInterrupt(ESTOP_ClosetBTN), isrESTOPCloset, CHANGE); // Attach the interrupt with RISING mode

        upperState  = digitalRead(ESTOP_UpperBTN);
        lowerState  = digitalRead(ESTOP_LowerBTN);
        carState    = digitalRead(ESTOP_CarBTN);
        closetState = digitalRead(ESTOP_ClosetBTN);

        estopLEDUpdate();        
    }



    
    


}



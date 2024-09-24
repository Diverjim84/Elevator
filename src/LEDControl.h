#pragma once



#include "Globals.h"
#include "PinDefinitions.h"

namespace LEDControl{
    void LEDs_On(){
        digitalWrite(CallButton_UpperButtonLED, 1);
        digitalWrite(CallButton_LowerButtonLED, 1);
        digitalWrite(Upper_RGB_R, 1);
        digitalWrite(Upper_RGB_G, 1);
        digitalWrite(Upper_RGB_B, 1);
        digitalWrite(Lower_RGB_R, 1);
        digitalWrite(Lower_RGB_G, 1);
        digitalWrite(Lower_RGB_B, 1);
    }

    void LEDs_Off(){
        digitalWrite(CallButton_UpperButtonLED, 0);
        digitalWrite(CallButton_LowerButtonLED, 0);
        digitalWrite(Upper_RGB_R, 0);
        digitalWrite(Upper_RGB_G, 0);
        digitalWrite(Upper_RGB_B, 0);
        digitalWrite(Lower_RGB_R, 0);
        digitalWrite(Lower_RGB_G, 0);
        digitalWrite(Lower_RGB_B, 0);
    }

    void init(){
        
        //Call Buttons
        pinMode(CallButton_UpperButtonLED, OUTPUT);
        pinMode(CallButton_LowerButtonLED, OUTPUT);

        pinMode(Upper_RGB_R, OUTPUT);
        pinMode(Upper_RGB_G, OUTPUT);
        pinMode(Upper_RGB_B, OUTPUT);

        pinMode(Lower_RGB_R, OUTPUT);
        pinMode(Lower_RGB_G, OUTPUT);
        pinMode(Lower_RGB_B, OUTPUT);

        LEDs_Off();


    }

    void update(){

        //Toggle All LEDs
        digitalWrite(CallButton_UpperButtonLED, digitalRead(CallButton_UpperButtonLED) ^ 1);
        digitalWrite(CallButton_LowerButtonLED, digitalRead(CallButton_LowerButtonLED) ^ 1);
        
        digitalWrite(Upper_RGB_R, digitalRead(Upper_RGB_R) ^ 1);
        digitalWrite(Upper_RGB_G, digitalRead(Upper_RGB_G) ^ 1);
        digitalWrite(Upper_RGB_B, digitalRead(Upper_RGB_B) ^ 1);
        
        digitalWrite(Lower_RGB_R, digitalRead(Lower_RGB_R) ^ 1);
        digitalWrite(Lower_RGB_G, digitalRead(Lower_RGB_G) ^ 1);
        digitalWrite(Lower_RGB_B, digitalRead(Lower_RGB_B) ^ 1);


    }

    
}
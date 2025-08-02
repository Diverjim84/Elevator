#pragma once



#include "Globals.h"
#include "PinDefinitions.h"

namespace LEDControl{
    
    void setUpperRGBLED(uint8_t red, uint8_t green, uint8_t blue){
        OCR3A = (uint16_t)red << 2;
        OCR3B = (uint16_t)green << 2;
        OCR3C = (uint16_t)blue << 2;
    
    }

    void setLowerRGBLED(uint8_t red, uint8_t green, uint8_t blue){
        OCR1A = (uint16_t)red << 2;
        OCR1B = (uint16_t)green << 2;
        OCR1C = (uint16_t)blue  << 2;
    }

    void LEDs_On(){
        digitalWrite(CallButton_UpperButtonLED, 1);
        digitalWrite(CallButton_LowerButtonLED, 1);
        setUpperRGBLED(255, 255, 255);
        setLowerRGBLED(255, 255, 255);
    }

    void LEDs_Off(){
        digitalWrite(CallButton_UpperButtonLED, 0);
        digitalWrite(CallButton_LowerButtonLED, 0);
        setUpperRGBLED(0, 0, 0);
        setLowerRGBLED(0, 0, 0);
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


        //configure RGB LEDs
        TCCR3A = 0;
        TCCR3B = 0;
        // Set non-inverting mode for each channel
        TCCR3A |= (1 << COM3A1) | (1 << COM3B1) | (1 << COM3C1);

        // Set Fast PWM mode, 10-bit resolution
        TCCR3A |= (1 << WGM31) | (1 << WGM30);
        TCCR3B |= (1 << WGM32);

        // Set prescaler to 256
        // 16MHz / 256 / 1024 ≈ 61.03Hz PWM frequency
        TCCR3B |= (1 << CS32);

        //configure RGB LEDs
        TCCR1A = 0;
        TCCR1B = 0;
        // Set non-inverting mode for each channel
        TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << COM1C1);

        // Set Fast PWM mode, 10-bit resolution
        TCCR1A |= (1 << WGM11) | (1 << WGM10);
        TCCR1B |= (1 << WGM12);

        // Set prescaler to 256
        // 16MHz / 256 / 1024 ≈ 61.03Hz PWM frequency
        TCCR1B |= (1 << CS12);

        LEDs_Off();


    }

    

    void update(){

        

    }

    
}
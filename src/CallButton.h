#pragma once

#include <Arduino.h>
//#include <mutex>
//#include <atomic>

class CallButton {
private:
    volatile bool _state;
    uint8_t _LEDPin;

public:
    CallButton(uint8_t LEDPin, bool state=false){
        _LEDPin = LEDPin;
        _state = state;
        pinMode(_LEDPin, OUTPUT);
        digitalWrite(_LEDPin, state);
    };

    ~CallButton(){};

    void set(bool state) {
        noInterrupts(); // Disable interrupts to create a critical section
        _state = state;
        interrupts();  // Re-enable interrupts
        digitalWrite(_LEDPin, state);
        
    };
    bool get() { 
        bool currentState = false;
        noInterrupts(); // Disable interrupts to create a critical section
        currentState = _state;
        interrupts();  // Re-enable interrupts
        return currentState;
    };
};

    
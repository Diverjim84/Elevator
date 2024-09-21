#pragma once

#include <Arduino.h>
//make sure header is only linked once.  Will draw on it from multiple files


class HydraulicController
{
public:
    HydraulicController();
    ~HydraulicController();

    enum HCFaults{
        VALVE_PWM_FAULT = 0,
        VALVE_PWM_OVER_CURRENT = 1,
        PUMP_OVER_PRESURE = 2,
        PUMP_OVER_TEMP = 3,
        PUMP_FLUID_LOW = 4,
        PUMP_NO_POWER = 5,
        PS24V_NO_POWER = 6
    };


    void Init();

    int GetVarValveCurrent_mA();

    void SetValve(uint16_t percent);
    void SetPWM1(uint16_t percent);
    void EnableValve(bool enable);

    void Stop();

    void TurnOnPump(bool on);

    bool BuildInTest();

    bool IsFaultPresent(HCFaults type);

    unsigned long GetPumpOnTimeMillis();

    void PWMFaultISR(bool state);
    void PumpOverPresureISR(bool state);
    void PumpPowerISR(bool state);
    void PumpFluidISR(bool state);

private:
    /* data */
    volatile uint8_t mFaults;//holds the current fault status
    

    bool mPWMEnabled;

    uint8_t mValvePercent;
    uint8_t mPWM1Percent;

    bool mPumpOn;
    unsigned long mPumpStartTime;

    
    void ConfigPWMTimer();
    

    inline void SetFaultValue(HCFaults type, bool value);
    

};




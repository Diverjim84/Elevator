#pragma once

//make sure header is only linked once.  Will draw on it from multiple files


class HydraulicController
{
private:
    /* data */
    volatile bool mPWMFault;
    volatile bool mPWMFault_Last;

    volatile bool mPumpOverPressure;
    volatile bool mPumpOverPressure_Last;

    volatile bool mPumpPower;
    volatile bool mPumpPower_Last;

    volatile bool mPumpFluid;
    volatile bool mPumpFluid_Last;

    bool mPWMEnabled;

    int mValvePercent;

    bool mPumpOn;
    unsigned long mPumpStartTime;

    
    void ConfigPWMTimer();
    void EnableValve(bool enable);
    
public:
    HydraulicController();
    ~HydraulicController();

    void Init();

    int GetVarValveCurrent_mA();

    void SetValve(int percent);

    void Stop();

    void TurnOnPump(bool on);

    bool BuildInTest();

    bool GetValveHasFault();

    unsigned long GetPumpOnTimeMillis();

    void PWMFaultISR(bool state);
    void PumpOverPresureISR(bool state);
    void PumpPowerISR(bool state);
    void PumpFluidISR(bool state);


};




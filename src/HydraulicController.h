#pragma once

#include <Arduino.h>
#include "EStops.h"
//make sure header is only linked once.  Will draw on it from multiple files

class EStops;

class HydraulicController
{
public:
    HydraulicController(EStops* es);
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

    enum MotionState{
        UP          =  4,
        UP_INIT     =  3,
        UP_PRIME    =  2,
        UP_STOP     =  1,
        STOPPED     =  0,
        DOWN_STOP   = -1,
        DOWN_PRIME  = -2,
        DOWN_INIT   = -3,
        DOWN        = -4
        
    };

    enum PositionState{
        PS_STOPPED = 0,
        PS_UPPER_FLOOR = 1,
        PS_LOWER_FLOOR = 2
    };

    void Init(EStops* es);

    int GetVarValveCurrent_mA();

    void UpdateValve();//ramp control
    void SetValve(uint16_t percent);
    void SetPWM1(uint16_t percent);

    void SetElevatorSpeed(int8_t speed, uint16_t pumpPressure, uint16_t cylinderPressure);
    void DriveElevator(int8_t speed, uint16_t pumpPressure, uint16_t cylinderPressure);
    void DrivePosition();

    MotionState GetNewState(int8_t speed);
    void PrintMotionState();
    
    
    void SetTargetPosition(float pos){ mPositionTarget = pos;};
    float GetTargetPosition(){return mPositionTarget;};
    void SetPositionControl(bool enable);
    bool GetPositionControl(){return mPositionControlEnabled;}
    float GetPositionError(){return mPositionError;};
    unsigned long GetTimeSinceLastPos(){return mPositionLastTime;};
    

    uint16_t GetValvePercent(){return mValvePercent;};
    uint16_t GetPWM1Percent(){return mPWM1Percent;};

    void EnableValve(bool enable);

    void Stop();

    void TurnOnPump(bool on);
    bool GetPumpState(){return mPumpOn;};
    unsigned long GetPumpOnTimeMillis();
    

    bool BuildInTest();

    bool IsFaultPresent(HCFaults type);

    

    void PWMFaultISR(bool state);
    void PumpOverPresureISR(bool state);
    void PumpPowerISR(bool state);
    void PumpFluidISR(bool state);

    void SetPumpPresureExceeded(bool exceeded){pumpMotor_PressureExceeded = exceeded;};
    bool GetPumpPresureExceeded(){return pumpMotor_PressureExceeded;};

    void SetPumpPowerFailure(bool failed){pumpMotor_PowerFailure = failed;};
    bool GetPumpPowerFailure(){return pumpMotor_PowerFailure;};

    void SetLowerLimitSwitch(bool state){_lowerTravelLS = state;};
    bool GetLowerLimitSwitch(){return _lowerTravelLS;};

    void SetUpperLimitSwitch(bool state){_upperTravelLS = state;};
    bool GetUpperLimitSwitch(){return _upperTravelLS;};

    void SetUpperDoorOpen(bool open){_upperDoorOpen = open;};
    bool GetUpperDoorOpen(){return _upperDoorOpen;};

    void SetLowerDoorOpen(bool open){_lowerDoorOpen = open;};
    bool GetLowerDoorOpen(){return _lowerDoorOpen;};


private:
    /* data */
    volatile uint8_t mFaults;//holds the current fault status
    
    EStops* _es;
    volatile bool _lowerTravelLS;
    volatile bool _upperTravelLS;

    volatile bool _upperDoorOpen;
    volatile bool _lowerDoorOpen;


    volatile bool pumpMotor_PressureExceeded;
    volatile bool pumpMotor_PowerFailure;

    bool mPWMEnabled;

    uint8_t mValvePercent;
    uint8_t mValveTarget;
    const uint8_t mValveRampRate = 5;

    float mPositionTarget;
    bool mPositionControlEnabled;
    float mPositionError;
    unsigned long mPositionLastTime;
    
    uint8_t mPWM1Percent;

    bool mPumpOn;
    unsigned long mPumpStartTime;
    MotionState mElevatorState;
    unsigned long mTimeInMotionState;
    
    void ConfigPWMTimer();
    

    inline void SetFaultValue(HCFaults type, bool value);
    

};




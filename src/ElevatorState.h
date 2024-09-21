#pragma once

/*
This file will hold the different states of the elevator
*/

enum ElevatorStates{
    ESTOPPED, 
    ESTOP_OVERRIDE,
    IN_TRANSIT,
    ARRIVED,
    WAITING_OPEN,
    IDLE,
    SERVICE
    

};

enum ElevatorPositions{
    UNKOWN,
    LOWER_LEVEL,
    MIDSHAFT,
    UPPER_LEVEL,
    OUT_OF_BOUNDS
};

enum ElevatorFault{
    VALVE_PWM_FAULT,
    VALVE_PWM_OVER_CURRENT,
    PUMP_OVER_PRESURE,
    PUMP_OVER_TEMP,
    PUMP_FLUID_LOW,
    PUMP_NO_POWER,
    PS24V_NO_POWER,
    LIMIT_LOWER,
    LIMIT_UPPER,
    DOOR_UPPER,
    DOOR_LOWER,
    TRAVEL_FAILURE_UNDER_PRESURE,
    CAR_OVERWEIGHT
};
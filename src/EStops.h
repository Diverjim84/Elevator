#pragma once

//Globals are defined in seperate Header
//Put all pin definitions in this file
#include "PinDefinitions.h"
#include "HydraulicController.h"

class HydraulicController;

class EStops
{
private:
    static EStops* instance;  // Static instance pointer
    
    HydraulicController* _hc;

    volatile bool upperState;
    volatile bool lowerState;
    volatile bool carState;
    volatile bool closetState;

    static void isrESTOPUpper();
    static void isrESTOPLower();
    static void isrESTOPCar();
    static void isrESTOPCloset();

public:
    EStops(HydraulicController* hydraulicController);
    void init(HydraulicController* hydraulicController);
    void update();
    
    bool getUpperState();
    bool getLowerState();
    bool getCarState();
    bool getClosetState();

    bool eStopped();
    void ESTOP();

    void printEstopStatus();

    

    ~EStops();
};






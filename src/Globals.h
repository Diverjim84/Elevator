#pragma once

// This is the class for the Hydraulics Control code.
#include "PinDefinitions.h"
#include "HydraulicController.h"
#include "EStops.h"
#include "CallButton.h"

namespace Globals
{
    extern CallButton upperCallBtn;
    extern CallButton lowerCallBtn;
    
    extern EStops* es; // Pointer to EStops
    extern HydraulicController* hc; // Pointer to HydraulicController

    extern uint8_t          EncoderStatus;
    extern float            EncoderAngle;
    extern float            EncoderPosition;
    extern float            EncoderSpeed;
    extern unsigned long    EncoderLastMsgTime;

    void initializeGlobals(); // Function to initialize global variables
}

#pragma once

//This is the class for the Hydraulics Control code.
#include "HydraulicController.h"

namespace globals {

HydraulicController hc;

volatile bool UpperDoorOpen = false;
volatile bool LowerDoorOpen = false;

volatile bool UpperTravelLimitReached = false;
volatile bool LowerTravelLimitReached = false;

volatile bool PumpMotor_PressureExceeded = false;
volatile bool PumpMotor_PowerFailure = false;

volatile bool UpperCalled = false;
volatile bool LowerCalled = false;

}
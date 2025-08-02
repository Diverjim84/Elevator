#include "Globals.h"
#include "HydraulicController.h"
#include "EStops.h"

// Initialize global instances
namespace Globals{

    CallButton upperCallBtn(CallButton_UpperButtonLED, false);
    CallButton lowerCallBtn(CallButton_LowerButtonLED, false);

    EStops* es = nullptr;              // Pointer to EStops
    HydraulicController* hc = nullptr; // Pointer to HydraulicController

    uint8_t EncoderStatus;
    float   EncoderAngle;
    float   EncoderPosition;
    float   EncoderSpeed; 
    unsigned long    EncoderLastMsgTime;

    void initializeGlobals() {
        hc = new HydraulicController(es); // Create instance of HydraulicController
        es = new EStops(hc);               // Create instance of EStops
        hc->Init(es); //update pointer to Estops in HC

        EncoderStatus   = 3;
        EncoderAngle    = -1;
        EncoderPosition = -1;
        EncoderSpeed    = 0; 
        EncoderLastMsgTime = 0;
    }
}
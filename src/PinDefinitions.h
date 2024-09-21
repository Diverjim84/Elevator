#pragma once
//make sure header is only linked once.  Will draw on it from multiple files

//Helper Macros
#ifndef ULONG_MAX
  #define ULONG_MAX 0xffffffffUL
#endif
//Pin map


//These are the Pins for the Pololu Motor Controller Board
//We will remap these to functions

//PWM H-Bridge 
/*
  EN = Enable pin
  DIR = Direction Pin
  PWM = PWM pin - Speed 
  DIAG = Diagnostics/fault - low = fault 
  OCM = Output Current Monitor = Analog sensor to detect current
*/

//All EN and DIR pins need to be moved to less 
//valuable pins in final design

#define M1EN 2
#define M1DIR 9 
//Pin 7 = PH4 - OC4B - Timer4
#define M1PWM 7 
#define M1DIAG 6
#define M1OCM A0
#define M1OCR OCR4B

#define M2EN 4
#define M2DIR 10
//Pin 8 = PH5 - OC4C - Timer4
#define M2PWM 8 
#define M2DIAG 12
#define M2OCM A1
#define M2OCR OCR4C


/***************************** Hydrolic Control ********************************/
//AC Pump
#define PumpMotor_Relay 31
#define PumpMotor_VoltagePresent 15
#define PumpMotor_Temp A2
#define PumpMotor_FluidLow 14
#define PumpMotor_OverPresure 3
#define PumpMotor_Presure A3

//Valve Control
#define PowerSuppy_Relay 30
#define PowerSuppy_VoltagePresent A4


#define Solenoid_Relay 32

#define Valve_Enable M2EN
#define Valve_Dir M2DIR
#define Valve_PWM M2PWM
#define Valve_Fault M2DIAG
#define Valve_CurrentMonitor M2OCM
#define Valve_CompareReg M2OCR

#define PWM1_CompareReg M1OCR




//Limit Switches
#define LimitSwitch_UpperMax 13
#define LimitSwitch_LowerMax 11


/***************************** Button Management ********************************/

#define CallButton_UpperButtonBTN A8
#define CallButton_UpperButtonLED 26

#define CallButton_LowerButtonBTN A9
#define CallButton_LowerButtonLED 27

#define CallButton_ClosetUpperBTN A10
#define CallButton_ClosetUpperLED 29
#define CallButton_ClosetLowerBTN A11
#define CallButton_ClosetLowerLED 28

#define ESTOP_UpperBTN 18
#define ESTOP_UpperLED 22

#define ESTOP_LowerBTN 20
#define ESTOP_LowerLED 24

#define ESTOP_CarBTN 19
#define ESTOP_CarLED 23

#define ESTOP_ClosetBTN 21
#define ESTOP_ClosetLED 25

/***************************** Door Control ********************************/

#define DoorLock_Lower_Relay 34
#define DoorLock_Upper_Relay 33

#define DoorLock_Lower_LimitSwitch A15
#define DoorLock_Upper_LimitSwitch A14
 

/***************************** Vent Fan ***********************************/


#define Vent_Relay 35

/***************************** Communication ******************************/

#define ESP32_SPI_CS 49
#define ESP32_SPI_MISO 50
#define ESP32_SPI_MOSI 51
#define ESP32_SPI_CLK 52



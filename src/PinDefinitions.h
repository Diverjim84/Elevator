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

#define Valve_Enable M2EN
#define Valve_Dir M2DIR
#define Valve_PWM M2PWM
#define Valve_Fault M2DIAG
#define Valve_CurrentMonitor M2OCM
#define Valve_CompareReg M2OCR

/***************************** Solenoid Control  *************************/
//solenoid valve on Motor
#define Solenoid_Enable M1EN
#define Solenoid_Dir M1DIR
#define Solenoid_PWM M1PWM
#define Solenoid_Fault M1DIAG
#define Solenoid_CurrentMonitor M1OCM
#define Solenoid_CompareReg M1OCR



/***************************** Analog ********************************/

#define PumpMotor_Temp A2
#define Cylinder_Pressure A3
#define PumpMotor_Presure A4
#define Temp_2 A5
#define Temp_3 A6
#define Joystick A7


/***************************** Relays ********************************/

//220V Pump Motor Relay
#define PumpMotor_Relay 31

//relay header
#define Solenoid_Relay 32
#define DoorLock_Upper_Relay 33
#define DoorLock_Lower_Relay 34
#define Vent_Relay 35


/***************************** Interrupts ********************************/

//ESTOP Buttons
#define ESTOP_CarBTN 18
#define ESTOP_UpperBTN 20
#define ESTOP_LowerBTN 19
#define ESTOP_ClosetBTN 21

//External Interrupt control registers
//  Int Id: 5   4    3    2    1    0
//Pin List: D3, D2, D18, D19, D20, D21
#define EStop_EICRA 0xFF
#define EStop_EIMSK 0x0F
/*
EICR = External Interrupt Control Register
EICRA = int3-int0
EICRB = int7-int4
ISCn1 ISCn0  Description
0     0      The low level of INTn generates an interrupt request
0     1      Any logical change on INTn generates an interrupt request
1     0      The falling edge between two samples of INTn generates an interrupt request
1     1      The rising edge between two samples of INTn generates an interrupt request
*/



//PCINT0 (Port B - PINB)
//Int Id:    7    6    5    4    3    2    1    0
//Pin List: D13, D12, D11, D10, D50, D51, D52, D53

//PCINT1 (Port J - PINJ)
// Int ID   15  14  13  12  11  10   9    8
//pin list: NC, NC, NC, NC, NC, D15, D14, D0
#define PumpMotor_OverPresure 14
#define PumpMotor_VoltagePresent 15

//PCINT2 (port K - PINK)
//pin list: A8-A15

//Call Buttons
#define CallButton_UpperBTN A8
#define CallButton_LowerBTN A9
//Door closed switches
#define DoorLock_Lower_LimitSwitch A13
//#define DoorLock_Upper_LimitSwitch A12
#define DoorLock_Upper_LimitSwitch A14
//Limit Switches
//#define LimitSwitch_UpperMax A14
#define LimitSwitch_UpperMax A12
#define LimitSwitch_LowerMax A15

//Pin Change control register masks PCMSK[2-0]
//PCINT 7-0
#define PCINT_Mask0 0x00
//PCINT 15-8
#define PCINT_Mask1 0b00000110
//PCINT 23-16
#define PCINT_Mask2 0b11110011
//Enable PCINT2, PCINT1, PCINT0
#define PCINT_En_Mask 0b00000110

/***************************** LED Management ********************************/

//LEDs
#define CallButton_UpperButtonLED 27
#define CallButton_LowerButtonLED 26
#define CallButton_ClosetUpperLED 29
#define CallButton_ClosetLowerLED 28

#define ESTOP_UpperLED 24
#define ESTOP_LowerLED 22
#define ESTOP_CarLED 23
#define ESTOP_ClosetLED 25

//RGB
//OC3{A,B,C}
#define Upper_RGB_R 11 
#define Upper_RGB_G 12 
#define Upper_RGB_B 13 

//OC1{A,B,C}
#define Lower_RGB_R 5
#define Lower_RGB_G 2
#define Lower_RGB_B 3

/***************************** Communication ******************************/

#define ESP32_SPI_CS 49
#define ESP32_SPI_MISO 50
#define ESP32_SPI_MOSI 51
#define ESP32_SPI_CLK 52


/************************** Opperating Parameters *************************/

#define PUMP_PRIME_TIME 350 //ms



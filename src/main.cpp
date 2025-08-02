#include <Arduino.h>


//Moved defines that control debugging to this header file
#include "Debug.h"

//Put all pin definitions in this file
#include "PinDefinitions.h"

#include "EStops.h"
#include "PinChangeManager.h"
#include "LEDControl.h"

//Globals are defined in seperate Header
#include "Globals.h"

#include <CAN.h>
#include <SPI.h>





unsigned long previousFastMillis = 0;
unsigned long previousSlowMillis = 0;
unsigned long previousVeryFastMillis = 0;

// CAN TX Variables
unsigned long prevTX = 0;                                        // Variable to store last execution time
const unsigned int invlTX = 1000;                                // One second interval constant
byte data[] = {0xAA, 0x55, 0x01, 0x10, 0xFF, 0x12, 0x34, 0x56};  // Generic CAN data to send

// CAN RX Variables
long unsigned int rxId;
unsigned char len;
unsigned char rxBuf[8];

// Serial Output String Buffer
char msgString[128];


#define CAN_INT 53                              // Set INT to pin 2\                               // Set CS to pin 10

void canInit(){
  
  //init Can bus at 500kb/s
  Serial.println("\nInitializing CAN...");
  
  CAN.setPins(49, 53);

  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while(1);
  }

  Serial.println("CAN Initialized...");

}




void setup() {
  //This only enables Serial if debugging
  #ifdef DEBUG_ON
    Serial.begin(115200);
  #endif
  while (!Serial);

  Serial.print("\nInitializing....");

  previousFastMillis = 0;
  previousSlowMillis = 0;

  canInit();

  //Use pin 13 to know board is on and working.
  //pinMode(LED_BUILTIN, OUTPUT);

  pinMode(A7, INPUT);
  pinMode(A6, INPUT);
  pinMode(A5, INPUT);
  pinMode(A4, INPUT);
  pinMode(A3, INPUT);
  pinMode(A2, INPUT);

  pinMode(36, INPUT_PULLUP);

  
  //pinMode(Solenoid_Relay, OUTPUT);
  pinMode(DoorLock_Lower_Relay, OUTPUT);
  pinMode(DoorLock_Upper_Relay, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(29, OUTPUT);
  pinMode(Vent_Relay, OUTPUT);
  

  PinChange::initPinChangeInterrupts();
  LEDControl::init();
  LEDControl::LEDs_Off();
  Globals::initializeGlobals();
  
}

float analogToVoltage(int pin) {
  float voltage = (float)analogRead(pin) * (5.0/1023.0);
  return voltage;

}

uint16_t analogToPSI(int pin) {
  //.5V-4.5V = 1000psi; 4V/1000psi = .004V/psi; 1023ticks/5v = .00489V/tick
  //x*(5/2033)*(1000/4) = x*1.22=psi
  float psi = (float)(analogRead(pin)-90) * (5.0/1023.0) * (1000.0/4.0);
  return (uint16_t)psi;
}

float analogToTemp(int pin) {
  // Constants
  const float SERIES_RESISTOR = 22000;  // 22k pull-up resistor
  const float THERMISTOR_NOMINAL = 100000;  // 100k thermistor
  const float TEMPERATURE_NOMINAL = 25;  // 25°C
  const float B_COEFFICIENT = 4267;  // B value of the thermistor

  float rawADC = analogToVoltage(pin);
  
  //Serial.print("\nRaw ADC value: ");
  //Serial.println(rawADC);
  
  // Calculate resistance of thermistor
  //float resistance = SERIES_RESISTOR * ((5.0 / rawADC )- 1.0);
  float resistance = rawADC / ((5.0 - rawADC)/ SERIES_RESISTOR);


  //Serial.print("Thermistor resistance: ");
  //Serial.println(resistance);
  
  // Calculate temperature using the B parameter equation
  float steinhart = resistance / THERMISTOR_NOMINAL;  // (R/Ro)
  //Serial.print("R/Ro: ");
  //Serial.println(steinhart, 6);
  
  steinhart = log(steinhart);  // ln(R/Ro)
  //Serial.print("ln(R/Ro): ");
  //Serial.println(steinhart, 6);
  
  steinhart /= B_COEFFICIENT;  // 1/B * ln(R/Ro)
  //Serial.print("1/B * ln(R/Ro): ");
  //Serial.println(steinhart, 6);
  
  steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15);  // + (1/To)
  //Serial.print("1/B * ln(R/Ro) + (1/To): ");
  //Serial.println(steinhart, 6);
  
  float temperature = 1.0 / steinhart;  // Invert
  temperature -= 273.15;  // Convert absolute temp to Celsius
  
  temperature = (temperature * 9.0) / 5 + 32;   // Convert to Fahren

  //Serial.print("Calculated temperature: ");
  //Serial.println(temperature, 2);  // Print with 2 decimal places
  
  return temperature;
  //float temperatureF = steinhart * 9.0 / 5.0 + 32.0;

  //return temperatureF;

} 

void can_decode(long id, int len){
  uint8_t data[8];
  float param = 0.0;

  switch (id)
  {
  case 0x10:
    if(len == 1 ){
      //Serial.print("\nEncoder Status: ");
      Globals::EncoderStatus = CAN.read();
      Globals::EncoderLastMsgTime = millis();
      //Serial.print(status);
    }else{
      Serial.print("\nEncoder Status: Msg Error");
    }
    break;
  case 0x11:
    if(len == 4 ){
      //Serial.print("\nEncoder Angle: ");
      for(int i =0; i<4; i++){
        data[i] = CAN.read();
      }
      memcpy(&param, data,sizeof(float));
      Globals::EncoderAngle = param;
      Globals::EncoderLastMsgTime = millis();
      //Serial.print(param);
    }else{
      Serial.print("\nEncoder Angle: Msg Error");
    }
    break;
  case 0x12:
    if(len == 4 ){
      //Serial.print("\nEncoder Position: ");
      for(int i =0; i<4; i++){
        data[i] = CAN.read();
      }
      memcpy(&param, data,sizeof(float));
      if(Globals::EncoderPosition<0){
        //if this is the first position message, set it to the target
        Globals::hc->SetTargetPosition(param);
      }
      Globals::EncoderPosition = param;
      Globals::EncoderLastMsgTime = millis();
      //Serial.print(param);
    }else{
      Serial.print("\nEncoder Position: Msg Error");
    }
    break;
  case 0x13:
    if(len == 4 ){
      //Serial.print("\nEncoder Speed: ");
      for(int i =0; i<4; i++){
        data[i] = CAN.read();
      }
      memcpy(&param, data,sizeof(float));
      Globals::EncoderSpeed = param; 
      Globals::EncoderLastMsgTime = millis();
      //Serial.print(param);
    }else{
      Serial.print("\nEncoder Speed: Msg Error");
    }
    break;
  case 0x14:
    if(len == 8){
      // Read all 8 bytes
      for(int i = 0; i<8; i++){
        data[i] = CAN.read();
      }

      // Decode status byte
      Globals::EncoderStatus = data[0];
      Globals::EncoderLastMsgTime = millis();
      
      // Decode raw_angle (scaled by 100)
      uint16_t d = (data[1] << 8) | data[2];
      Globals::EncoderAngle = (float)d / 100.0;
      
      // Decode position (scaled by 100)
      d = (data[3] << 8) | data[4];
      Globals::EncoderPosition = (float)d / 100.0;

      // Decode velocity (scaled by 100, signed int16_t)
      int16_t v = (int16_t)((data[5] << 8) | data[6]);
      Globals::EncoderSpeed = (float)v / 100.0;

      // Optional: Check marker or reserved byte
      if (data[7] != 0xaa) {
          // Handle unexpected value in byte 7 if needed
      }
    }
    break;
  default:
    break;
  }

}

void can_loop(){
  int packetSize = CAN.parsePacket();
  if(packetSize){
    if(!CAN.packetRtr()){
      //can_decode(CAN.packetId(), packetSize);
    }
  }
}

void veryFastLoop(){
  Globals::hc->UpdateValve();
  if(Globals::hc->GetPositionControl()){
    Globals::hc->DrivePosition();
  }

}

void fastLoop() {

  //center is at 490, top is 1023, bottom is 0
  //after centering, top is 533, bottom is -490
  float deadzone = 15.0; //deadzone of 15
  int stickX = analogRead(A7);
  float speed = ((float)stickX)-490.0; 
  
  bool positionControl = !digitalRead(36);
  Globals::hc->SetPositionControl(positionControl);


  if(speed < deadzone && speed > -deadzone ) {
    //stop
    speed = 0;
  }else{
    if (speed > deadzone)//if speed is positive
    {
      speed = (speed*100.0)/533.0;//scale by positive range
    }else{
      speed = (speed*100.0)/490.0;//scale by negative range
    }
  }
  

  //if(!EStop::eStopped()) {
  //  globals::hc.SetElevatorSpeed((int8_t)speed, analogToPSI(A4), analogToPSI(A3));
  //}

  //Drive Elevator Controls
  ///*
  if (positionControl) {
    Serial.println("I shouldn't be here...");
    //Globals::hc->DrivePosition();
  } else if(Globals::lowerCallBtn.get()){
    //Serial.println("Lowering");
    Globals::hc->DriveElevator(-90, 0, 0);
    //globals::hc.DriveElevator(-30.0, analogToPSI(PumpMotor_Presure), analogToPSI(Cylinder_Pressure) );
  }else if (Globals::upperCallBtn.get()){
    //Serial.println("Raising");
    Globals::hc->DriveElevator(80, 0, 0);
    //globals::hc.DriveElevator(30.0, analogToPSI(PumpMotor_Presure), analogToPSI(Cylinder_Pressure) );
  }else{
    //Serial.println("Stopping");
    //Globals::hc->DriveElevator(speed, 0, 0);

    if(abs(speed) > 0){
      
      if(speed>0){
        Globals::hc->SetValve(speed);
        Globals::hc->SetPWM1(0);
      }else{
        Globals::hc->SetValve(100-abs(speed));
        Globals::hc->SetPWM1(100);
      }
    }else{
      Globals::hc->DriveElevator(0, 0, 0);
    }
    //globals::hc.DriveElevator(speed, analogToPSI(PumpMotor_Presure), analogToPSI(Cylinder_Pressure) );
  }
  //*/
  

}

void slowLoop() {
  //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));//Toggle Built in LED
  //digitalWrite(CallButton_UpperButtonLED, !digitalRead(CallButton_UpperButtonLED));//Toggle Built in LED

  //LEDControl::update();
  Globals::es->update();

  

  //center is at 490, top is 1023, bottom is 0
  //after centering, top is 533, bottom is -490
  float deadzone = 15.0; //deadzone of 15
  float speed = (float)analogRead(A7)-490.0; 
  
  if(speed < deadzone && speed > -deadzone ) {
    //stop
    speed = 0;
  }else{
    if (speed > deadzone)//if speed is positive
    {
      speed = (speed*100.0)/533.0;//scale by positive range
    }else{
      speed = (speed*100.0)/490.0;//scale by negative range
    }
    
  }

  //if(!EStop::eStopped()) {
  //  globals::hc.SetElevatorSpeed((int8_t)speed, analogToPSI(A4), analogToPSI(A3));
  //}

  
  if(Globals::es->eStopped()) {
    Globals::es->printEstopStatus();
  }
  /*
  //Serial.print("\n\nAnalog 7: "); Serial.print((int8_t)speed);//valvePercent);
  Serial.print("\nValve PWM: "); Serial.print(Globals::hc->GetValvePercent());
  Serial.print("; M1 PWM: "); Serial.print(Globals::hc->GetPWM1Percent());
  Serial.print("; Pump Pressure: "); Serial.print(analogToPSI(A4)); Serial.print("psi("); Serial.print(analogRead(A4));Serial.print(" ticks)");
  Serial.print("; Cylinder Presure: "); Serial.print(analogToPSI(A3)); Serial.print("psi("); Serial.print(analogRead(A3));Serial.print(" ticks)");
  //Serial.print(";\n Pump Temp: "); Serial.print(analogToTemp(A2));Serial.print("F");
  //Serial.print("; Temp 2: "); Serial.print(analogToTemp(A5));Serial.print("F");
  //Serial.print("; Temp 3: "); Serial.print(analogToTemp(A6));Serial.print("F");
  //Serial.print(";\n Pump State: "); Serial.print(Globals::hc->GetPumpState());
  //Serial.print("; Motion State: "); Globals::hc->PrintMotionState();
  //Serial.print("; UpLS: "); Serial.print(digitalRead(LimitSwitch_UpperMax));//Serial.print(Globals::hc->GetUpperLimitSwitch());
  //Serial.print("; LwLS: "); Serial.print(Globals::hc->GetLowerLimitSwitch());
  //Serial.print("; Upper Call: "); Serial.print(Globals::upperCallBtn.get());
  //Serial.print("; Lower Call: "); Serial.print(Globals::lowerCallBtn.get());
  //Serial.print("; Pump On time: "); Serial.print(Globals::hc->GetPumpOnTimeMillis());Serial.print("ms\n");
  //*/
  ///*
  /*
  Serial.print("\nEncoder: Status: "); Serial.print(Globals::EncoderStatus);
  Serial.print("; Angle: ");  Serial.print(Globals::EncoderAngle);
  Serial.print("; Position: ");  Serial.print(Globals::EncoderPosition);
  Serial.print("; Speed: ");  Serial.print(Globals::EncoderSpeed);
  */

  Serial.print("\nTarget Pos: "); Serial.print(Globals::hc->GetTargetPosition());
  Serial.print("; Pos Ctrl En: ");  Serial.print(Globals::hc->GetPositionControl());
  Serial.print("; Motor: ");  Serial.print(Globals::hc->GetPumpState());
  Serial.print("; Valve: ");  Serial.print(Globals::hc->GetValvePercent());
  Serial.print("; Solenoid: ");  Serial.print(Globals::hc->GetPWM1Percent());
  Serial.print("; State: ");  Globals::hc->PrintMotionState();
  
  /*
  float e = Globals::hc->GetTargetPosition() - Globals::EncoderPosition;
  Serial.print("; Pos Error: ");  Serial.print(e);
  if(abs(e)<400){
    e = e/400;
    if(e<-15){
      e = map((long)abs(e), 0,100, 50, 0);
    }else if(e>15){
      e = map((long)abs(e), 0,100, 35, 100);
    }else{
      e = 35;
    }
  }else{
    e = 100;
  }
  Serial.print("; Output: ");  Serial.print(e);
  Serial.print("\n");
  */
  
  //*/

  




/*
  if(hc.BuildInTest()){
    #ifdef PRINT_DEBUG_MOTOR_FAULTS
      Serial.print("\nValve Fault Detected\n");
    #endif //PRINT_DEBUG_MOTOR_FAULTS
  }

  #ifdef PRINT_DEBUG_MOTOR_CURRENT
    hc.GetVarValveCurrent_mA();//use built in debug print statements
  #endif //PRINT_DEBUG_MOTOR_CURRENT
 */

}


void loop() {
  can_loop();
  unsigned long currentMillis = millis();
  if(currentMillis - previousVeryFastMillis >= 20) {
    previousVeryFastMillis = currentMillis;
    //veryFastLoop();
  }

  // Call the fast function every 5 ms
  if (currentMillis - previousFastMillis >= 100) {
      //Serial.println("Fast Loop.");
      previousFastMillis = currentMillis; // Update the last time function was called
      fastLoop(); // Call your fast function
      //can_loop(); // Call CAN loop
  }

  // Call the slow function every 250 ms
  if (currentMillis - previousSlowMillis >= 150) {
      previousSlowMillis = currentMillis; // Update the last time function was called
      slowLoop(); // Call your slow function
      //Serial.print("\n Slow Loop.\n");
      //Serial.print("millis(): "); Serial.print(currentMillis); Serial.print("; previousFastTime: "); Serial.println(previousFastMillis);
  }

    // Other code can go here

}

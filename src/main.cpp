#include <Arduino.h>


//Comment out to end all Debug
#define DEBUG_ON

#ifdef DEBUG_ON
  //comment out to stop printing different levels
  //Very important for keeping serial port output manageable

  //Print PWM debug Statements
  #define PRINT_DEBUG_PWM

  //Print settings of motors
  #define PRINT_DEBUG_MOTOR

  //Print motor Faults
  //#define PRINT_DEBUG_MOTOR_FAULTS

  //Print Motor Current draw
  //#define PRINT_DEBUG_MOTOR_CURRENT

#endif //DEBUG_ON


//MotorShieldPins()

//Pin map

/*
  EN = Enable pin
  DIR = Direction Pin
  PWM = PWM pin - Speed 
  DIAG = Diagnostics/fault - low = fault 
  OCM = Output Current Monitor = Analog sensor to detect current
*/

#define M1EN 2
//#define M1DIR 7
#define M1DIR 9 
//#define M1PWM 9 //PH6 - OC2B - Timer2
#define M1PWM 7 //PH4 - OC4B - Timer4
#define M1DIAG 6
#define M1OCM A0

#define M2EN 4
//#define M2DIR 8
#define M2DIR 10
//#define M2PWM 10 //PB4 - OC2A - Timer2
#define M2PWM 8 //PH5 - OC4C - Timer4
#define M2DIAG 12
#define M2OCM A1

void configMotor1Pins(){
  pinMode(M1EN, OUTPUT);
  pinMode(M1DIR, OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(M1DIAG, OUTPUT);
  pinMode(M1OCM, INPUT);

}

/*
This Function Configures the pins for Motor 2
*/
void configMotor2Pins(){
  pinMode(M2EN, OUTPUT);
  pinMode(M2DIR, OUTPUT);
  pinMode(M2PWM, OUTPUT);
  pinMode(M2DIAG, OUTPUT);
  pinMode(M2OCM, INPUT);
  
}

/*
  Current code assumes high is Enabled and low is Disabled
*/
void setMotor1Enable(bool enable){

  digitalWrite(M1EN, (uint8_t)enable);//Cast bool to int, set pin
  
  #ifdef PRINT_DEBUG_MOTOR
    Serial.print("\nMotor 1 Enable set to: ");
    
    if(digitalRead(M1EN)==1){
      Serial.print("Enabled (pin = ");
    }else{
      Serial.print("Disabled (pin = ");
    }

    Serial.print(digitalRead(M1EN), DEC);
    Serial.print(")\n");

  #endif //PRINT_DEBUG_MOTOR
  
}


/*
  Current code assumes high is Enabled and low is Disabled
*/
void setMotor2Enable(bool enable){

  digitalWrite(M2EN, (uint8_t)enable);//Cast bool to int, set pin
  
  #ifdef PRINT_DEBUG_MOTOR
    Serial.print("\nMotor 2 Enable set to: ");
    
    if(digitalRead(M2EN)==1){
      Serial.print("Enabled (pin = ");
    }else{
      Serial.print("Disabled (pin = ");
    }

    Serial.print(digitalRead(M2EN), DEC);
    Serial.print(")\n");

  #endif //PRINT_DEBUG_MOTOR
  
}

/*
  Current code assumes high is forward and low is reverse
*/
void setMotor1Dir(bool forward){

  digitalWrite(M1DIR, (uint8_t)forward);//Cast bool to int, set pin
  
  #ifdef PRINT_DEBUG_MOTOR
    Serial.print("\nMotor 1 Direction Set to: ");
    
    if(forward){
      Serial.print("Forward (pin = ");
    }else{
      Serial.print("Reverse (pin = ");
    }

    Serial.print(digitalRead(M1DIR), DEC);
    Serial.print(")\n");

  #endif //PRINT_DEBUG_MOTOR
  
}

/*
  Current code assumes high is forward and low is reverse
*/
void setMotor2Dir(bool forward){

  digitalWrite(M2DIR, (uint8_t)forward);//Cast bool to int, set pin
  
  #ifdef PRINT_DEBUG_MOTOR
    Serial.print("\nMotor 2 Direction Set to: ");
    
    if(forward){
      Serial.print("Forward (pin = ");
    }else{
      Serial.print("Reverse (pin = ");
    }

    Serial.print(digitalRead(M2DIR), DEC);
    Serial.print(")\n");

  #endif //PRINT_DEBUG_MOTOR
  
}

//False = no fault
bool getMotor1HasFault(){
  #ifdef PRINT_DEBUG_MOTOR_FAULTS
    Serial.print("\nMotor 1 Fault Pin = ");
    Serial.print(digitalRead(M1DIAG), DEC);
    Serial.print("\n");
  #endif //PRINT_DEBUG_MOTOR_FAULTS
  
  return (bool)digitalRead(M1DIAG);
}

//False = no fault
bool getMotor2HasFault(){
  
  #ifdef PRINT_DEBUG_MOTOR_FAULTS
    Serial.print("\nMotor 2 Fault Pin = ");
    Serial.print(digitalRead(M2DIAG), DEC);
    Serial.print("\n");
  #endif //PRINT_DEBUG_MOTOR_FAULTS
  
  return (bool)digitalRead(M2DIAG);

}

//5V / 1024 ADC counts / 500 mV per A = 10 mA per count
int getMotor1Current_mA(){
  #ifdef PRINT_DEBUG_MOTOR_CURRENT
    Serial.print("\nMotor 1 Current Consumtion = ");
    Serial.print(analogRead(M1OCM) * 10, DEC);
    Serial.print("mA (raw = ");
    Serial.print(analogRead(M1OCM), DEC);
    Serial.print(")\n");
  #endif //PRINT_DEBUG_MOTOR_CURRENT
  
  // 5V / 1024 ADC counts / 500 mV per A = 10 mA per count
  return analogRead(M1OCM) * 10;
}

//5V / 1024 ADC counts / 500 mV per A = 10 mA per count
int getMotor2Current_mA(){
  #ifdef PRINT_DEBUG_MOTOR_CURRENT
    Serial.print("\nMotor 2 Current Consumtion = ");
    Serial.print(analogRead(M2OCM) * 10, DEC);
    Serial.print("mA (raw = ");
    Serial.print(analogRead(M2OCM), DEC);
    Serial.print(")\n");
  #endif //PRINT_DEBUG_MOTOR_CURRENT
  
  // 5V / 1024 ADC counts / 500 mV per A = 10 mA per count
  return analogRead(M2OCM) * 10;
}

/*
  ***THIS ASSUMES BOARD HAS BEEN CHANGED****
  This is assuming PWMs for Motor 1 and 2 are pins 7&8 

  This function will configure Timer 4 (16 bit timer)  For a 200Hz period
  This will use the PWM Fast mode that will use a clock prescaler of 8, which 
  will yield a max freq of 3921.16 Hz with the effective frequency defined by 
  ICR4 for the maximum resolution of the PWM period and duty cycle possible 
  with a 16bit counter/timer.
*/
void configTimer4(){

  //Clear default values from Config Registers
  TCCR4A = 0x00;
  TCCR4B = 0x00;

  //Configure Pin 8 to use Timer 4, setup fast PWM 
  //Prescaler = 8 = 0b010 = Max freq is 3921.16 Hz
  //Fast PWM(Top = ICR4) = WGM4 = 0b1110
  //Clear on compare of Pin 7 (OCR4B) and set on bottom = COM4B = 0b10
  //Clear on compare of Pin 8 (OCR4C) and set on bottom = COM4C = 0b10
  
  //TCCR4A = COM4A1 COM4A0 COM4B1 COM4B0 COM4C1 COM4C0 WGM41 WGM40
  TCCR4A |= (1<<COM4B1) | (1<<COM4C1) | (1<<WGM41); 
  
  //TCCR4B = ICNC4 ICES4 – WGM43 WGM42 CS42 CS41 CS40
  TCCR4B |= (1<<WGM43) | (1<<WGM42) | (1<<CS41); //Set timer4, fast PWM, prescaler to 8
  
  ICR4 = 9997; //(16,000,000Hz/200Hz/8)-1 = 9,999

  OCR4B = 0; //Set pin 7 Duty Cycle to 0% - Off
  OCR4C = 0; //Set pin 8 Duty Cycle to 0% - Off

  //if PRINT_DEBUG_PWM is defined print out registers set
  #ifdef PRINT_DEBUG_PWM
    Serial.print("\n\nTimer 4 Configured:");

    Serial.print("\nTCCR4A: ");
    Serial.print(TCCR4A, BIN);
    Serial.println("\nTCCR4A = [COM4A1, COM4A0, COM4B1, COM4B0, COM4C1, COM4C0, WGM41, WGM40]");

    Serial.print("\nTCCR4B: ");
    Serial.print(TCCR4B, BIN);
    Serial.println("\nTCCR4B = [ICNC4, ICES4, - , WGM43, WGM42, CS42, CS41, CS40]");

    Serial.print("\nTCCR4C: ");
    Serial.print(TCCR4C, BIN);
    Serial.println("\nTCCR4C = [FOC4A, FOC4B, FOC4C, - , - , - , - , -]");

    Serial.print("\nICR4: ");
    Serial.print(ICR4, DEC);
    Serial.println("\nICR4 = top of timer = period");

    Serial.print("\nOCR4B: ");
    Serial.print(OCR4B, DEC);
    Serial.println("\nOCR4B = Counter Value when pin should turn off");

    Serial.print("\nOCR4C: ");
    Serial.print(OCR4C, DEC);
    Serial.println("\nOCR4C = Counter Value when pin should turn off");

  #endif //PRINT_DEBUG_PWM
  
}

/*
  Accepts an unsigned int16 from 0-100 as the percent ON Duty Cycle
  This works for Pin 7 PWM control using ICR4 as the top fo counter.
*/
void setMotor1DutyCycle(uint16_t percent){
  //Check limits
  if(percent>100){
    percent = 100;
  }

  //Set compare register to the value to turn off
  OCR4B = ICR4*percent/100;

  //Print all details of changing PWM on Pin 7 if debugging
  #ifdef PRINT_DEBUG_PWM
    Serial.print("\nPin 7 Duty Cycle Changed to:");
    Serial.print(percent, DEC);

    Serial.print("\nICR4: ");
    Serial.print(ICR4, BIN);
    Serial.println("\nICR4 = top of timer = period");

    Serial.print("\nOCR4B: ");
    Serial.print(OCR4B, BIN);
    Serial.println("\nOCR4B = Counter Value when pin should turn off");

  #endif //PRINT_DEBUG_PWM
  
}

/*
  Accepts an unsigned int16 from 0-100 as the percent ON Duty Cycle
  This works for Pin 8 PWM control using ICR4 as the top fo counter.
*/
void setMotor2DutyCycle(uint16_t percent){
  //Check limits
  if(percent>100){
    percent = 100;
  }

  //Set compare register to the value to turn off
  OCR4C = ICR4*percent/100;
    //Print all details of changing PWM on Pin 7 if debugging
  #ifdef PRINT_DEBUG_PWM
    Serial.print("\nPin 8 Duty Cycle Changed to:");
    Serial.print(percent, DEC);

    Serial.print("\nICR4: ");
    Serial.print(ICR4, BIN);
    Serial.println("\nICR4 = top of timer = period");

    Serial.print("\nOCR4C: ");
    Serial.print(OCR4C, BIN);
    Serial.println("\nOCR4C = Counter Value when pin should turn off");
  #endif //PRINT_DEBUG_PWM
  
}

void setup() {
  //This only enables Serial if debugging
  #ifdef DEBUG_ON
    Serial.begin(115200);
  #endif

  //Configure Timer for Motor PWMs
  configTimer4();

  //Configure Motor 1
  configMotor1Pins();
  setMotor1Dir(true);
  setMotor1Enable(true);

  //Configure Motor 2
  configMotor2Pins();
  setMotor2Dir(true);
  setMotor2Enable(true);
  

  setMotor1DutyCycle(75);//Set motor 1 to 75%
  setMotor2DutyCycle(25);//Set motor 2 to 25%

  //Use pin 13 to know board is on and working.
  pinMode(LED_BUILTIN, OUTPUT);

}

void loop() {

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));//Toggle Built in LED
  delay(250);//2hz blink

  if(getMotor1HasFault()){
    #ifdef PRINT_DEBUG_MOTOR_FAULTS
      Serial.print("\nMotor 1 Fault Detected\n");
    #endif //PRINT_DEBUG_MOTOR_FAULTS
  }

  if(getMotor2HasFault()){
    #ifdef PRINT_DEBUG_MOTOR_FAULTS
      Serial.print("\nMotor 2 Fault Detected\n");
    #endif //PRINT_DEBUG_MOTOR_FAULTS
  }

  #ifdef PRINT_DEBUG_MOTOR_CURRENT
    getMotor1Current_mA();//use built in debug print statements
    getMotor2Current_mA();//use built in debug print statements
  #endif //PRINT_DEBUG_MOTOR_CURRENT
  
}

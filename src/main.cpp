#include <Arduino.h>

//comment out to stop printing PWM debug Statements
#define PRINT_DEBUG_PWM

//Comment out to end all Debug
#define DEBUG_ON

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
  
  ICR4 = 9999; //(16,000,000Hz/200Hz/8)-1 = 9,999

  OCR4B = 0; //Set pin 7 Duty Cycle to 0% - Off
  OCR4C = 0; //Set pin 8 Duty Cycle to 0% - Off

  //if PRINT_DEBUG_PWM is defined print out registers set
  #ifdef DEBUG_ON
  #ifdef PRINT_DEBUG_PWM
    Serial.print("\n\nTimer 4 Configured:");

    Serial.print("\nTCCR4A: ");
    Serial.print(TCCR4A, BIN);
    Serial.println("TCCR4A = [COM4A1, COM4A0, COM4B1, COM4B0, COM4C1, COM4C0, WGM41, WGM40]");

    Serial.print("\nTCCR4B: ");
    Serial.print(TCCR4B, BIN);
    Serial.println("TCCR4B = [ICNC4, ICES4, - , WGM43, WGM42, CS42, CS41, CS40]");

    Serial.print("\nTCCR4C: ");
    Serial.print(TCCR4C, BIN);
    Serial.println("TCCR4B = [FOC4A, FOC4B, FOC4C, - , - , - , - , -]");

    Serial.print("\nICR4: ");
    Serial.print(ICR4, BIN);
    Serial.println("ICR4 = top of timer = period");

    Serial.print("\nOCR4B: ");
    Serial.print(OCR4B, BIN);
    Serial.println("OCR4B = Counter Value when pin should turn off");

    Serial.print("\nOCR4C: ");
    Serial.print(OCR4C, BIN);
    Serial.println("OCR4C = Counter Value when pin should turn off");

  #endif //PRINT_DEBUG_PWM
  #endif //DEBUG_ON
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
  #ifdef DEBUG_ON
  #ifdef PRINT_DEBUG_PWM
    Serial.print("\n\nPin 7 Duty Cycle Changed to:");
    Serial.print(percent, DEC);

    Serial.print("\nICR4: ");
    Serial.print(ICR4, BIN);
    Serial.println("ICR4 = top of timer = period");

    Serial.print("\nOCR4B: ");
    Serial.print(OCR4B, BIN);
    Serial.println("OCR4B = Counter Value when pin should turn off");

  #endif //PRINT_DEBUG_PWM
  #endif //DEBUG_ON
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
  #ifdef DEBUG_ON
  #ifdef PRINT_DEBUG_PWM
    Serial.print("\n\nPin 7 Duty Cycle Changed to:");
    Serial.print(percent, DEC);

    Serial.print("\nICR4: ");
    Serial.print(ICR4, BIN);
    Serial.println("ICR4 = top of timer = period");

    Serial.print("\nOCR4C: ");
    Serial.print(OCR4C, BIN);
    Serial.println("OCR4C = Counter Value when pin should turn off");
  #endif //PRINT_DEBUG_PWM
  #endif //DEBUG_ON
}

void setup() {
  //This only enables Serial if debugging
  #ifdef DEBUG_ON
    Serial.begin(115200);
  #endif

  configMotor1Pins();
  configMotor2Pins();
  configTimer4();

  setMotor1DutyCycle(75);//Set motor 1 to 75%
  setMotor2DutyCycle(25);//Set motor 2 to 25%

  //Use pin 13 to know board is on and working.
  pinMode(LED_BUILTIN, OUTPUT);

}

void loop() {

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));//Toggle Built in LED
  delay(250);//2hz blink

  
}

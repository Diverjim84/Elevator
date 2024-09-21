#pragma once
//make sure header is only linked once.  Will draw on it from multiple files


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
  #define PRINT_DEBUG_MOTOR_FAULTS

  //Print Motor Current draw
  //#define PRINT_DEBUG_MOTOR_CURRENT

#endif //DEBUG_ON
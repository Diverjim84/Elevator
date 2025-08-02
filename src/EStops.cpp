#include "EStops.h"
#include "Debug.h"


EStops* EStops::instance = nullptr;  // Initialize static instance pointer

EStops::EStops(HydraulicController* hydraulicController)
{
    instance = this;
    init(hydraulicController);
}

EStops::~EStops()
{
}
 
//code to estop system
void EStops::ESTOP(){
    _hc->Stop();

}


//check estop status
bool EStops::eStopped(){
    bool estopState;

    noInterrupts(); //No interupts while checking this

    if( 
        #ifndef ESTOP_UPPER_DISABLED
            upperState   ||
        #endif
        #ifndef ESTOP_LOWER_DISABLED
            lowerState   ||
        #endif
        #ifndef ESTOP_CAR_DISABLED
            carState     ||
        #endif
        #ifndef ESTOP_CLOSEST_DISABLED
            closetState 
        #endif
    ){
        estopState = true;
    }else{
        estopState = false;
    }
    
    interrupts(); //allow interupts

    return estopState;
}

void EStops::printEstopStatus(){
    Serial.print("\nUpper: "); Serial.print(upperState);Serial.print("(");Serial.print(digitalRead(ESTOP_UpperBTN));
    Serial.print("); Lower: "); Serial.print(lowerState);Serial.print("(");Serial.print(digitalRead(ESTOP_UpperBTN));
    Serial.print("); Car: "); Serial.print(carState);Serial.print("(");Serial.print(digitalRead(ESTOP_UpperBTN));
    Serial.print("); Closet: "); Serial.print(closetState);Serial.print("(");Serial.print(digitalRead(ESTOP_UpperBTN));
    Serial.print(")\n");
}

void EStops::update(){
    //digitalWrite(ESTOP_UpperLED, digitalRead(ESTOP_UpperLED)^1);
    //digitalWrite(ESTOP_LowerLED, digitalRead(ESTOP_LowerLED)^1);
    //digitalWrite(ESTOP_CarLED, digitalRead(ESTOP_CarLED)^1);
    //digitalWrite(ESTOP_ClosetLED, digitalRead(ESTOP_ClosetLED)^1);
    
    if(eStopped()){
        //if Estop button is triggered, then it should be solid, others blink
        if(upperState){//on
            digitalWrite(ESTOP_UpperLED, HIGH);
        }else{//blink
            digitalWrite(ESTOP_UpperLED, !digitalRead(ESTOP_UpperLED));
        }
        if(lowerState){//on
            digitalWrite(ESTOP_LowerLED, HIGH);
        }else{//blink
            digitalWrite(ESTOP_LowerLED, !digitalRead(ESTOP_LowerLED));
        }
        if(carState){//on
            digitalWrite(ESTOP_CarLED, HIGH);
        }else{//blink
            digitalWrite(ESTOP_CarLED, !digitalRead(ESTOP_CarLED));
        }
        if(closetState){//on
            digitalWrite(ESTOP_ClosetLED, HIGH);
        }else{//blink
            digitalWrite(ESTOP_ClosetLED, !digitalRead(ESTOP_ClosetLED));
        }
    }else{ //turn off all LEDs
        digitalWrite(ESTOP_UpperLED, LOW);
        digitalWrite(ESTOP_LowerLED, LOW);
        digitalWrite(ESTOP_CarLED, LOW);
        digitalWrite(ESTOP_ClosetLED, LOW);
    }
    //*/
}


void EStops::isrESTOPUpper(){
    instance->upperState = digitalRead(ESTOP_UpperBTN);

    if(instance->upperState == HIGH){ //ESTOP SET
        digitalWrite(ESTOP_UpperLED, HIGH);//Turn on Estop Light
        instance->ESTOP();
        #ifdef PRINT_DEBUG_INTERRUPT
            Serial.print("\n\n*** ISR: Upper Estop Button State: Set\n");
        #endif
    }else{  //ESTOP CLEAR
        digitalWrite(ESTOP_UpperLED, LOW);
        #ifdef PRINT_DEBUG_INTERRUPT
            Serial.print("\n\n*** ISR: Upper Estop Button State: Clear\n");
        #endif
    }

}

void EStops::isrESTOPLower(){
    instance->lowerState = digitalRead(ESTOP_LowerBTN);

    if(instance->lowerState == HIGH){ //ESTOP SET
        digitalWrite(ESTOP_LowerLED, HIGH);
        instance->ESTOP();
        #ifdef PRINT_DEBUG_INTERRUPT
            Serial.print("\n\n*** ISR: Lower Estop Button State: Set\n");
        #endif
    }else{   //ESTOP CLEAR
        digitalWrite(ESTOP_LowerLED, LOW);
        #ifdef PRINT_DEBUG_INTERRUPT
            Serial.print("\n\n*** ISR: Lower Estop Button State: Clear\n");
        #endif
    }
    
}

void EStops::isrESTOPCar(){
    instance->carState = digitalRead(ESTOP_CarBTN);
    
    if(instance->carState){ //ESTOP SET
        digitalWrite(ESTOP_CarLED, HIGH);
        instance->ESTOP();
        #ifdef PRINT_DEBUG_INTERRUPT
            Serial.print("\n\n*** ISR: Car Estop Button State: Set\n");
        #endif
    }else{    //ESTOP CLEAR
        digitalWrite(ESTOP_CarLED, LOW);
        #ifdef PRINT_DEBUG_INTERRUPT
            Serial.print("\n\n*** ISR: Car Estop Button State: Clear\n");
        #endif
    }

}

void EStops::isrESTOPCloset(){
    instance->closetState = digitalRead(ESTOP_ClosetBTN);

    if(instance->closetState == HIGH){ //ESTOP SET
        digitalWrite(ESTOP_ClosetLED, HIGH);
        instance->ESTOP();
            #ifdef PRINT_DEBUG_INTERRUPT
            Serial.print("\n\n*** ISR: Closet Estop Button State: Set\n");
            #endif
        }else{     //ESTOP CLEAR
        digitalWrite(ESTOP_ClosetLED, LOW);
            #ifdef PRINT_DEBUG_INTERRUPT
            Serial.print("\n\n*** ISR: Closet Estop Button State: Clear\n");
            #endif
    }
    
}

void EStops::init(HydraulicController* hydraulicController){
    _hc = hydraulicController;
    //Upper Estop
    pinMode(ESTOP_UpperLED, OUTPUT);//LED
    pinMode(ESTOP_UpperBTN, INPUT_PULLUP); // Set the pin as input with internal pullup resistor
    attachInterrupt(digitalPinToInterrupt(ESTOP_UpperBTN), isrESTOPUpper, CHANGE); // Attach the interrupt with RISING mode

    //Lower Estop
    pinMode(ESTOP_LowerLED, OUTPUT);//LED
    pinMode(ESTOP_LowerBTN, INPUT_PULLUP); // Set the pin as input with internal pullup resistor
    attachInterrupt(digitalPinToInterrupt(ESTOP_LowerBTN), isrESTOPLower, CHANGE); // Attach the interrupt with RISING mode

    //Car Estop
    pinMode(ESTOP_CarLED, OUTPUT);//LED
    pinMode(ESTOP_CarBTN, INPUT_PULLUP); // Set the pin as input with internal pullup resistor
    attachInterrupt(digitalPinToInterrupt(ESTOP_CarBTN), isrESTOPCar, CHANGE); // Attach the interrupt with RISING mode

    //Closet Estop
    pinMode(ESTOP_ClosetLED, OUTPUT);//LED
    pinMode(ESTOP_ClosetBTN, INPUT_PULLUP); // Set the pin as input with internal pullup resistor
    attachInterrupt(digitalPinToInterrupt(ESTOP_ClosetBTN), isrESTOPCloset, CHANGE); // Attach the interrupt with RISING mode

    upperState  = digitalRead(ESTOP_UpperBTN);
    lowerState  = digitalRead(ESTOP_LowerBTN);
    carState    = digitalRead(ESTOP_CarBTN);
    closetState = digitalRead(ESTOP_ClosetBTN);

    update();        
}



    
    

/*

shiftStepper 0.0.1 Alpha

This is example code for driving unipolar stepper motors off of a Logos Electromechanical
sixteen channel high current driver shield for Arduino. 

(c) Logos Electromechanical LLC 2010
Licensed under CC-BY-NC-SA 3.0 

*/

#include <ShiftStepper.h>

#define DATPIN    13  
#define SCLPIN    12
#define LATPIN    7
#define MRPIN     8  
#define INDPIN    2

// Set up the step sequence for a random motor...
static const uint8_t stepSequence[4] = {0x2, 0x4, 0x1, 0x8};
// Set up channel combinations for the motors...
static const uint8_t motChans0[__channelsPerMotor__] = {0,2,3,1};
static const uint8_t motChans1[__channelsPerMotor__] = {4,5,6,7};
static const uint8_t motChans2[__channelsPerMotor__] = {11,9,10,8};
static const uint8_t motChans3[__channelsPerMotor__] = {15,14,13,12};

// Declare some required globals

shiftChain *myChain = 0; // initializes so everyone can see it, but doesn't call the constructor
shiftStepMotor motor0(__channelsPerMotor__, stepSequence, motChans0);
shiftStepMotor motor1(__channelsPerMotor__, stepSequence, motChans1);
shiftStepMotor motor2(__channelsPerMotor__, stepSequence, motChans2);
shiftStepMotor motor3(__channelsPerMotor__, stepSequence, motChans3);
shiftDevice *motors[4] = {&motor0, &motor1, &motor2, &motor3};
shiftSixteen board0(4, motors);
shiftBoard *boards[1] = {&board0};
shiftChain storeChain(1, boards, DATPIN, SCLPIN, LATPIN, MRPIN, INDPIN);

/* ISR functions to shift it all out */

ISR (TIMER2_OVF_vect)
{
  myChain->doTick();
}

void setup (void) 
{
  myChain = &storeChain;
  Serial.begin(57600); 
  Serial.println("I live!"); 
  myChain->startTimer(__preScaler32__, 0, 2);
  //setupTimer();
  Serial.println("Timer started"); 
}

void loop (void) {
  shiftStepMotor *thisMotor;
  shiftBoard *thisBoard;
  char in = 0;                        // input character
  if (Serial.available() > 0) {
    in = Serial.read();
    switch (in) {
      case '+':
        ((shiftStepMotor*)myChain->getBoard(0)->getDev(0))->incrStep(1);
        ((shiftStepMotor*)myChain->getBoard(0)->getDev(1))->incrStep(1);
        ((shiftStepMotor*)myChain->getBoard(0)->getDev(2))->incrStep(1);
        ((shiftStepMotor*)myChain->getBoard(0)->getDev(3))->incrStep(1);
        Serial.print("Positive one step: 0x");
        Serial.println(((shiftStepMotor*)myChain->getBoard(0)->getDev(0))->getSeqStep(), HEX);
        break;
      case '-':
        ((shiftStepMotor*)myChain->getBoard(0)->getDev(0))->incrStep(-1);
        ((shiftStepMotor*)myChain->getBoard(0)->getDev(1))->incrStep(-1);
        ((shiftStepMotor*)myChain->getBoard(0)->getDev(2))->incrStep(-1);
        ((shiftStepMotor*)myChain->getBoard(0)->getDev(3))->incrStep(-1);
        Serial.print("Negative one step: 0x");
        Serial.println(((shiftStepMotor*)myChain->getBoard(0)->getDev(0))->getSeqStep(), HEX);
        break;
    }
  } 
  delay(10);
} 
  



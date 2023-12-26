#include "state_machine.h"

CustomStepper J1(7,6,0,4000,100);
CustomStepper J2(2,3,0,4000,100);
CustomStepper J3(11,10,0,4000,100);
CustomStepper J4(9,8,0,10000,100);
CustomStepper J5(4,5,0,10000,100);
StateMachine* stateMachine = new StateMachine();

void setup()
{
  Serial.begin(9600);
  stateMachine->setStepper(0,&J1);
  stateMachine->setStepper(1,&J2);
  stateMachine->setStepper(2,&J3);
  stateMachine->setStepper(3,&J4);
  stateMachine->setStepper(4,&J5);
}

void loop()
{  
  stateMachine->processSerial();
  stateMachine->processNextCommand();
  stateMachine->run();
  delayMicroseconds(1);
}
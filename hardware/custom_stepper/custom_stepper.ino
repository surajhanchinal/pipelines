#include "state_machine.h"

CustomStepper J1(2,4,0,0,4000,100,4000,"J1");
CustomStepper J2(16,14,1,0,4000,100,3600,"J2");
CustomStepper J3(23,21,1,0,4000,100,3600,"J3");
CustomStepper J4(38,36,1,0,4000,100,3600,"J4");
CustomStepper J5(6,8,0,0,4000,100,3600,"J5");

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
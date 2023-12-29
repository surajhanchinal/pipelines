#include "state_machine.h"

CustomStepper J1(4,5,0,0,8000,100,1000,"J1");
CustomStepper J2(9,8,0,1,8000,100,900,"J2");
CustomStepper J3(2,3,1,0,4000,100,900,"J3");
CustomStepper J4(7,6,1,0,4000,100,900,"J4");
CustomStepper J5(11,10,0,0,4000,100,900,"J5");

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
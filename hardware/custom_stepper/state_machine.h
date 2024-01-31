#pragma once
#include <string.h>
#include "line_queue.h"
#include "custom_stepper.h"

enum Command
{
  MOVE,
  RELATIVE_MOVE,
  BLOCKING_RELATIVE_MOVE,
  SET_ACCELERATION,
  RESET_POSITION,
  UNKNOWN_COMMAND,
  STATUS
};

class StateMachine
{
private:
  char* buffer;
  int buffer_limit;
  int buf_index = 0;
  int readHead = 0;
  char* currLine;
  int currLineLength = 0;
  LineQueue qq;
  CustomStepper* steppers[5];
  long targets[100][5];
  int targetIndex = 0;
  int moveCommands[100];
  int currStepperIndex[5];
  int targetsLen = 0;
  long accelerationValues[3];
  bool blockUntilSync = false;
  bool blockUntilAllStopped = false;
  bool moving = false;

  Command parseCommand()
  {
    while(readHead < currLineLength){
      if(currLine[readHead] == ' '){
        readHead++;
      }
      else{
        break;
      }
    }
    if (currLineLength == readHead)
    {
      return UNKNOWN_COMMAND;
    }
    if (currLineLength - readHead >= 2 && (currLine[readHead+0] == 'M' || currLine[readHead+0] == 'm') && (currLine[readHead+1] == 'R' || currLine[readHead+1] == 'r'))
    {
      readHead += 2;
      return RELATIVE_MOVE;
    }
    if (currLineLength - readHead >= 2 && (currLine[readHead+0] == 'M' || currLine[readHead+0] == 'm') && (currLine[readHead+1] == 'B' || currLine[readHead+1] == 'b'))
    {
      readHead += 2;
      return BLOCKING_RELATIVE_MOVE;
    }
    else if (currLine[readHead+0] == 'M' || currLine[readHead+0] == 'm')
    {
      readHead++;
      return MOVE;
    }
    else if (currLineLength - readHead >= 2 && (currLine[readHead+0] == 'S' || currLine[readHead+0] == 's') &&
             (currLine[readHead+1] == 'A' || currLine[readHead+1] == 'a'))
    {
      readHead += 2;
      return SET_ACCELERATION;
    }
    else if (currLineLength - readHead >= 2 && (currLine[readHead+0] == 'R' || currLine[readHead+0] == 'r') &&
             (currLine[1] == 'P' || currLine[1] == 'p'))
    {
      readHead += 2;
      return RESET_POSITION;
    }
    else if (currLineLength - readHead >= 2 && (currLine[readHead+0] == 'G' || currLine[readHead+0] == 'g') &&
             (currLine[readHead+1] == 'S' || currLine[readHead+1] == 's'))
    {
      readHead += 2;
      return STATUS;
    }
    return UNKNOWN_COMMAND;
  }

  void parseMoves(Command cmd)
  {
    targetIndex = (targetIndex + 1) % 100;
    targetsLen++;
    moveCommands[targetIndex] = cmd;

    currLine = currLine + readHead;
    char* next;
    for (int i = 0; i < 5; i++)
    {
      long& jp = targets[targetIndex][i];
      jp = strtol(currLine, &next, 10);
      if (currLine == next && i != 4)
      {
        return;
      }
      currLine = next;
    }
    long j2_target = targets[targetIndex][1];
    long j3_target = targets[targetIndex][2] + j2_target / 3;
    long j4_target = targets[targetIndex][3] - targets[targetIndex][4] + targets[targetIndex][2] / 3 + 2*(j2_target / 9.0);
    long j5_target = targets[targetIndex][3] + targets[targetIndex][4] + targets[targetIndex][2] / 3 + 2*(j2_target / 9.0);
    targets[targetIndex][1] = targets[targetIndex][1];
    targets[targetIndex][2] = j3_target;
    targets[targetIndex][3] = j4_target;
    targets[targetIndex][4] = j5_target;
  }

  void parseSetAcceleration()
  {
    currLine = currLine + readHead;
    char* next;
    for (int i = 0; i < 3; i++)
    {
      long& av = accelerationValues[i];
      av = strtol(currLine, &next, 10);
      if (currLine == next && i != 2)
      {
        return;
      }
      currLine = next;
    }
    steppers[accelerationValues[0]]->setSlope(accelerationValues[1]);
    steppers[accelerationValues[0]]->setConstant(accelerationValues[2]);
  }

  void returnStatus()
  {
    long positions[5];
    positions[0] = steppers[0]->getPosition();
    positions[1] = steppers[1]->getPosition();
    positions[2] = steppers[2]->getPosition() - steppers[1]->getPosition() / 3;
    positions[3] = (steppers[3]->getPosition() + steppers[4]->getPosition()) / 2 - steppers[3]->getPosition() / 3 +
                   steppers[4]->getPosition() / 9;
    positions[4] = (steppers[4]->getPosition() - steppers[3]->getPosition()) / 2;
    for (int i = 0; i < 5; i++)
    {
      Serial.print("Stepper ");
      Serial.print(i + 1);
      Serial.print(" : ");
      Serial.print("m: ");
      Serial.print(steppers[i]->getSlope());
      Serial.print(", c: ");
      Serial.print(steppers[i]->getConstant());
      Serial.print(", pos: ");
      Serial.println(positions[i]);
    }
  }

  void resetPosition()
  {
    for (int i = 0; i < 5; i++)
    {
      steppers[i]->resetPosition();
    }
  }

public:
  StateMachine()
  {
    buffer_limit = 1000;
    buffer = new char[buffer_limit];
    for(int i=0;i<5;i++){
      currStepperIndex[i] = 0;
    }
  }

  void setStepper(int index, CustomStepper* stepper)
  {
    steppers[index] = stepper;
  }

  void processSerial()
  {
    /*if(moving){
        return;
    }*/

    while (Serial.available())
    {
      char c = Serial.read();
      buffer[buf_index++] = c;
      if (c == '|' || c == '\n')
      {
        buffer[buf_index - 1] = 0;
        char* line = new char[buf_index];
        strcpy(line, buffer);
        qq.add(line);
        buf_index = 0;
      }
    }
  }

  void processNextCommand()
  {
    if (blockUntilAllStopped)
    {
      return;
    }
    if (qq.isEmpty())
    {
      return;
    }

    currLine = qq.get();
    currLineLength = strlen(currLine);
    readHead = 0;
    Command cmd = parseCommand();
    if (moving and !(cmd == MOVE or cmd == RELATIVE_MOVE or cmd == BLOCKING_RELATIVE_MOVE or cmd == SET_ACCELERATION))
    {
      blockUntilAllStopped = true;
      return;
    }
    qq.pop();
    switch (cmd)
    {
      case MOVE:
        parseMoves(cmd);
        break;
      case RELATIVE_MOVE:
        parseMoves(cmd);
        break;
      case BLOCKING_RELATIVE_MOVE:
        parseMoves(cmd);
        break;
      case SET_ACCELERATION:
        parseSetAcceleration();
        break;
      case RESET_POSITION:
        resetPosition();
        break;
      case STATUS:
        returnStatus();
        break;
      default:
        break;
    }
  }

  void run()
  {
    bool allDone = true;
    bool postDone = true;
    for (int i = 0; i < 5; i++)
    {
      bool done = steppers[i]->done;
      allDone = done and allDone;
      if (done)
      {
        int currCommand = currStepperIndex[i] % 100;
        if (moveCommands[currCommand] == BLOCKING_RELATIVE_MOVE)
        {
          if (blockUntilSync)
          {
            continue;
          }
        }
        if (currStepperIndex[i] < targetsLen)
        {
          currStepperIndex[i] = (currStepperIndex[i] + 1);
          int newCommandIndex = currStepperIndex[i] % 100;
          int cmd = moveCommands[newCommandIndex];
          if (cmd == BLOCKING_RELATIVE_MOVE or cmd == RELATIVE_MOVE)
          {
            steppers[i]->setRelativeTarget(targets[newCommandIndex][i]);
          }
          else if (cmd == MOVE)
          {
            steppers[i]->setTarget(targets[newCommandIndex][i]);
          }
        }
      }
      steppers[i]->run();
      bool pDone = steppers[i]->done;
      postDone = done and pDone;
    }
    if (allDone)
    {
      blockUntilSync = false;
    }
    // Basically did not move and we can read other non-move commands now
    if (allDone and postDone)
    {
      blockUntilAllStopped = false;
      moving = false;
    }
    else {
      moving = true;
    }
  }

};
#pragma once
#include <string.h>
#include "line_queue.h"
#include "custom_stepper.h"

enum Command {
  MOVE,
  RELATIVE_MOVE,
  SET_ACCELERATION,
  RESET_POSITION,
  UNKNOWN_COMMAND,
};

class StateMachine {

    private:
        char* buffer;
        int buffer_limit;
        int buf_index = 0;
        int readHead = 0;
        char* currLine;
        int currLineLength = 0;
        LineQueue qq;
        CustomStepper* steppers[5];
        long targets[5];
        long accelerationValues[3];

        bool moving = false;

        Command parseCommand(){
            if(currLineLength == 0){
                return UNKNOWN_COMMAND;
            }
            if(currLineLength >= 2 && (currLine[0] == 'M' || currLine[0] == 'm') && (currLine[1] == 'R' || currLine[1] == 'r')){
                readHead += 2;
                return RELATIVE_MOVE;
            }
            else if(currLine[0] == 'M' || currLine[0] == 'm'){
                readHead++;
                return MOVE;
            }
            else if(currLineLength >= 2 && (currLine[0] == 'S' && currLine[0] == 's') && (currLine[1] == 'A' || currLine[1] == 'a')){
                readHead += 2;
                return SET_ACCELERATION;
            }
            else if(currLineLength >= 2 && (currLine[0] == 'R' && currLine[0] == 'r') && (currLine[1] == 'P' || currLine[1] == 'p')){
                readHead += 2;
                return RESET_POSITION;
            }
            return UNKNOWN_COMMAND;
        }


        void parseMove(){
            currLine = currLine + readHead;
            char *next;
            for(int i=0;i<5;i++){
                long &jp = targets[i];
                jp = strtol(currLine,&next,10);
                if(currLine == next && i != 4){
                    return;
                }
                currLine = next;
            }
            
            targets[2] = targets[2] - targets[1]/3;
            targets[3] = targets[3] + targets[1]/9 - targets[2]/3;
            targets[4] = targets[4] + targets[1]/9 - targets[2]/3;
            for(int i=0;i<5;i++){
                steppers[i]->setTarget(targets[i]);
            }
        }

        void parseRelativeMove(){
            currLine = currLine + readHead;
            char *next;
            for(int i=0;i<5;i++){
                long &jp = targets[i];
                jp = strtol(currLine,&next,10);
                if(currLine == next && i != 4){
                    return;
                }
                currLine = next;
            }
            
            targets[2] = targets[2] - targets[1]/3;
            targets[3] = targets[3] + targets[1]/9 - targets[2]/3;
            targets[4] = targets[4] + targets[1]/9 - targets[2]/3;
            for(int i=0;i<5;i++){
                steppers[i]->setRelativeTarget(targets[i]);
            }
        }

        void parseSetAcceleration(){
            currLine = currLine + readHead;
            char *next;
            for(int i=0;i<3;i++){
                long &av = accelerationValues[i];
                av = strtol(currLine,&next,10);
                if(currLine == next && i != 2){
                    return;
                }
                currLine = next;
            }
            steppers[accelerationValues[0]]->setSlope(accelerationValues[1]);
            steppers[accelerationValues[0]]->setConstant(accelerationValues[2]);
        }

        void resetPosition(){
            for(int i=0;i<5;i++){
                steppers[i]->resetPosition();
            }
        }

        

    public:


        StateMachine(){
            buffer_limit = 1000;
            buffer = new char[buffer_limit];
        }

        void setStepper(int index,CustomStepper *stepper){
            steppers[index] = stepper;
        }


        void processSerial(){
            if(moving){
                return;
            }

            while(Serial.available()) {
                char c = Serial.read();
                buffer[buf_index++] = c;
                if(c == '|'  || c == '\n'){
                    buffer[buf_index-1] = 0;
                    char* line = new char[buf_index];
                    strcpy(line,buffer);
                    qq.add(line);
                    buf_index = 0;           
                }
            }
        }

        void processNextCommand(){
            if(moving){
                return;
            }
            if(qq.isEmpty()){
              return;
            }
            
            currLine = qq.get();
            currLineLength = strlen(currLine);
            readHead = 0;
            Command cmd = parseCommand();
            switch (cmd)
            {
            case MOVE:
                parseMove();
                break;
            case RELATIVE_MOVE:
              parseRelativeMove();
              break;
            case SET_ACCELERATION:
                parseSetAcceleration();
                break;
            case RESET_POSITION:
                resetPosition();
                break;            
            default:
                break;
            }
        }

        void run(){
            bool done = true;
            for(CustomStepper* stp : steppers){
                done = done && stp->done;
                stp->run();
            }
            moving = !done;
        }


};
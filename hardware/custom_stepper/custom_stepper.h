#pragma once

class CustomStepper {
    private:
        float _m,_c;
        long currPos = 0;
        int _pulsePin = 0,_dirPin = 0;
        long _toGo = 0,elapsed = 0;
        int _dir = 1;
        long _startSpeed = 0;
        long timeToNextPulse = 0;
        long startTime,endTime;
        long prevStepTime = 0;
        float speed = 0;
        float acceleration = 0;
        long stepsTo360 = 0;
        String name;
        bool reverse = 0;

        void setDirection(int dir){
          if(reverse){
            digitalWrite(_dirPin,!dir);
          }
          else {
            digitalWrite(_dirPin,dir);
          }
        }

        void step(){
            digitalWrite(_pulsePin,HIGH);
            delayMicroseconds(1);
            digitalWrite(_pulsePin,LOW);
            elapsed++;
            currPos += _dir;    
            currPos = currPos % stepsTo360;
        }

        float getAcceleration(){
            int acc_dir = elapsed < _toGo/2 ? 1 : -1;
            return acc_dir*(_m*speed + _c);
        }
    public:
        bool done = true;

        CustomStepper(int PULSE_PIN,int DIR_PIN,bool _reverse,float m,float c,long startSpeed,long _stepsTo360,String _name) {
            _pulsePin = PULSE_PIN;
            _dirPin = DIR_PIN;
            _m = m;
            _c = c;
            _startSpeed = startSpeed;
            pinMode(PULSE_PIN,OUTPUT);
            pinMode(DIR_PIN,OUTPUT);
            stepsTo360 = _stepsTo360;
            name = _name;
            reverse = _reverse;
        }

        void resetPosition(){
            currPos = 0;
        }

        void setTarget(long target) {
            target = target % stepsTo360;
            done = false;
            elapsed = 0;
            _toGo = target - currPos;
            int dir = _toGo > 0 ? 1 : 0;
            _dir = _toGo > 0 ? 1 : -1;
            _toGo = abs(_toGo);
            setDirection(dir);
            speed = _startSpeed;
            timeToNextPulse = (long)(((float)1000000.0)/speed);
            if(_toGo == 0){
              done = true;
            }
        }

        void setRelativeTarget(long delta){
            long target = delta + currPos;
            setTarget(target);
        }

        void run(){
            if(elapsed == _toGo){
                endTime = micros();
                if(!done){
                    long takenMs = (endTime - startTime)/1000;
                    Serial.print(name);
                    Serial.print(" : ");
                    Serial.println(takenMs);
                }
                done = true;
                return;
            }

            if(elapsed == 0){
                startTime = micros();
                step();
                prevStepTime = micros();
                return;
            }
            long currTime = micros();
            if(currTime - prevStepTime >= timeToNextPulse){
                step();
                prevStepTime = micros();
                speed = speed + getAcceleration()/speed;
                timeToNextPulse = (long)(((float)1000000.0)/speed);
            }
        }

        long getModuloTarget(long target){
          return target % stepsTo360;
        }

        float getSlope(){
          return _m;
        }

        float getConstant(){
          return _c;
        }

        long getPosition(){
          return currPos;
        }

        void setSlope(float slope){
            _m = slope;
        }

        void setConstant(float c){
            _c = c;
        }
};

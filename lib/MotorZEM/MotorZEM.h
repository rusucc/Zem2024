#ifndef MotorZEM_h
#define MotorZEM_h

#include "Arduino.h"
#include "PIDZEM.h"
#include "const.h"
class MotorZEM
{
public:
    volatile long count, countAbs; // count pentru pulsuri, se reseteaza
    double speed, targetSpeed, rotations, targetRotations, rotationsAbs;
    int runMode, cpr, out = 0, PWM = 0, reductor;
    int IN1, IN2, enc, ENABLE;
    bool direction = 1; //forward
    PIDZEM PID;
    bool arrived = false;
    MotorZEM(int IN1, int IN2, int enc, int ENABLE, int SLEW, double KPM, double KIM, double KDM, int reductor, int cpr);
    inline void calculateRotations()
    {
        rotations = double(count) / cpr / reductor;
    }
    inline void calculateRotationsAbs()
    {
        rotationsAbs = double(countAbs) / cpr / reductor;
    }
    inline void setTargetRotations(int rot)
    {
        targetRotations = rot;
    }
    inline void calculateSpeed()
    {
        speed = (rotations * 1000) / dt; // rotatii pe secunda
    }
    inline void setTargetSpeed(double v)
    {
        targetSpeed = v;
    }
    inline void setRunMode(int mode)
    {
        if (mode == 0)
        {
            runMode = 0;
        } // viteza continua
        else if (mode == 1)
        {
            runMode = 1;
            countAbs = 0;
            rotationsAbs = 0;
            arrived = 0;
        } // oprire cand ajunge la viteza
    }
    inline void updateCount(volatile int count)
    {
        this->count = count;
        if (runMode == 1)
            this->countAbs += count;
    }
    inline void writePWM()
    {
        PWM = (PWM < 0) ? 0 : PWM;
        PWM = (PWM > max_pwm) ? max_pwm : PWM;
        if(direction == 1 ){
            analogWrite(IN1, PWM);
            analogWrite(IN2, 0);
        }
        else{
            analogWrite(IN1, 0);
            analogWrite(IN2, PWM);
        }
    }
    inline void setDirection(int dir){
        direction = dir;
    }
    inline void run()
    {
        if (runMode == 0)
        {
            this->writePWM();
        }
        if (runMode == 1)
        {
            if (abs(targetRotations - rotationsAbs) > 1 and targetRotations > rotationsAbs)
            {
                this->writePWM();
            } // trimite pwmul pe pin daca nu este in zona si inca nu a depasit zona
            else
                arrived = true;
        }
    }
    inline void setPWM(int val)
    {
        PWM = val;
    }
    String printV();
    String printPID();
    String printAll();
    String printCSV();
    String printRotations();
};
#endif
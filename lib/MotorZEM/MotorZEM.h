#ifndef MotorZEM_h
#define MotorZEM_h

#include "Arduino.h"
#include "PIDZEM.h"
#include "const.h"
class MotorZEM
{
public:
    double targetRPM, freq, rpm,count;
    double PWM=0;
    int runMode, cpr, out = 0, reductor;
    int IN1, IN2, enc1, ENABLE;
    PIDZEM PID;
    bool arrived = false;
    MotorZEM(int IN1, int IN2, int enc1, int ENABLE, int SLEW, double KPM, double KIM, double KDM, int reductor, int cpr);
    inline void setTargetRPM(double v)
    {
        targetRPM = v;
    }
    inline void updateCount(double c){
        count = c;
    }
    inline void updateFreq(volatile double freq)
    {
        this->freq = freq;
    }
    inline void calculateSpeed(){
        rpm = freq/cpr;
    }
    inline void writePWM()
    {
        PWM = (PWM < 0) ? 0 : PWM;
        PWM = (PWM > 255) ? 255 : PWM;
        analogWrite(IN1, PWM);
        analogWrite(IN2, 0);
    }
    inline void run()
    {
        this->writePWM();
    }
    inline void setPWM(int val)
    {
        PWM = val;
    }
    String printRPM();
    String printPID();
    String printAll();
    String printCSV();
};
#endif
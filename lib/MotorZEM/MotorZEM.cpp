#include "MotorZEM.h"

MotorZEM::MotorZEM(int IN1, int IN2, int enc, int ENABLE, int SLEW, double KPM, double KIM, double KDM, int reductor, int cpr)
{
    this->IN1 = IN1;
    this->IN2 = IN2;
    this->ENABLE = ENABLE;
    this->enc = enc;
    PID = PIDZEM(KPM, KIM, KDM);
    this->reductor = reductor;
    digitalWrite(ENABLE, HIGH);
    digitalWrite(SLEW, HIGH);
    this->runMode = 0;
    this->cpr = cpr;
}
String MotorZEM::printV()
{
    return "V: " + String(speed) + " Vt: " + String(targetSpeed);
};
String MotorZEM::printPID()
{
    return "P: " + String(PID.KP) + " I: " + String(PID.KI) + " D: " + String(PID.KD) + " E: " + String(speed - targetSpeed) + " O: " + String(out) + " I:" + String(PID.integral);
};
String MotorZEM::printAll()
{
    return "V: " + String(speed) + " Vt: " + String(targetSpeed) + " Output PID: " + String(out) + " PWM: " + String(PWM) + " E: " + String(speed - targetSpeed) + " I:" + PID.integral;
};
String MotorZEM::printCSV()
{ // v,vt,target,out,pwm,e,i
    return "," + String(speed) + "," + String(targetSpeed) + "," + String(out) + "," + String(PWM) + "," + String(speed - targetSpeed) + "," + PID.integral;
};
String MotorZEM::printRotations()
{
    return "target" + String(targetRotations) + " countAbs:" + String(countAbs) + " rotationsAbs:" + String(rotationsAbs);
};
/*
inline void MotorZEM::calculateRotations()
{
    rotations = double((count / cpr)) / reductor;
}
inline void MotorZEM::calculateRotationsAbs()
{
    rotationsAbs = double((countAbs / cpr)) / reductor;
}
inline void MotorZEM::setTargetRotations(int rot)
{
    targetRotations = rot;
}
inline void MotorZEM::calculateSpeed()
{
    calculateRotations();
    speed = rotations * 1000 / dt; // rotatii pe secunda
}
inline void MotorZEM::setTargetSpeed(double v)
{
    targetSpeed = v;
}
inline void MotorZEM::setRunMode(int mode)
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
inline void MotorZEM::updateCount(volatile int count)
{
    this->count = count;
    if (runMode == 1)
        this->countAbs += count;
}
inline void MotorZEM::writePWM()
{
    PWM = (PWM < 0) ? 0 : PWM;
    PWM = (PWM > 255) ? 255 : PWM;
    analogWrite(IN1, PWM);
    analogWrite(IN2, 0);
}
inline void MotorZEM::run()
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
inline void MotorZEM::setPWM(int val)
{
    PWM = val;
}
*/
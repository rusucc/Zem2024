#include "MotorZEM.h"

MotorZEM::MotorZEM(int IN1, int IN2, int enc1, int ENABLE, int SLEW, double KPM, double KIM, double KDM, int reductor, int cpr)
{
    this->IN1 = IN1;
    this->IN2 = IN2;
    this->ENABLE = ENABLE;
    this->enc1 = enc1;
    PID = PIDZEM(KPM, KIM, KDM,0);
    this->reductor = reductor;
    digitalWrite(ENABLE, HIGH);
    digitalWrite(SLEW, HIGH);
    this->cpr = cpr;
    this->PID.k_filter=1;
}
String MotorZEM::printRPM()
{
    return "RPM: " + String(rpm) + " Vt: " + String(targetRPM);
};
String MotorZEM::printPID()
{
    return "KP: " + String(PID.KP) + " KI: " + String(PID.KI) + " KD: " + String(PID.KD) + " E: " + String(rpm - targetRPM) + " O: " + String(out) + " I:" + String(PID.integral);
};
String MotorZEM::printAll()
{
    return "V: " + String(rpm) + " Vt: " + String(targetRPM) + " Output PID: " + String(out) + " PWM: " + String(PWM) + " E: " + String(rpm - targetRPM) + " I:" + PID.integral;
};
String MotorZEM::printCSV()
{ // v,vt,target,out,pwm,e,i
    return "," + String(rpm) + "," + String(targetRPM) + "," + String(out) + "," + String(PWM) + "," + String(rpm - targetRPM) + "," + PID.integral;
};
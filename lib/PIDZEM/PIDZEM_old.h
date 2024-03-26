#ifndef PIDZEM_h
#define PIDZEM_h
class PIDZEM
{
public:
    double KP, KI, KD, integral = 0, e_old = 0;
    double maxI, minI;
    // e_old-eroare veche
public:
    PIDZEM(double KP = 0, double KI = 0, double KD = 0)
    {
        this->KP = KP;
        this->KI = KI;
        this->KD = KD;
    }
    void setK(double KP, double KI, double KD)
    {
        this->KP = KP;
        this->KI = KI;
        this->KD = KD;
    }
    void setLimits(double maxI = 10000, double minI = 0)
    {
        this->maxI = maxI;
        this->minI = minI;
    }
    double calculateOutput(double target, double current)
    {
        double e = current - target;
        integral += (KI * e);
        integral = (integral > 4000) ? 4000 : integral;
        integral = (integral < -4000) ? -4000 : integral;
        double out = KP * e + integral + KD * (e - e_old);
        e_old = e;
        return out;
    }
};
#endif
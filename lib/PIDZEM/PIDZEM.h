#ifndef PIDZEM_h
#define PIDZEM_h
class PIDZEM
{
public:
    double KP, KI, KD, integral = 0, e_old = 0;
    double maxI, minI;
    int integrator_type=0,index_errors=0;
    float k_filter;
    bool integrate_flag = true;
    double errors[10];
    // e_old-eroare veche
public:
    PIDZEM(double KP = 0, double KI = 0, double KD = 0,int integrator_type=0);
    inline void setK(double KP, double KI, double KD)
    {
        this->KP = KP;
        this->KI = KI;
        this->KD = KD;
    }
    inline void setLimits(double maxI = 10000, double minI = -10000)
    {
        this->maxI = maxI;
        this->minI = minI;
    }
    inline double calculateOutput(double target, double current){
        double e = current - target;
        if(integrate_flag){
            if(integrator_type==0) integral += (KI * e);
            else if(integrator_type==1){
                if (e*e_old<0) integral = 0;
                else integral += (KI * e);
            }
            else if(integrator_type==2){
                index_errors++;
                if(index_errors==10) index_errors = 0;
                integral-=errors[index_errors];
                errors[index_errors]=KI*e;
                integral+=errors[index_errors];
            }
            integral = (integral > maxI) ? maxI : integral;
            integral = (integral < minI) ? minI : integral;
        }
        else{
            integral = 0;
        }
        double out = KP * e + integral + KD * (e-e_old);
        e_old = e;

        return out;
    }
    inline void reset(){
        integral = 0;
        e_old = 0;
        for(int i=0;i<10;i++) errors[i]=0;
    }
    inline void stopIntegral(){
        integrate_flag = false;
    }
    inline void startIntegral(){
        integrate_flag = true;
    }
};
#endif
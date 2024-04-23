#include "PIDZEM.h"

PIDZEM::PIDZEM(double KP = 0, double KI = 0, double KD = 0,int integrator_type=0)
    {
        this->KP = KP;
        this->KI = KI;
        this->KD = KD;
        this->integrator_type = integrator_type;
        this->k_filter=1;
    }
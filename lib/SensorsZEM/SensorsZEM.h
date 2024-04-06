#ifndef SensorsZEM_h
#define SensorsZEM_h

#include "Arduino.h"
#include "const.h"
#include "PIDZEM.h"
class SensorsZEM
{
public:
    bool line;
    int read_number;
    struct calibrated_values
    {
        int max_value;
        int min_value;
    } calib[number];
    int values[number];
    int rawValues[number];
    int pins[number];
    const int threshold = 200;
    int position;
    int out;
    PIDZEM PID;
    SensorsZEM(double KPS, double KIS, double KDS, int pins[]);
    void readRaw();
    void calculatePosition();
    void calibrate(int cycles);
    void resetPID();
    String printValues();
    String printRawValues();
    String print();
};

#endif
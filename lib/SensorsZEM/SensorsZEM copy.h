/*#ifndef SensorsZEM_h
#define SensorsZEM_h
#include "Arduino.h"
#include "const.h"
#include "PIDZEM.h"
class SensorsZEM
{
public:
    bool line;
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
    void readRaw()
    {
        int T = millis();
        for (int i = 0; i < number; i++)
        {
            while (millis() - T < dts)
                ;
            T = millis();
            rawValues[i] = analogRead(pins[i]);
        }
    }
    SensorsZEM(double KPS, double KIS, double KDS, int pins[])
    {
        PID = PIDZEM(KPS, KIS, KDS);
        memcpy(this->pins, pins, number * sizeof(int));
        for (int i = 0; i < number; i++)
            calib[i].min_value = 0, calib[i].max_value = 1000;
    }
    void calculatePosition()
    {
        int sum = 0;
        int w_avg = 0;
        line = false;
        readRaw();
        for (int i = 0; i < number; i++)
        {
            values[i] = map(rawValues[i], calib[i].min_value, calib[i].max_value, 0, 1000);
            if (values[i] > threshold)
                line = true;
            if (values[i] > 100)
            {
                sum += values[i];
                w_avg += (i * 1000) * values[i];
            }
        }
        if (!line)
        {
            if (position < (number - 1) * 1000 / 2)
                position = 0;
            else
                position = (number - 1) * 1000;
        }
        else
            position = w_avg / sum;
    }
    void calibrate(int cycles)
    {
        for (int c = 0; c < cycles; c++)
        {
            for (int i = 0; i < number; i++)
            {
                int T = millis();
                while(millis() - T < dts);
                int temp_value = analogRead(pins[i]);
                calib[i].min_value = min(calib[i].min_value, temp_value);
                calib[i].max_value = max(calib[i].max_value, temp_value);
            }
        }
    }

    String printValues()
    {
        String ret = "";
        for (int i = 0; i < number; i++)
        {
            ret += String(values[i]);
            ret += " ";
        }
        return ret;
    }
    String printRawValues()
    {
        String ret = "";
        for (int i = 0; i < number; i++)
        {
            ret += String(rawValues[i]);
            ret += " ";
        }
        return ret;
    }

    String print()
    {
        return String(position) + " " + String(out);
    }
};
#endif
*/
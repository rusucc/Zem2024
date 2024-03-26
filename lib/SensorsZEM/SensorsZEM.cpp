#include "const.h"
#include "SensorsZEM.h"
void SensorsZEM::readRaw()
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
SensorsZEM::SensorsZEM(double KPS, double KIS, double KDS, int pins[])
{
    PID = PIDZEM(KPS, KIS, KDS);
    memcpy(this->pins, pins, number * sizeof(int));
    for (int i = 0; i < number; i++)
        calib[i].min_value = 0, calib[i].max_value = 1000;
}
void SensorsZEM::calculatePosition()
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
void SensorsZEM::calibrate(int cycles)
{
    for (int c = 0; c < cycles; c++)
    {
        for (int i = 0; i < number; i++)
        {
            int T = millis();
            while (millis() - T < dts)
                ;
            int temp_value = analogRead(pins[i]);
            calib[i].min_value = min(calib[i].min_value, temp_value);
            calib[i].max_value = max(calib[i].max_value, temp_value);
        }
    }
}

String SensorsZEM::printValues()
{
    String ret = "";
    for (int i = 0; i < number; i++)
    {
        ret += String(values[i]);
        ret += " ";
    }
    return ret;
}
String SensorsZEM::printRawValues()
{
    String ret = "";
    for (int i = 0; i < number; i++)
    {
        ret += String(rawValues[i]);
        ret += " ";
    }
    return ret;
}

String SensorsZEM::print()
{
    return String(position) + " " + String(out);
}
#include <Arduino.h>
#include <const.h>
#include <PIDZEM.h>
#include <MotorZEM.h>
#include <SensorsZEM.h>
#include <QTRSensors.h>
#include <FreqMeasureMulti.h>
// put function declarations here:
IntervalTimer t_line;
IntervalTimer t_telem;
IntervalTimer t_motor;

FreqMeasureMulti freq_m1;
FreqMeasureMulti freq_m2;
float sum1 = 0, sum2 = 0;
int count1 = 0, count2 = 0;
elapsedMillis timeout;

int pins_sensors[number] = {A8, A13, A12, A11, A10, A14, A15, A16};
int pins_sensors_lat[2] = {A1, A2};
MotorZEM M1 = MotorZEM(1, 8, 9, 28, 29, 0.22, 0.1, 0.03, 10, 3);
// 0.18 0.005, 0.03
MotorZEM M2 = MotorZEM(7, 5, 6, 34, 35, 0.22, 0.1, 0.03, 10, 3);
// MotorZEM(IN1, IN2, enc1, enc2, ENABLE, SLEW, KPM, KIM, KDM, reductor, cpr);
SensorsZEM QRE(0.008, 0, 0.025, pins_sensors);
int sensors_lat[2];

inline void update_motors()
{
  M1.out = M1.PID.calculateOutput(M1.targetRPM, M1.rpm);
  M2.out = M2.PID.calculateOutput(M2.targetRPM, M2.rpm);

  M1.PWM -= M1.out;
  M2.PWM -= M2.out;
}
inline void update_sensors()
{
  QRE.calculatePosition();
  QRE.out = QRE.PID.calculateOutput(3500, QRE.position);
  sensors_lat[0] = 1024 - analogRead(pins_sensors_lat[0]);
  sensors_lat[1] = 1024 - analogRead(pins_sensors_lat[1]);
}
inline void telemetry()
{
  //Serial.printf("RPM_M1: %f RPM_M2: %f \n", M1.rpm, M2.rpm);
}
void setup()
{
  delay(2000);
  Serial.println("Start calib");
  QRE.calibrate(200);
  Serial.println("Sfarsit calib");
  delay(1000);
  freq_m1.begin(M1.enc1);
  freq_m2.begin(M2.enc1);
  for (int i = 0; i < number; i++)
  {
    Serial.print(QRE.calib[i].min_value),Serial.print(" ");
  }
  Serial.println();
  for (int i = 0; i < number; i++)
  {
    Serial.print(QRE.calib[i].max_value),Serial.print(" ");
  }
  Serial.println();
  delay(1000);
  //t_motor.begin(update_motors, dt * 200);  // dt milisecunde
  t_telem.begin(telemetry, 250 * 1000);    // telemetrie la fiecare 200 ms
  t_line.begin(update_sensors, 30 * 1000); // 20 milisecunde intre citiri de senzori de linie
  // Serial.println("Setup end");
}

void loop()
{

  if (freq_m1.available())
  {
    sum1 = sum1 + freq_m1.read();
    count1 = count1 + 1;
  }
  if (freq_m2.available())
  {
    sum2 = sum2 + freq_m2.read();
    count2 = count2 + 1;
  }
  if (timeout > 200)
  {
    if (count1 > 0)
      M1.updateFreq(freq_m1.countToFrequency(sum1 / count1));
    else
      M1.updateFreq(0);

    if (count2 > 0)
      M2.updateFreq(freq_m2.countToFrequency(sum2 / count2));
    else
      M2.updateFreq(0);
    //Serial.printf("M1: %f || M2: %f \n",M1.rpm,M2.rpm);
    sum1 = 0;
    sum2 = 0;
    count1 = 0;
    count2 = 0;
    timeout = 0;
    M1.calculateSpeed();
    M2.calculateSpeed();
    update_motors();
    Serial.print(M1.printRPM());
    Serial.print(" ");
    Serial.println(M2.printRPM());
  }

  double baseSpeed = 20;
  int semnalM1 = baseSpeed+QRE.out;
  semnalM1 = semnalM1 > 0 ? semnalM1 : 0;

  int semnalM2 = baseSpeed-QRE.out;
  semnalM2 = semnalM2 > 0 ? semnalM2 : 0;

  // M2.setTargetSpeed(semnalM2>0? semnalM2:0);//speed - pulsuri pe dts milisecunde
  //M1.setTargetRPM(10);
  M2.setTargetRPM(semnalM2);
  M1.setTargetRPM(semnalM1);
  M1.run();
  M2.run();
}

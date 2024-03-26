#include <Arduino.h>
#include <const.h>
#include <PIDZEM.h>
#include <MotorZEM.h>
#include <SensorsZEM.h>
// put function declarations here:
IntervalTimer t_line;
IntervalTimer t_telem;
IntervalTimer t_motor;

volatile long count0 = 0;
volatile long count1 = 0;

void encA()
{
  count0++;
} // functie apelata pe interrupt pentru motorul A
void encB()
{
  count1++;
} // functie apelata pe interupt pentru motorul B

int pins_sensors[number] = {A8, A13, A12, A11, A10, A14, A15, A16};
int pins_sensors_lat[2] = {A1, A2};
MotorZEM M1 = MotorZEM(1, 8, 32, 28, 29, 0.0625, 0, 0, 10, 3);
MotorZEM M2 = MotorZEM(7, 5, 36, 34, 35, 1, 0, 0, 10, 3);
MotorZEM M3 = MotorZEM(1, 8, 32, 28, 29, 1, 0, 0, 10, 3);
// MotorZEM(IN1, IN2, enc, ENABLE, SLEW, KPM, KIM, KDM, reductor, cpr);
// MotorZEM(I7, 5, enc34, ENABLE, SLEW, KPM, KIM, KDM, reductor, cpr);
SensorsZEM QRE(0.03, 0, 0, pins_sensors);
int sensors_lat[2];

inline void update_motors()
{
  volatile int bufc0 = count0;
  volatile int bufc1 = count1;

  M1.updateCount(bufc0), count0 = 0;
  M2.updateCount(bufc1), count1 = 0;

  M1.calculateSpeed();
  M2.calculateSpeed();

  M1.out = M1.PID.calculateOutput(M1.targetSpeed, M1.speed);
  M2.out = M2.PID.calculateOutput(M1.targetSpeed, M1.speed);

  M1.PWM -= M1.out;
  M2.PWM -= M2.out;

  if (M1.runMode == 1)
    M1.countAbs += bufc0, M1.calculateRotationsAbs();
  if (M2.runMode == 1)
    M2.countAbs += bufc1, M2.calculateRotationsAbs();
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
}
void setup()
{
  Serial.begin(9600);
  t_motor.begin(update_motors, dt * 1000); // dt milisecunde
  t_telem.begin(telemetry, 200 * 1000);    // telemetrie la fiecare 200 ms
  t_line.begin(update_sensors, 20 * 1000); // 20 milisecunde intre citiri de senzori de linie
  attachInterrupt(digitalPinToInterrupt(M1.enc), encA, RISING);
  attachInterrupt(digitalPinToInterrupt(M2.enc), encB, RISING);
  M1.setRunMode(0);
  M2.setRunMode(0);
}

void loop()
{
  // put your main code here, to run repeatedly:
}

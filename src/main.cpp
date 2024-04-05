#include <Arduino.h>
#include <const.h>
#include <PIDZEM.h>
#include <MotorZEM.h>
#include <SensorsZEM.h>
#include <QTRSensors.h>
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
MotorZEM M1 = MotorZEM(1, 8, 32, 28, 29, 0.4, 0.1, 0.2, 10, 6);
//0.18 0.005, 0.03
MotorZEM M2 = MotorZEM(7, 5, 36, 34, 35, 0.4, 0.1, 0.2, 10, 6);
// MotorZEM(IN1, IN2, enc, ENABLE, SLEW, KPM, KIM, KDM, reductor, cpr);
SensorsZEM QRE(0.002, 0, 0, pins_sensors);
int sensors_lat[2];

inline void update_motors()
{
  int bufc0 = count0;
  int bufc1 = count1;
  
  M1.updateCount(bufc0), count0 = 0;
  M2.updateCount(bufc1), count1 = 0;
  
  M1.calculateRotations();
  M2.calculateRotations();

  M1.calculateSpeed();
  M2.calculateSpeed();

  M1.out = M1.PID.calculateOutput(M1.targetSpeed, M1.avg_speed);
  M2.out = M2.PID.calculateOutput(M2.targetSpeed, M2.avg_speed);

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
  //Serial.print(QRE.position),Serial.print(" "),Serial.println("3500");
  //Serial.print(M1.printPID()),Serial.print(" "),Serial.print(M1.printV()),Serial.print(" | ");
  //Serial.println(M2.printPID()),Serial.print(" "),Serial.println(M2.printV());
  //for(int i = 0; i<10;i++){
  //  Serial.print(M1.old_speeds[i]);
  //  Serial.print(" ");
  //}
  Serial.println(M1.printV());
}
void setup()
{
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(M1.enc), encA, RISING);
  attachInterrupt(digitalPinToInterrupt(M2.enc), encB, RISING);
  attachInterrupt(digitalPinToInterrupt(M1.enc), encA, FALLING);
  attachInterrupt(digitalPinToInterrupt(M2.enc), encB, FALLING);

  M1.setRunMode(0);
  M2.setRunMode(0);
  delay(2000);
  Serial.println("Start calib");
  //QRE.calibrate(200);
  Serial.println("Sfarsti calib");
  //delay(1000);
  for(int i=0;i<number;i++){
    //Serial.print(QRE.calib[i].min_value),Serial.print(" ");
  }
  //Serial.println();
  for(int i=0;i<number;i++){
    //Serial.print(QRE.calib[i].max_value),Serial.print(" ");
  }
  //Serial.println();
  delay(1000);
  t_motor.begin(update_motors, 100 * 1000); // dt milisecunde
  t_telem.begin(telemetry, 200 * 1000);    // telemetrie la fiecare 200 ms
  t_line.begin(update_sensors, 30 * 1000); // 20 milisecunde intre citiri de senzori de linie
  //Serial.println("Setup end");
}

void loop()
{
  double baseSpeed = 10;
  //int semnalM1 = baseSpeed+QRE.out;
  //int semnalM2 = baseSpeed-QRE.out;
  double semnalM1 = baseSpeed;
  double semnalM2 = baseSpeed;

  M2.setTargetSpeed(semnalM2>0? semnalM2:0);//speed - pulsuri pe dts milisecunde
  M1.setTargetSpeed(semnalM1>0? semnalM1:0);
  M1.run();
  M2.run();
}

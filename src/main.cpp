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
MotorZEM M1 = MotorZEM(1, 8, 32, 28, 29, 0.22, 0.005, 0.03, 10, 3);
//0.18 0.005, 0.03
MotorZEM M2 = MotorZEM(7, 5, 36, 34, 35, 0.22, 0.005, 0.03, 10, 3);
// MotorZEM(IN1, IN2, enc, ENABLE, SLEW, KPM, KIM, KDM, reductor, cpr);
SensorsZEM QRE(0.014, 0, 0.08, pins_sensors);
//15.0.11
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

  M1.out = M1.PID.calculateOutput(M1.targetSpeed, M1.speed);
  M2.out = M2.PID.calculateOutput(M2.targetSpeed, M2.speed);

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
  sensors_lat[0] = analogRead(pins_sensors_lat[0]);
  sensors_lat[1] = analogRead(pins_sensors_lat[1]);
}
inline void telemetry()
{
  //Serial.print(M1.PWM),Serial.print(" "),Serial.println(M2.PWM);
  Serial.print(sensors_lat[0]),Serial.print(" "),Serial.println(sensors_lat[1]);
}
void setup()
{
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(M1.enc), encA, RISING);
  attachInterrupt(digitalPinToInterrupt(M2.enc), encB, RISING);
  M1.setRunMode(0);
  M2.setRunMode(0);
  delay(2000);
  Serial.println("Start calib");
  QRE.calibrate(200);
  Serial.println("Sfarsti calib");
  delay(1000);
  for(int i=0;i<number;i++){
    Serial.print(QRE.calib[i].min_value),Serial.print(" ");
  }
  Serial.println();
  for(int i=0;i<number;i++){
    Serial.print(QRE.calib[i].max_value),Serial.print(" ");
  }
  Serial.println();
  delay(1000);
  t_motor.begin(update_motors, dt * 1000); // dt milisecunde
  t_telem.begin(telemetry, dt_telem * 1000);    // telemetrie la fiecare 200 ms
  t_line.begin(update_sensors, dt_line * 1000); // 20 milisecunde intre citiri de senzori de linie
  Serial.println("Setup end");
}

void loop()
{
  int baseSpeed = 30;
  //50
  int semnalM1 = baseSpeed+QRE.out;
  semnalM1 = semnalM1>0? semnalM1:0;

  int semnalM2 = baseSpeed-QRE.out;
  semnalM2 = semnalM2>0? semnalM2:0;
  /*if(QRE.read_number>2){
    if(sensors_lat[0]>500 and sensors_lat[1]>500) {
      M1.setPWM(baseSpeed);
      M2.setPWM(baseSpeed);
    } //intersectie dreapta
    else if(sensors_lat[0]>500) {
      M1.setPWM(2*baseSpeed);
      M2.setPWM(0*baseSpeed);
    }
    else if(sensors_lat[1]>500){
      M1.setPWM(0*baseSpeed);
      M2.setPWM(2*baseSpeed);
      }

    int T = millis();
    while(millis() - T<300) M1.run(),M2.run();
  }
  else M1.setPWM(semnalM1),M2.setPWM(semnalM2);
  */
  if(QRE.read_number>5){
    M1.setPWM(baseSpeed);
    M2.setPWM(baseSpeed);
    int T = millis();
    while(millis()-T<500)M1.run(),M2.run();
    update_sensors();
    if(QRE.line){
      M1.setPWM(baseSpeed+QRE.out);
      M2.setPWM(baseSpeed-QRE.out);
    }
    else{
      Serial.println("Caut linia!");
      t_line.end();
      QRE.resetPID();
      while(abs(QRE.position-3500)<150){ 
        M1.setPWM(30);
        M2.setPWM(0);
        M1.run();
        M2.run();
        update_sensors();
        delay(dt_line);
      }
      Serial.println("Am gasit linia");
      while(true) M1.setPWM(0),M2.setPWM(0),M1.run(),M2.run();
      while(sensors_lat[0]<500){
        update_sensors();
        M1.setPWM(30+QRE.out);
        M2.setPWM(30-QRE.out);
        M1.run();
        M2.run();
        delay(dt_line);
      }
      Serial.println("PID pe linie");
      M1.setRunMode(1);
      M2.setRunMode(1);
      M1.setTargetRotations(3);
      M2.setTargetRotations(6);
      while(!M1.arrived or !M2.arrived){
        if(!M1.arrived) M1.setPWM(30);
        else M1.setPWM(0);
        if(!M2.arrived) M2.setPWM(30);
        else M2.setPWM(0);
        M1.run();
        M2.run();
      }
      Serial.println("Am mers rotatiile");
      M1.setRunMode(0);
      M2.setRunMode(0);
      Serial.println("iesire din giratoriu");
      t_line.begin(update_sensors,dt_line*1000);
    }
  }
  else{
    M1.setPWM(semnalM1);
    M2.setPWM(semnalM2);
  }
  M1.run();
  M2.run();
  //int T = millis();
  //while(millis()-T<500);
  /*for(int i=0;i<number;i++){
    Serial.print(QRE.rawValues[i]),Serial.print(" ");
  }
  */
  //Serial.println();
}

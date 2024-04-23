#include <Arduino.h>
#include <const.h>
#include <PIDZEM.h>
#include <MotorZEM.h>
#include <SensorsZEM.h>
#include <HCSR04.h>
// put function declarations here:
String tip_intersectie;

IntervalTimer t_telem;

int pins_sensors[number] = {A8, A13, A12, A11, A10, A14, A15, A16};
int pins_sensors_lat[2] = {A3, A2};
MotorZEM M1 = MotorZEM(1, 8, 32, 28, 29, 0.22, 0.005, 0.03, 10, 3);
//0.18 0.005, 0.03
MotorZEM M2 = MotorZEM(7, 5, 36, 34, 35, 0.22, 0.005, 0.03, 10, 3);
// MotorZEM(IN1, IN2, enc, ENABLE, SLEW, KPM, KIM, KDM, reductor, cpr);
SensorsZEM QRE(0.01, 0.0001, 0.15, pins_sensors);
//15.0.11
//009|0001|062
int sensors_lat[2];

//senzori suplimentari:
const int echo_pin = 36;
const int trig_pin = 32;
UltraSonicDistanceSensor distanceSensor(trig_pin, echo_pin);
float wallDistance;
PIDZEM wallPID(3,0,1.5);
inline void turn_right_until_wall();
inline void update_ultrasonic();
inline void follow_wall();
inline void update_sensors()
{
  QRE.calculatePosition();
  QRE.out = QRE.PID.calculateOutput(3500, QRE.position);
  sensors_lat[0] = analogRead(pins_sensors_lat[0]);
  sensors_lat[1] = analogRead(pins_sensors_lat[1]);
}
inline void telemetry()
{
  for(int i=0;i<number;i++){
    Serial.printf("%04d | ",QRE.values[i]);
  }
  Serial.printf("Senzori laterali [0](dreapta): %04d | [1](stanga): %04d, intersectie:",sensors_lat[0],sensors_lat[1]);
  Serial.println(tip_intersectie);
  Serial.println(QRE.read_number);
  
  //Serial.println(tip_intersectie);
}
void setup()
{
  Serial.begin(9600);
  delay(100);
  Serial.println("Hello");
  //while(!Serial.available());
  M1.setRunMode(0);
  M2.setRunMode(0);
  QRE.PID.setLimits();
  delay(2000);
  Serial.println("Start calib");
  QRE.calibrate(200);
  Serial.println("Sfarsit calib");
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
  t_telem.begin(telemetry, dt_telem * 1000);    // telemetrie la fiecare 200 ms
  Serial.println("Setup end");
}

inline void follow_line(int pwm);
inline void turn_left_until_line();
inline void turn_right_until_line();
inline void forward(int ms);
inline void roundabout();
inline void stop();
void loop()
{
  update_sensors();
  update_ultrasonic();
  if(QRE.read_number>=min_number_sensors_read and (sensors_lat[0]>threshold_lateral or sensors_lat[1]>threshold_lateral)){
    tip_intersectie = "Existenta";
    forward(80);
    //stop();
    if(QRE.line) follow_line(base_pwm),QRE.PID.reset();
    else{
      tip_intersectie="Giratoriu";
      QRE.resetPID();
      roundabout();
      tip_intersectie="linie";
      QRE.resetPID();
      //stop();
    }
  }
  else{
    if(sensors_lat[0]>threshold_lateral and sensors_lat[1]<threshold_lateral  and QRE.read_number>=3 and QRE.read_number<=5){
      tip_intersectie="90 dreapta";
      forward(chicane_forward_time);
      turn_right_until_line();
      tip_intersectie="linie";
      QRE.PID.reset();
      //stop();
    }
    else if(sensors_lat[1]>threshold_lateral and sensors_lat[0]<threshold_lateral  and QRE.read_number>=3 and QRE.read_number<=5){
      tip_intersectie="90 stanga";
      forward(chicane_forward_time);
      turn_left_until_line();
      tip_intersectie="linie";
      QRE.PID.reset();
      //stop();
    }
    else if(!QRE.line and wallDistance>=0 and wallDistance<=10){
      while(wallDistance!=30) follow_wall();
      forward(100);
      turn_right_until_wall();
      while(!QRE.line) follow_wall();
      wallPID.reset();
      QRE.PID.reset();
    }
    else{
    follow_line(base_pwm);
    tip_intersectie="linie";
    }
  }
  //M1.run();
  //M2.run();
  //if(Serial.read()==0) stop();
}
inline void follow_wall(){
  QRE.PID.stopIntegral();
  update_ultrasonic();
  update_sensors();
  int outWallPID = wallPID.calculateOutput(wall_setpoint,wallDistance);
  update_sensors();
  M1.setPWM(wall_pwm-outWallPID);
  M2.setPWM(wall_pwm+outWallPID);
  M1.run();
  M2.run();
  elapsedMillis T;
  while(T<50);
  Serial.printf("Distanta perete: %f, \n",wallDistance);
}
inline void turn_right_until_wall(){
  QRE.PID.stopIntegral();
  elapsedMillis T = 0;
  while(wallDistance>=wall_setpoint){
    M1.setPWM(wall_inside_pwm+correction_motor);
    M1.setDirection(1);
    M2.setPWM(wall_outside_pwm-correction_motor);
    M1.run();
    M2.run();
    while(T<=stability_delay);
    T=0;
    update_sensors();
    update_ultrasonic();
  }
  M2.setDirection(1);
  M1.setDirection(1);
}
inline void update_ultrasonic(){
  wallDistance=distanceSensor.measureDistanceCm();
  if(wallDistance>30) wallDistance = 30;
}
inline void follow_line(int pwm){
  M1.setDirection(1);
  M2.setDirection(1);
  QRE.PID.startIntegral();
  int semnalM1 = pwm+QRE.out;
  semnalM1 = semnalM1>0? semnalM1:0;

  int semnalM2 = pwm-QRE.out;
  semnalM2 = semnalM2>0? semnalM2:0;
  M1.setPWM(semnalM1);
  M2.setPWM(semnalM2);
  M1.run();
  M2.run();
}
inline void forward(int ms){
  QRE.PID.stopIntegral();
  elapsedMillis T = 0;
  while(T<ms){
    M1.setDirection(1);
    M2.setDirection(1);
    M1.setPWM(forward_pwm+correction_motor);
    M2.setPWM(forward_pwm-correction_motor);
    M1.run();
    M2.run();
  }
  update_sensors();
}
inline void roundabout(){
  QRE.PID.stopIntegral();
  turn_left_until_line();
  elapsedMillis T = 0;
  while(sensors_lat[1]<threshold_lateral){
    follow_line(roundabout_pwm);
    update_sensors();
    while(T<=stability_delay);
    T=0;
  }
  forward(80);
  turn_left_until_line();
}
inline void turn_left_until_line(){
  QRE.PID.stopIntegral();
  elapsedMillis T = 0;
  while(QRE.values[4]<threshold_calibrated and QRE.values[5]<threshold_calibrated){
    M2.setDirection(0);
    M1.setPWM(turn_outside_pwm+correction_motor);
    M2.setPWM(turn_inside_pwm-correction_motor);
    M1.run();
    M2.run();
    while(T<=stability_delay);
    T=0;
    update_sensors();
  }
  M1.setDirection(1);
  M2.setDirection(1);
}
inline void turn_right_until_line(){
  QRE.PID.stopIntegral();
  elapsedMillis T = 0;
  while(QRE.values[4]<threshold_calibrated and QRE.values[5]<threshold_calibrated){
    M1.setPWM(turn_inside_pwm+correction_motor);
    M1.setDirection(0);
    M2.setPWM(turn_outside_pwm-correction_motor);
    M1.run();
    M2.run();
    while(T<=stability_delay);
    T=0;
    update_sensors();
  }
  M1.setDirection(1);
  M2.setDirection(1);
}
inline void stop(){
  QRE.PID.stopIntegral();
  while(true){
    digitalWrite(M1.ENABLE,LOW);
    digitalWrite(M2.ENABLE,LOW);
    M1.setPWM(0);
    M2.setPWM(0);
    M1.run();
    M2.run();
    Serial.println("stop");
    delay(200);
  }
}
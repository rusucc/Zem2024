#include <Arduino.h>
#include <const.h>
#include <PIDZEM.h>
#include <MotorZEM.h>
#include <SensorsZEM.h>
#include <HCSR04.h>
// put function declarations here:
String tip_intersectie;

IntervalTimer t_telem;
IntervalTimer t_ultrasonic;

int pins_sensors[number] = {A8, A13, A12, A11, A10, A14, A15, A16};
int pins_sensors_lat[2] = {A3, A2};
MotorZEM M1 = MotorZEM(1, 8, 32, 28, 29, 0.22, 0.005, 0.03, 10, 3);
//0.18 0.005, 0.03
MotorZEM M2 = MotorZEM(7, 5, 36, 34, 35, 0.22, 0.005, 0.03, 10, 3);
// MotorZEM(IN1, IN2, enc, ENABLE, SLEW, KPM, KIM, KDM, reductor, cpr);
SensorsZEM QRE(0.016, 0, 0.09, pins_sensors);
//15.0.11
int sensors_lat[2];

//senzori suplimentari:
const int echo_pin = 36;
const int trig_pin = 32;
UltraSonicDistanceSensor distanceSensor(trig_pin, echo_pin);
float wallDistance;
PIDZEM wallPID(8,0,0);

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
  Serial.println();
  //Serial.printf("| Senzori laterali [0](dreapta): %04d | [1](stanga): %04d, intersectie:",sensors_lat[0],sensors_lat[1]);
  //Serial.println(wallDistance);
}
void setup()
{
  Serial.begin(9600);
  delay(100);
  Serial.println("Hello");
  while(!Serial.available());
  M1.setRunMode(0);
  M2.setRunMode(0);
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
  t_ultrasonic.begin([](){wallDistance=distanceSensor.measureDistanceCm();}, dt_ultrasonic * 1000);
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
  if(QRE.line and wallDistance<=20){
    while(QRE.line){
      forward(20);
      update_sensors();
    }
    Serial.println("Stau doar pe perete");
    while(!QRE.line){
      //perete pe dreapta;
      int outWallPID = wallPID.calculateOutput(10,wallDistance);
      update_sensors();
      M1.setPWM(wall_pwm-outWallPID);
      M2.setPWM(wall_pwm+outWallPID);
      M1.run();
      M2.run();
      elapsedMillis T;
      while(T<30);
      Serial.printf("Distanta perete: %f\n",wallDistance);
    }
    Serial.println("Am gasit linia");
    stop();
  }
  if(QRE.read_number>min_number_sensors_read and sensors_lat[0]>threshold_lateral and sensors_lat[1]>threshold_lateral){
    tip_intersectie = "Existenta";
    forward(50);
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
    else{
    follow_line(base_pwm);
    tip_intersectie="linie";
    }
  }
}
inline void follow_line(int pwm){
  M1.setDirection(1);
  M2.setDirection(1);
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
  elapsedMillis T = 0;
  while(T<ms){
    M1.setPWM(forward_pwm+correction_motor);
    M2.setPWM(forward_pwm-correction_motor);
    M1.run();
    M2.run();
  }
  update_sensors();
}
inline void roundabout(){
  turn_left_until_line();
  elapsedMillis T = 0;
  while(sensors_lat[1]<threshold_lateral){
    follow_line(roundabout_pwm);
    update_sensors();
    while(T<=stability_delay);
    T=0;
  }
  turn_left_until_line();
}
inline void turn_left_until_line(){
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
}
inline void turn_right_until_line(){
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
  M2.setDirection(1);
}
inline void stop(){
  while(true){
    M1.setPWM(0);
    M2.setPWM(0);
    M1.run();
    M2.run();
    Serial.println("stop");
    delay(200);
  }
}
#include "car.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>



Servo SteeringServo; 

Car::Car(int servoPin, int encoderPin, int Ena, int in1, int in2, int wheelsize) {
 
  _servoPin = servoPin;
  _encoderPin = encoderPin;
  _Ena = Ena;
  _in1 = in1;
  _in2 = in2;
  _wheelsize = wheelsize;  
  
  pinMode(_encoderPin, INPUT_PULLUP); 
  pinMode(_Ena, OUTPUT); 
  pinMode(_in1, OUTPUT);
  pinMode(_in2, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

}

void Car::begin(double bdrate) {
  delay(30);
  Serial.begin(bdrate);      
  Serial.println("Started");
  Serial.print("in1 = ");  
  Serial.println(_in1);
  Serial.print("in2 = ");  
  Serial.println(_in2);
  Serial.print("enA = ");  
  Serial.println(_Ena);
  Serial.print("Encoder Pin = ");  
  Serial.println(_encoderPin);
  Serial.print("Steering Servo Pin = ");  
  Serial.println(_servoPin);
  Serial.print("Reflected light Pin = ");  
  Serial.println(_reflightPin);

  SteeringServo.attach(_servoPin);
  delay(30);

}

double Car::PIDcalc(double inp, int sp){
   currentTime = millis();                //get current time
   elapsedTime = (double)(currentTime - previousTime)/1000; //compute time elapsed from previous computation (60ms approx). divide in 1000 to get in Sec
   //Serial.print(currentTime); //for serial plotter
   //Serial.println("\t"); //for serial plotter
   error = sp - inp;                                  // determine error
   cumError += error * elapsedTime;                   // compute integral
   rateError = (error - lastError)/elapsedTime;       // compute derivative deltaError/deltaTime
   double out = kp*error + ki*cumError + kd*rateError; //PID output               
   //Serial.println(cumError);
   lastError = error;                                 //remember current error
   previousTime = currentTime;                        //remember current time
   if(out > 254){out = 254;}    //limit the function for smoother operation
   if(out < -254){out = -254;}
   if(cumError > 255 || cumError < -255){cumError = 0; out = 0;} // reset the Integral commulator
   if(rateError < 0.3 || rateError > -0.3){cumError = 0;}             // reset the Integral commulator
   return out;                                        //the function returns the PID output value 
  
}
long Car::getSteps(){
  if(digitalRead(_encoderPin)){ //1 = obstruction, 0 = hole
     steps = steps +1; 
   }
  return steps;
}

long Car::goencoder(int clicks, int times, double KP, double KI, double KD){
  kp = KP;
  ki = KI; 
  kd = KD;
  steps = 0; //reset the steps count
  for(int i = 0; i < times; i++) {
    int tempsteps = getSteps(); //input value
    int output = PIDcalc(tempsteps, clicks); //output value = the calculated error
    //lcdenshow(clicks, output, tempsteps);
    delay(30);
    if (output > 0){  // 
    move(output);
    }
    if (output < 0){  // 
    stop();
    } 
  }
 stop(); //reset the motor after mmoving
 return(steps);
}
void Car::move(int speed){
  if(speed > 0){
    digitalWrite(_in1, HIGH);
    digitalWrite(_in2, LOW);
    analogWrite(_Ena, speed);
  }
  else if(speed < 0){
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, HIGH);
    analogWrite(_Ena, abs(speed));
  }
  else{stop();}
  
}

void Car::stop(){
  digitalWrite(_in1, HIGH);
  digitalWrite(_in2, HIGH);
  analogWrite(_Ena, 0);
}  
void Car::steer(int dir){
  //if(dir)
  SteeringServo.write(dir); 
}
long Car::getrefligh(){
  int x = analogRead(A1);
  return(x);
}
void Car::trackline(int speed, int color, double KP, double KI, double KD){
  kp = KP;
  ki = KI; 
  kd = KD;
  int tempcolor = getrefligh(); //input value
  int output = PIDcalc(tempcolor, color); //output value = the calculated error
  move(speed);
  steer(output);
}
void Car::gomm(long distancemm){ //covert to encoder clicks
  //one click is perimeter divided by 20 holes (in my encoder wheel)
  long clicks = distancemm / ((PI * _wheelsize) / 20);
  Serial.println(goencoder(clicks, 200, 3, 0, 0));
}
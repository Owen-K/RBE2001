#include <Arduino.h>
#include <Chassis.h>
#include <Rangefinder.h>
#include "line.h"

Chassis chassis;
LeftMotor leftMotor;
RightMotor rightMotor;
Rangefinder rangefinder = Rangefinder(2, 3);

void setup() {
  chassis.init();
  chassis.setMotorPIDcoeffs(5, 0.5);
  Serial.begin(115200);
  rangefinder.init();

}


void TwistOffLineAndLook(){
  float Vright = analogRead(A2);
  delay(1000);
  chassis.turnFor(20,100, true);
  delay(500);
  while(Vright < 100){
    leftMotor.setMotorEffort(-30);
    rightMotor.setMotorEffort(30);
    Vright = analogRead(A2);
    }
}

void turnRight(){
  float Vleft = analogRead(A3);
  delay(1000);
  chassis.turnFor(-30,100, true);
  delay(500);
  do{
    leftMotor.setMotorEffort(30);
    rightMotor.setMotorEffort(-30);
    Vleft = analogRead(A3);
    }while(Vleft < 100);
}

void turnLeft(){
  float Vright = analogRead(A2);
  delay(1000);
  chassis.turnFor(30,100, true);
  delay(500);
  do{
    leftMotor.setMotorEffort(-30);
    rightMotor.setMotorEffort(30);
    Vright = analogRead(A2);
    }while(Vright < 100);
}



void loop() {
  //Serial.println(rangefinder.getDistance());
  
  turnLeft();
  while(checkIntersectionEvent() == false) {
    lineFollow(50);
  }
  chassis.driveFor(8, 20, true);
  delay(1000);
  turnRight();
  while(rangefinder.getDistance() >= 12.7) {
    Serial.println(rangefinder.getDistance());
    lineFollow(50);
  }
  chassis.idle();
  exit(0);


}
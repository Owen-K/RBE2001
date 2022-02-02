#include <Arduino.h>
#include <Chassis.h>
#include <Rangefinder.h>
#include "line.h"

Chassis chassis;
LeftMotor leftMotor;
RightMotor rightMotor;
Rangefinder rangefinder = Rangefinder(17, 12);

void setup() {
  chassis.init();
  chassis.setMotorPIDcoeffs(5, 0.5);
  Serial.begin(115200);
  rangefinder.init();

}

void turnRight(){ //Turn right until the robot is centered over a line
  float Vleft = analogRead(A3);
  delay(100);
  chassis.turnFor(-30,100, true);
  delay(100);
  do{
    leftMotor.setMotorEffort(50);
    rightMotor.setMotorEffort(-50);
    Vleft = analogRead(A3);
    }while(Vleft < 100);
}

void turnLeft(){ //Turn left until the robot is centered over a line
  float Vright = analogRead(A2);
  delay(100);
  chassis.turnFor(30,100, true);
  delay(100);
  do{
    leftMotor.setMotorEffort(-50);
    rightMotor.setMotorEffort(50);
    Vright = analogRead(A2);
    }while(Vright < 100);
}



void loop() {
  
  turnLeft();
  while(checkIntersectionEvent() == false) { //Follow the line until an intersection is seen
    lineFollow(70);
  }
  chassis.driveFor(8, 20, true);
  turnRight();
  while(rangefinder.getDistance() >= 12.7) { //Follow the line until 5 inches from the wall
    Serial.println(rangefinder.getDistance());
    lineFollow(70);
  }
  chassis.idle();
  exit(0);


}
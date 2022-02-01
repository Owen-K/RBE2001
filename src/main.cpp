#include <Arduino.h>
#include <Chassis.h>
#include "line.h"

Chassis chassis;
LeftMotor leftMotor;
RightMotor rightMotor;


void setup() {
  chassis.init();
  chassis.setMotorPIDcoeffs(5, 0.5);
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



void loop() {
  TwistOffLineAndLook();
  while(checkIntersectionEvent() == false) {
    lineFollow(50);
  }
  chassis.driveFor(5, 20);
  chassis.turnFor(90, 20);


}
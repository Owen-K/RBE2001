#include <Arduino.h>
#include <Chassis.h>
#include <Rangefinder.h>
#include <Romi32U4.h>
#include <servo32u4.h>
#include <IRdecoder.h>
#include <ir_codes.h>

#include "line.h"
#include "BlueMotor.h"

enum ROBOT_STATE {STARTUP, SETUP, TURN_LEFT, TURN_RIGHT, LINING, IDLE, PICKUP, DROPOFF, TRAVELING, ESTOP, END};

Chassis chassis;
LeftMotor leftMotor;
RightMotor rightMotor;
Rangefinder rangefinder = Rangefinder(17, 12);
BlueMotor motor;
Servo32U4 servo;
IRDecoder decoder(14);
ROBOT_STATE state = STARTUP;
int code;
bool locked = false;
int Road = 0;   
bool hasPlate = false;
bool bothSides = false;
const int gripperOpen = 600;
const int gripperClosed = 2440;
const int servoAnalogOpen = 290;
const int servoAnalogClosed = 455;
int ultraSonicStopCount = 0;
const int armPosition25Deg = 7340; 
const int armPosition45Deg = 2680;
const int armPositionStage = 494;
const int armDrivePosition = 7378;
ROBOT_STATE lastState;




void eStopCheck(){
  if (code == PLAY_PAUSE) {
    lastState = state;
    state = ESTOP;
  }
}


void turnRight(){ //Turn right until the robot is centered over a line
  float Vleft = analogRead(A3);
  //delay(100);
  if (!locked) {
    chassis.turnFor(-30,100, true);
    locked = true;
  }
  //delay(100);
  leftMotor.setMotorEffort(50);
  rightMotor.setMotorEffort(-50);
  if (Vleft > 200) {
    state = LINING;
    locked = false;
  }
}

void turnLeft(){ //Turn left until the robot is centered over a line
  float Vright = analogRead(A2);
  //delay(100);
  if (!locked) {
    chassis.turnFor(30,100, true);
    locked = true;
  }
  //delay(100);
  leftMotor.setMotorEffort(-50);
  rightMotor.setMotorEffort(50);
  if (Vright > 200) {
    state = LINING;
    locked = false;
  }
}


void whatShouldIDo(){
  if (hasPlate){
    chassis.idle();
    state = DROPOFF; //drop off
  } else {
    chassis.idle();
    state = PICKUP; //pickup 
  }
}


void whereAmI(){
  if (Road == 1){ 
    if (checkIntersectionEvent() == true){
      chassis.driveFor(8, 20, true);
      Road = 2;
      state = TURN_LEFT;                //turn left
    }
    if (rangefinder.getDistance() <= 12.7){
                                // at stage 
      chassis.idle();
      ultraSonicStopCount++;
      whatShouldIDo();          //pickup? dropoff?
    }

  } else if (Road == 2){ 
    if (checkIntersectionEvent() == true){
      chassis.driveFor(8, 20, true);
      Road = 1;
      state = TURN_RIGHT;                //turn right
    }
    if (rangefinder.getDistance() <= 12.7){
                                // at roof 25 degrees 
      chassis.idle();
      ultraSonicStopCount++;
      whatShouldIDo();          //pickup? dropoff?
    }

  } else if (Road == 3){ 
    if (checkIntersectionEvent() == true){
      chassis.driveFor(8, 20, true);
      Road = 4;
      state = TURN_RIGHT;                //turn right
    }
    if (rangefinder.getDistance() <= 12.7){
                                // at stage
      chassis.idle();
      ultraSonicStopCount++;
      whatShouldIDo();          //pickup? dropoff?
    }

  } else if (Road == 4){ 
    if (checkIntersectionEvent() == true){
      chassis.driveFor(8, 20, true);
      Road = 3;
      state = TURN_LEFT;                //turn left
    }
    if (rangefinder.getDistance() <= 12.7){
                                // at roof 45 degrees
      chassis.idle();
      ultraSonicStopCount++;
      whatShouldIDo();          //pickup? dropoff?
    }
  }
  
}


void howManyTrips(){ 
  if (ultraSonicStopCount == 2){
    state = PICKUP;            //needs to make a second trip starts picking up 

  } else if (ultraSonicStopCount == 4){
    if (bothSides){
      state = END;
    } else {
      state = TRAVELING;           //has made two trips cross path
    }
               
  } else {
    state = TURN_LEFT;            //turn around at roof
  }

}



void pickUp(int targetPosition){
  if (!locked) {
    locked = motor.moveTo(targetPosition);
  } else {
    chassis.driveFor(3, 20, true);
    servo.writeMicroseconds(gripperClosed);
    if (motor.moveTo(armDrivePosition)) {
      hasPlate = true;
      locked = false;
      state = TURN_LEFT;          //turns around
    }
  }

}



void dropOff(int targetPosition){
  if (!locked) {
    locked = motor.moveTo(targetPosition);
  } else {
    servo.writeMicroseconds(gripperOpen);
    if (motor.moveTo(armDrivePosition)) {
      hasPlate = false;
      locked = false;
      howManyTrips();          //turns around
    }
  }
  
}



void setup() {
  motor.setup();
  motor.reset();
  chassis.init();
  decoder.init();
  chassis.setMotorPIDcoeffs(5, 0.5);
  Serial.begin(115200);
  rangefinder.init();
  servo.setMinMaxMicroseconds(500, 2500);
  servo.writeMicroseconds(gripperClosed);

}


void loop() {
  code = decoder.getKeyCode(true);

  //State machine 
  /*

  variables: Roads 1-4, starting side, old state 
  has plate condition
  Estop condition

  New State machine
  case 0 starting state waits for 25/45 button press, stores 25 or 45
  case 1 turn left state 
  case 2 turn right state
  case 3 line follow 
      check road
      checks for intersection with line sensors 
      checks for roof with ultra
  case 4 motor idle 
  case 5 pickup 
    changes has plate
  case 6 dropoff 
    changes has plate
  case 7 change sides 
  case 8 E-Stop
  
  */


  switch (state){
    default:
      break;

    case STARTUP:            //start state
      eStopCheck();
      //motor.moveTo(armDrivePosition);
      switch (code){
        default:
          motor.setEffort(0);
          break;
        case UP_ARROW:
          motor.setEffort(400);
          break;
        case DOWN_ARROW:
          motor.setEffort(-400);
          break;
        case NUM_2:
          //25 degree roof
          motor.reset();
          Road = 1;
          state = SETUP;
          break;
        case NUM_4:
          //45 degree roof
          motor.reset();
          Road = 3;
          state = SETUP;
          break;
      }
      delay(150);
      break;
    
    case SETUP:
      if (motor.moveTo(armDrivePosition))
        state = TURN_LEFT;
      break;
    case TURN_LEFT:            // turn left 
      eStopCheck();
      turnLeft(); 
      break;           //new state is linefollow

    case TURN_RIGHT:            // turn right
      eStopCheck(); 
      turnRight(); 
      break;           //new state is linefollow
    
    case LINING:            // line follow
      eStopCheck();
      lineFollow(70);
      whereAmI();
      break;

    case IDLE:          //drive across field
      eStopCheck();
      chassis.setMotorEfforts(100, 100);
      if (checkIntersectionEvent() == true){
        chassis.driveFor(8, 20, true);
        if (Road == 1){
          turnLeft();
           Road = 3;
          ultraSonicStopCount = 0;
          state = TURN_RIGHT;
        } else if (Road == 3){
          turnRight();
          Road = 1;
          ultraSonicStopCount = 0;
          state = TURN_RIGHT;
        }
      }
      break;

    case PICKUP:            // pickup
      eStopCheck();
      if (Road == 2){
        dropOff(armPosition25Deg);
      } else if (Road == 4){
        dropOff(armPosition45Deg);
      } else {
        dropOff(armPositionStage);
      }
      break;

    case DROPOFF:            // dropoff
      eStopCheck();
      if (Road == 2){
        dropOff(armPosition25Deg);
      } else if (Road == 4){
        dropOff(armPosition45Deg);
      } else {
        dropOff(armPositionStage);
      }
      break;

    case TRAVELING:            // turn to switch sides
      eStopCheck();
      chassis.idle();
      if (Road == 1){
        chassis.turnFor(-90, 45, true);
      } else if (Road == 3){
        chassis.turnFor(90, 45, true);
      }
      bothSides = true;
      state = IDLE;
      break;

    case ESTOP:            // E-stop
      chassis.idle();
      motor.setEffort(0);
      if (code == ENTER_SAVE)
        state = lastState;
      break;

    case END:
      chassis.idle();
      motor.setEffort(0);
      exit(0);
      break;
  }

}




#include <Arduino.h>
#include <Chassis.h>
#include <Rangefinder.h>
#include <Romi32U4.h>
#include <servo32u4.h>
#include <IRdecoder.h>
#include <ir_codes.h>

#include "line.h"
#include "BlueMotor.h"

enum ROBOT_STATE {STARTUP, TURN_LEFT, TURN_RIGHT, LINING, IDLE, PICKUP, DROPOFF, TRAVELING, ESTOP, END};

Chassis chassis;
LeftMotor leftMotor;
RightMotor rightMotor;
Rangefinder rangefinder = Rangefinder(17, 12);
BlueMotor motor;
Servo32U4 servo;
ROBOT_STATE state = STARTUP;
int Road = 0;   
bool hasPlate = false;
bool bothSides = false;
const int gripperOpen = 600;
const int gripperClosed = 2440;
const int servoAnalogOpen = 290;
const int servoAnalogClosed = 455;
int ultraSonicStopCount = 0;
const int armPosition25Deg = 2000; 
const int armPosition45Deg = 2000;
const int armPositionStage = 1000;
const int armDrivePosition = 7375;
ROBOT_STATE lastState;


void setup() {
  motor.setup();
  motor.reset();
  chassis.init();
  chassis.setMotorPIDcoeffs(5, 0.5);
  Serial.begin(115200);
  rangefinder.init();
  servo.setMinMaxMicroseconds(500, 2500);
  servo.writeMicroseconds(gripperClosed);

}

void turnRight(){ //Turn right until the robot is centered over a line
  float Vleft = analogRead(A3);
  delay(100);
  chassis.turnFor(-30,100, true);
  delay(100);
  do{
    eStopCheck();
    leftMotor.setMotorEffort(50);
    rightMotor.setMotorEffort(-50);
    Vleft = analogRead(A3);
    }while(Vleft < 100);
    state = LINING;
  
}

void turnLeft(){ //Turn left until the robot is centered over a line
  float Vright = analogRead(A2);
  delay(100);
  chassis.turnFor(30,100, true);
  delay(100);
  do{
    eStopCheck();
    leftMotor.setMotorEffort(-50);
    rightMotor.setMotorEffort(50);
    Vright = analogRead(A2);
    }while(Vright < 100);
    state = LINING;

}


void whatShouldIDo(){
  if (hasPlate){
    state = DROPOFF; //drop off
  } else {
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



void pickUp(){ 
  //MOVE TOO IS BLOCKING CODE 
  if (Road == 2){
    motor.moveTo(armPosition25Deg);
  } else if (Road == 4){
    motor.moveTo(armPosition45Deg);
  } else {
    motor.moveTo(armPositionStage);
  }
  servo.writeMicroseconds(gripperClosed);
  motor.moveTo(armDrivePosition);
  hasPlate = true;
  state = TURN_LEFT;          //turns around
}



void dropOff(){
  if (Road == 2){
    motor.moveTo(armPosition25Deg);
  } else if (Road == 4){
    motor.moveTo(armPosition45Deg);
  } else {
    motor.moveTo(armPositionStage);
  }
  servo.writeMicroseconds(gripperOpen);
  motor.moveTo(armDrivePosition);
  hasPlate = false;
  howManyTrips();          //turns around

}



void eStopCheck(){
  //if button press equal estop button
  // or if otherpress = true
  //then oldState = State
  //state = ESTOP
}



void loop() {
  
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
      motor.moveTo(armDrivePosition);
      /*if button equal 1 
        Road = 1 (25 degree side)
      if button press equal 2 
        Road = 3 (45 degree side)
      */
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
      chassis.idle();
      pickUp(); //BLOCKING CODE
      break;

    case DROPOFF:            // dropoff
      eStopCheck();
      chassis.idle();
      dropOff();  //BLOCKING CODE
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
      // if resume button press then state = old state
      break;

    case END:
      chassis.idle();
      motor.setEffort(0);
      exit(0);
      break;
  }

}




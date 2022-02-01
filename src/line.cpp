#include <Chassis.h>

#include "line.h"


unsigned long lastTime = 0;

//interval at which line following actions are taken in ms.
const unsigned long LINE_FOLLOWING_INTERVAL = 2;

// ADC value required for a line to be detected.
const int DARKNESS_THRESHOLD = 250;

const float KP = 0.024; // gain


int rightReading;
int leftReading;

int prevLeft = 0;
int prevRight = 0;

/*
 * Moves forward at baseSpeed while keeping the Romi centered over a dark line
 */
void lineFollow(float baseSpeed)
{
    unsigned long currTime = millis();
    if(currTime - lastTime > LINE_FOLLOWING_INTERVAL)
    {
        //read sensors
       rightReading = analogRead(A2);
       leftReading = analogRead(A3);


        //calculate error (right reading - left reading)
        int error = rightReading - leftReading;
        
        //calculate effort (kp * error)
        float effort = KP * error;

        //command the motors (basespeed +- effort)
        chassis.leftMotor.setMotorEffort(baseSpeed  + effort);
        chassis.rightMotor.setMotorEffort(baseSpeed - effort);
        
        lastTime = currTime;
    }
}

bool checkIntersectionEvent(void)
{
    bool retVal = false;

    //if intersection is detected return TRUE
    if((rightReading >= DARKNESS_THRESHOLD && leftReading >= DARKNESS_THRESHOLD) 
        && (prevLeft < DARKNESS_THRESHOLD || prevRight < DARKNESS_THRESHOLD))
        {
            retVal = true;
        }
    
    prevLeft = leftReading;
    prevRight = rightReading;      
    return retVal;
}
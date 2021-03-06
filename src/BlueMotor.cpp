#include <Arduino.h>
#include <BlueMotor.h>
#include <Romi32U4.h>

int oldValue = 0;
int newValue;
int count = 0;
unsigned time = 0;

BlueMotor::BlueMotor()
{
}

void BlueMotor::setup()
{
    pinMode(PWMOutPin, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
    TCCR1A = 0xA8; //0b10101000; //gcl: added OCR1C for adding a third PWM on pin 11
    TCCR1B = 0x11; //0b00010001;
    ICR1 = 400;
    OCR1C = 0;

    attachInterrupt(digitalPinToInterrupt(ENCA), isrA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB), isrB, CHANGE);
    reset();
}

long BlueMotor::getPosition()
{
    return count;
}

void BlueMotor::reset()
{
    count = 0;
}


void BlueMotor::isrA()
{
    if (digitalRead(ENCA) == digitalRead(ENCB)) count++;
     else count--;
}


void BlueMotor::isrB()
{
   if (digitalRead(ENCA) == digitalRead(ENCB)) count--;
    else count++;
}

void BlueMotor::setEffort(int effort)
{
    if (effort < 0)
    {
        setEffort(-effort, true);
    }
    else
    {
        setEffort(effort, false);
    }
}

void BlueMotor::setEffort(int effort, bool clockwise)
{
    if (clockwise)
    {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    }
    else
    {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }
    OCR1C = constrain(effort, 0, 400);
}

void BlueMotor::setEffortWithoutDB(long effort)
{
    int adjustedEffort; 
    if (effort >= 0)
    {
        adjustedEffort = effort + 260;
        setEffort(adjustedEffort);
    }
    if (effort <= 0)
    {
        adjustedEffort = effort - 260;
        setEffort(adjustedEffort);
    } 
}

bool BlueMotor::moveTo(long target)  //Move to this encoder position within the specified
{                                    //tolerance in the header file using proportional control
                                     //then stop
    // Otherpress = false 
    if (!(getPosition() < (target + tolerance) && getPosition() > (target - tolerance))) {
        setEffortWithoutDB((target - getPosition()) * KP);
    } else {
        setEffort(0);
        return true;
    }  
    return false;  
}



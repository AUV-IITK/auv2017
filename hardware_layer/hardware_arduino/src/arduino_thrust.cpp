#include "Arduino.h"
#include "arduino_thrust.h"
#include <iostream.h>

using namespace std;

arduino_thrust::arduino_thrust();

void arduino_thrust::setPins(int A1, int D1, int D2)
{
    An1_=A1;
    D1_=D1;
    D2_=D2;
    pinMode(An1_,OUTPUT);
    pinMode(D1_,OUTPUT);
    pinMode(D2_,OUTPUT);
}

void arduino_thrust::ON(int pwm,bool Dir1,bool Dir2)
{
    analogWrite(An1_,pwm);
    digitalWrite(D1_,Dir1);
    digitalWrite(D2_,Dir2);
}

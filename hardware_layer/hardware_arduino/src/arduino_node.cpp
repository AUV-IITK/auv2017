#include <ros.h>
#include <Arduino.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <Wire.h>
#include "MS5837.h"

#define pwmPinWest 3
#define pwmPinEast 2
#define directionPinEast1 30
#define directionPinEast2 31
#define directionPinWest1 32
#define directionPinWest2 33

#define pwmPinNorthSway 5
#define pwmPinSouthSway 4
#define directionPinSouthSway1 27
#define directionPinSouthSway2 26
#define directionPinNorthSway1 29
#define directionPinNorthSway2 287

#define pwmPinNorthUp 6
#define pwmPinSouthUp 7
#define directionPinNorthUp1 24
#define directionPinNorthUp2 25
#define directionPinSouthUp1 22
#define directionPinSouthUp2 23

#define analogPinPressureSensor A0

MS5837 sensor;

float last_pressure_sensor_value, pressure_sensor_value;
std_msgs::Float64 voltage;
ros::NodeHandle nh;

void TEastCb(const std_msgs::Int32 msg)
{
    int pwm=msg.data;
    bool isForward=true;
    if(pwm<=0)
    {
        pwm=abs(pwm);
        isForward=false;
    }
    analogWrite(pwmPinEast, 255 - pwm);
    if(isForward)
    {
        digitalWrite(directionPinEast1, HIGH);
        digitalWrite(directionPinEast2, LOW);
    }
    else
    {
        digitalWrite(directionPinEast1, LOW);
        digitalWrite(directionPinEast2, HIGH);
    }
}

void TWestCb(const std_msgs::Int32 msg)
{
    int pwm=msg.data;
    bool isForward=true;
    if(pwm<=0)
    {
        pwm=abs(pwm);
        isForward=false;
    }
    analogWrite(pwmPinWest, 255 - pwm);
    if(isForward)
    {
        digitalWrite(directionPinWest1, HIGH);
        digitalWrite(directionPinWest2, LOW);
    }
    else
    {
        digitalWrite(directionPinWest1, LOW);
        digitalWrite(directionPinWest2, HIGH);
    }
}

void TNorthSwayCb(const std_msgs::Int32 msg)
{
    int pwm=msg.data;
    bool isSideward=true;
    if(pwm<=0)
    {
        pwm=abs(pwm);
        isSideward=false;
    }
    analogWrite(pwmPinNorthSway, 255 - pwm);
    if(isSideward)
    {
        digitalWrite(directionPinNorthSway1, HIGH);
        digitalWrite(directionPinNorthSway2, LOW);
    }
    else
    {
        digitalWrite(directionPinNorthSway1, LOW);
        digitalWrite(directionPinNorthSway2, HIGH);
    }
}

void TSouthSwayCb(const std_msgs::Int32 msg)
{
    int pwm=msg.data;
    bool isSideward=true;
    if(pwm<=0)
    {
        pwm=abs(pwm);
        isSideward=false;
    }
    analogWrite(pwmPinSouthSway, 255 - pwm);
    if(isSideward)
    {
        digitalWrite(directionPinSouthSway1, HIGH);
        digitalWrite(directionPinSouthSway2, LOW);
    }
    else
    {
        digitalWrite(directionPinSouthSway1, LOW);
        digitalWrite(directionPinSouthSway2, HIGH);
    }
}

void TNorthUpCb(const std_msgs::Int32 msg)
{
    int pwm=msg.data;
    bool isUpward=true;
    if(pwm<=0)
    {
        pwm=abs(pwm);
        isUpward=false;
    }
    analogWrite(pwmPinNorthUp, 255 - pwm);
    if(isUpward)
    {
        digitalWrite(directionPinNorthUp1, HIGH);
        digitalWrite(directionPinNorthUp2, LOW);
    }
    else
    {
        digitalWrite(directionPinNorthUp1, LOW);
        digitalWrite(directionPinNorthUp2, HIGH);
    }
}

void TSouthUpCb(const std_msgs::Int32 msg)
{
    int pwm=msg.data;
    bool isUpward=true;
    if(pwm<=0)
    {
        pwm=abs(pwm);
        isUpward=false;
    }
    analogWrite(pwmPinSouthUp, 255 - pwm);
    if(isUpward)
    {
        digitalWrite(directionPinSouthUp1, HIGH);
        digitalWrite(directionPinSouthUp2, LOW);
    }
    else
    {
        digitalWrite(directionPinSouthUp1, LOW);
        digitalWrite(directionPinSouthUp2, HIGH);
    }
}

ros::Subscriber<std_msgs::Int32> subPwmEast("/ard/east", &TEastCb);
ros::Subscriber<std_msgs::Int32> subPwmWest("/ard/west", &TWestCb);
ros::Subscriber<std_msgs::Int32> subPwmNorthSway("/ard/northsway", &TNorthSwayCb);
ros::Subscriber<std_msgs::Int32> subPwmSouthSway("/ard/southsway", &TSouthSwayCb);
ros::Subscriber<std_msgs::Int32> subPwmNorthUp("/ard/northup", &TNorthUpCb);
ros::Subscriber<std_msgs::Int32> subPwmSouthUp("/ard/southup", &TSouthUpCb);
ros::Publisher ps_voltage("/varun/sensors/pressure_sensor/depth", &voltage);


void setup()
{
    nh.initNode();
    Wire.begin();
    
    sensor.init();
    
    sensor.setFluidDensity(997);    //kg/m^3 (freshwater, 1029 for seawater)
    pinMode(pwmPinEast, OUTPUT);
    pinMode(directionPinEast1, OUTPUT);
    pinMode(directionPinEast2, OUTPUT);
    pinMode(pwmPinWest, OUTPUT);
    pinMode(directionPinWest1, OUTPUT);
    pinMode(directionPinWest2, OUTPUT);
    
    pinMode(directionPinSouthSway1, OUTPUT);
    pinMode(directionPinSouthSway2, OUTPUT);
    pinMode(pwmPinNorthSway, OUTPUT);
    pinMode(directionPinNorthSway2, OUTPUT);
    pinMode(pwmPinSouthSway, OUTPUT);
    pinMode(directionPinNorthSway1, OUTPUT);
    
    pinMode(directionPinSouthUp1, OUTPUT);
    pinMode(directionPinSouthUp2, OUTPUT);
    pinMode(pwmPinNorthUp, OUTPUT);
    pinMode(directionPinNorthUp2, OUTPUT);
    pinMode(pwmPinSouthUp, OUTPUT);
    pinMode(directionPinNorthUp1, OUTPUT);
    
    nh.subscribe(subPwmEast);
    nh.subscribe(subPwmWest);
    nh.subscribe(subPwmNorthSway);
    nh.subscribe(subPwmSouthSway);
    nh.subscribe(subPwmNorthUp);
    nh.subscribe(subPwmSouthUp);
    nh.advertise(ps_voltage);
    Serial.begin(57600);
    std_msgs::Int32 v;
    v.data=0;
    TEastCb(v);
    TWestCb(v);
    TNorthSwayCb(v);
    TSouthSwayCb(v);
    TNorthUpCb(v);
    TSouthUpCb(v);
    
    sensor.read();
    last_pressure_sensor_value = -(sensor.depth() * 100);
}

void loop()
{
    sensor.read();
    // voltage.data made -ve because pressure sensor data should increase going up
    pressure_sensor_value = -(sensor.depth() * 100);
    // to avoid random high values
    if (abs(last_pressure_sensor_value - pressure_sensor_value) < 100)
    {
        voltage.data = 0.7 * pressure_sensor_value + 0.3 * last_pressure_sensor_value;
        ps_voltage.publish(&voltage);
        last_pressure_sensor_value = pressure_sensor_value;
    }
    delay(200);
    nh.spinOnce();
}

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

//The below variables are named in such fashion: <pwm-value-range><Direction-in-which-thruster-will-move><the-intended-thruster> 
//for ex: 'highForwardEast' means "the east thruster of the bot will move forward with a high pwm value"    


#define highForwardEast 
#define medForwardEast
#define lowForwardEast

#define highForwardWest
#define medForwardWest
#define lowForwardWest

#define highBackwardEast
#define medBackwardEast
#define lowBackwardEast

#define highBackwardWest
#define medBackwardWest
#define lowBackwardWest

#define highUpNorth
#define medUpNorth
#define lowUpNorth

#define highUpSouth
#define medUpSouth
#define lowUpSouth

#define highDownNorth
#define medDownNorth
#define lowDownNorth

#define highDownSouth
#define medDownSouth
#define lowDownSouth

#define highLeftNorth
#define medLeftNorth
#define lowLeftNorth

#define highLeftSouth
#define medLeftSouth
#define lowLeftSouth

#define highRightNorth
#define medRightNorth
#define lowRightNorth

#define highRightSouth
#define medRightSouth
#define lowRightSouth

#define highAnticlockNorth
#define medAnticlockNorth
#define lowAnticlockNorth

#define highAnticlockSouth
#define medAnticlockSouth
#define lowAnticlockSouth

#define highClockNorth
#define medClockNorth
#define lowClockNorth

#define highClockSouth
#define medClockSouth
#define lowClockSouth

#define highAnticlockEast
#define medAnticlockEast
#define lowAnticlockEast

#define highAnticlockWest
#define medAnticlockWest
#define lowAnticlockWest

#define highClockEast
#define medClockEast
#define lowClockEast

#define highClockWest
#define medClockWest
#define lowClockWest

MS5837 sensor;
int SPEED_MODE;
int HIGH_SPEED = 3;
int MED_SPEED = 2;
int LOW_SPEED = 1;

bool IS_FORWARD = false;
bool IS_BACKWARD = false;
bool IS_UPWARD = false;
bool IS_DOWNWARD = false;
bool IS_LEFT = false;
bool IS_RIGHT = false;
bool IS_CLOCKWISE = false;
bool IS_ANTICLOCKWISE = false;

float last_pressure_sensor_value, pressure_sensor_value;

std_msgs::Float64 voltage;
ros::NodeHandle nh;

void mode(int pwm_ard)
{
  if (pwm_ard < 50)
    {
      return HIGH_SPEED;
    }
    else if (pwm_ard>50 && pwm_ard<100)
    {
      return MED_SPEED;
    }
    else
    {
      return LOW_SPEED;
    }
}

void thrusterEast(int mode, bool IS_FORWARD, bool IS_BACKWARD, bool IS_ANTICLOCKWISE, bool IS_CLOCKWISE)
{
  if (IS_FORWARD)
  {
    switch(mode)
    {
      case HIGH_SPEED:
          analogWrite(pwmPinEast, highForwardEast);
      case MED_SPEED:
          analogWrite(pwmPinEast, medForwardEast);
      case LOW_SPEED:
          analogWrite(pwmPinEast, lowForwardEast);
    }

    digitalWrite(directionPinEast1, HIGH);
        digitalWrite(directionPinEast2, LOW);
  }
  

  if (IS_BACKWARD)
  {
    switch(mode)
    {
      case HIGH_SPEED:
          analogWrite(pwmPinEast, highBackwardEast);
      case MED_SPEED:
          analogWrite(pwmPinEast, medBackwardEast);
      case LOW_SPEED:
          analogWrite(pwmPinEast, lowBackwardEast);
    }

    digitalWrite(directionPinEast1, LOW);
      digitalWrite(directionPinEast2, HIGH);
    }

    if (IS_ANTICLOCKWISE)
    {
       switch(mode)
    {
      case HIGH_SPEED:
          analogWrite(pwmPinEast, highAnticlockEast);
      case MED_SPEED:
          analogWrite(pwmPinEast, medAnticlockEast);
      case LOW_SPEED:
          analogWrite(pwmPinEast, lowAnticlockEast);
    }

    digitalWrite(directionPinEast1, HIGH);
      digitalWrite(directionPinEast2, LOW);
    }

    if (IS_CLOCKWISE)
    {
      switch(mode)
    {
      case HIGH_SPEED:
          analogWrite(pwmPinEast, highClockEast);
      case MED_SPEED:
          analogWrite(pwmPinEast, medClockEast);
      case LOW_SPEED:
          analogWrite(pwmPinEast, lowClockEast);
    }

    digitalWrite(directionPinEast1, LOW);
      digitalWrite(directionPinEast2, HIGH);
    }

}

void thrusterWest(int mode, bool IS_FORWARD, bool IS_BACKWARD, bool IS_ANTICLOCKWISE, bool IS_CLOCKWISE)
{
  if (IS_FORWARD)
  {
    switch(mode)
    {
      case HIGH_SPEED:
          analogWrite(pwmPinWest, highForwardWest);
      case MED_SPEED:
          analogWrite(pwmPinWest, medForwardWest);
      case LOW_SPEED:
          analogWrite(pwmPinWest, lowForwardWest);
    }

    digitalWrite(directionPinWest1, HIGH);
        digitalWrite(directionPinWest2, LOW);
  }
  

  if (IS_BACKWARD)
  {
    switch(mode)
    {
      case HIGH_SPEED:
          analogWrite(pwmPinWest, highBackwardWest);
      case MED_SPEED:
          analogWrite(pwmPinWest, medBackwardWest);
      case LOW_SPEED:
          analogWrite(pwmPinWest, lowBackwardWest);
    }
  
    digitalWrite(directionPinWest1, LOW);
      digitalWrite(directionPinWest2, HIGH);
    }

    if (IS_ANTICLOCKWISE)
    {
       switch(mode)
    {
      case HIGH_SPEED:
          analogWrite(pwmPinWest, highAnticlockWest);
      case MED_SPEED:
          analogWrite(pwmPinWest, medAnticlockWest);
      case LOW_SPEED:
          analogWrite(pwmPinWest, lowAnticlockWest);
    }

    digitalWrite(directionPinWest1, LOW);
      digitalWrite(directionPinWest2, HIGH);
    }

    if (IS_CLOCKWISE)
    {
      switch(mode)
    {
      case HIGH_SPEED:
          analogWrite(pwmPinWest, highClockWest);
      case MED_SPEED:
          analogWrite(pwmPinWest, medClockWest);
      case LOW_SPEED:
          analogWrite(pwmPinWest, lowClockWest);
    }

    digitalWrite(directionPinWest1, HIGH);
      digitalWrite(directionPinWest2, LOW);
    }
}


void thrusterNorthUp(int mode, bool IS_UPWARD, bool IS_DOWNWARD)
{
  if (IS_UPWARD)
  {
    switch(mode)
    {
      case HIGH_SPEED:
          analogWrite(pwmPinNorthUp, highUpNorth);
      case MED_SPEED:
          analogWrite(pwmPinNorthUp, medUpNorth);
      case LOW_SPEED:
          analogWrite(pwmPinNorthUp, lowUpNorth);
    }

    digitalWrite(directionPinNorthUp1, HIGH);
        digitalWrite(directionPinNorthUp2, LOW);
  }
  

  if (IS_DOWNWARD)
  {
    switch(mode)
    {
      case HIGH_SPEED:
          analogWrite(pwmPinNorthUp, highDownNorth);
      case MED_SPEED:
          analogWrite(pwmPinNorthUp, medDownNorth);
      case LOW_SPEED:
          analogWrite(pwmPinNorthUp, lowDownNorth);
    }

    digitalWrite(directionPinNorthUp1, LOW);
      digitalWrite(directionPinNorthUp2, HIGH);
    } 
}

void thrusterSouthUp(int mode, bool IS_UPWARD, bool IS_DOWNWARD)
{
  if (IS_UPWARD)
  {
    switch(mode)
    {
      case HIGH_SPEED:
          analogWrite(pwmPinSouthUp, highUpSouth);
      case MED_SPEED:
          analogWrite(pwmPinSouthUp, medUpSouth);
      case LOW_SPEED:
          analogWrite(pwmPinSouthUp, lowUpSouth);
    }

    digitalWrite(directionPinSouthUp1, HIGH);
        digitalWrite(directionPinSouthUp2, LOW);
  }
  

  if (IS_DOWNWARD)
  {
    switch(mode)
    {
      case HIGH_SPEED:
          analogWrite(pwmPinSouthUp, highDownSouth);
      case MED_SPEED:
          analogWrite(pwmPinSouthUp, medDownSouth);
      case LOW_SPEED:
          analogWrite(pwmPinSouthUp, lowDownSouth);
    }

    digitalWrite(directionPinSouthUp1, LOW);
        digitalWrite(directionPinSouthUp2, HIGH);
  } 
}

void thrusterNorthSway(int mode, bool IS_LEFT, bool IS_RIGHT, bool IS_ANTICLOCKWISE, bool IS_CLOCKWISE)
{
  if (IS_LEFT)
  {
    switch(mode)
    {
      case HIGH_SPEED:
          analogWrite(pwmPinNorthSway, highLeftNorth);
      case MED_SPEED:
          analogWrite(pwmPinNorthSway, medLeftNorth);
      case LOW_SPEED:
          analogWrite(pwmPinNorthSway, lowLeftNorth);
    }

    digitalWrite(directionPinNorthSway1, HIGH);
        digitalWrite(directionPinNorthSway2, LOW);
  }
  

  if (IS_RIGHT)
  {
    switch(mode)
    {
      case HIGH_SPEED:
          analogWrite(pwmPinNorthSway, highRightNorth);
      case MED_SPEED:
          analogWrite(pwmPinNorthSway, medRightNorth);
      case LOW_SPEED:
          analogWrite(pwmPinNorthSway, lowRightNorth);
    }

    digitalWrite(directionPinNorthSway1, LOW);
        digitalWrite(directionPinNorthSway2, HIGH);
  } 

  if (IS_ANTICLOCKWISE)
    {
       switch(mode)
    {
      case HIGH_SPEED:
          analogWrite(pwmPinNorthSway, highAnticlockNorth);
      case MED_SPEED:
          analogWrite(pwmPinNorthSway, medAnticlockNorth);
      case LOW_SPEED:
          analogWrite(pwmPinNorthSway, lowAnticlockNorth);
    }

    digitalWrite(directionPinNorthSway1, HIGH);
      digitalWrite(directionPinNorthSway2, LOW);
    }

    if (IS_CLOCKWISE)
    {
       switch(mode)
    {
      case HIGH_SPEED:
          analogWrite(pwmPinNorthSway, highClockNorth);
      case MED_SPEED:
          analogWrite(pwmPinNorthSway, medClockNorth);
      case LOW_SPEED:
          analogWrite(pwmPinNorthSway, lowClockNorth);
    }

    digitalWrite(directionPinNorthSway1, LOW);
      digitalWrite(directionPinNorthSway2, HIGH);
    }
}

void thrusterSouthSway(int mode, bool IS_LEFT, bool IS_RIGHT, bool IS_ANTICLOCKWISE, bool IS_CLOCKWISE)
{
  if (IS_LEFT)
  {
    switch(mode)
    {
      case HIGH_SPEED:
          analogWrite(pwmPinSouthSway, highLeftSouth);
      case MED_SPEED:
          analogWrite(pwmPinSouthSway, medLeftSouth);
      case LOW_SPEED:
          analogWrite(pwmPinSouthSway, lowLeftSouth);
    }

    digitalWrite(directionPinSouthSway1, HIGH);
        digitalWrite(directionPinSouthSway2, LOW);
  }
  

  if (IS_RIGHT)
  { 
    switch(mode)
    {
      case HIGH_SPEED:
          analogWrite(pwmPinSouthSway, highRightSouth);
      case MED_SPEED:
          analogWrite(pwmPinSouthSway, medRightSouth);
      case LOW_SPEED:
          analogWrite(pwmPinSouthSway, lowRightSouth);
    }

    digitalWrite(directionPinSouthSway1, LOW);
        digitalWrite(directionPinSouthSway2, HIGH);
  
  }

  if (IS_ANTICLOCKWISE)
    {
       switch(mode)
    {
      case HIGH_SPEED:
          analogWrite(pwmPinSouthSway, highAnticlockSouth);
      case MED_SPEED:
          analogWrite(pwmPinSouthSway, medAnticlockSouth);
      case LOW_SPEED:
          analogWrite(pwmPinSouthSway, lowAnticlockSouth);
    }

    digitalWrite(directionPinSouthSway1, LOW);
      digitalWrite(directionPinSouthSway2, HIGH);
    } 

    if (IS_CLOCKWISE)
    {
       switch(mode)
    {
      case HIGH_SPEED:
          analogWrite(pwmPinSouthSway, highClockSouth);
      case MED_SPEED:
          analogWrite(pwmPinSouthSway, medClockSouth);
      case LOW_SPEED:
          analogWrite(pwmPinSouthSway, lowClockSouth);
    }

    digitalWrite(directionPinSouthSway1, HIGH);
      digitalWrite(directionPinSouthSway2, LOW);
    }
}

void PWMCbForward(const std_msgs::Int32& msg)
{
    int pwm = msg.data;
    if(pwm>0)
    {
      IS_FORWARD = true;
    }
    else
    {
      IS_BACKWARD = true;
    }
    pwm=abs(pwm);
    pwm_ard = 255-pwm;

    SPEED_MODE = mode(pwm_ard);

  thrusterEast(SPEED_MODE, IS_FORWARD, IS_BACKWARD, IS_ANTICLOCKWISE, IS_CLOCKWISE)
  thrusterWest(SPEED_MODE, IS_FORWARD, IS_BACKWARD, IS_ANTICLOCKWISE, IS_CLOCKWISE)
}

void PWMCbUpward(const std_msgs::Int32& msg)
{
    int pwm = msg.data;
    if(pwm>0)
    {
      IS_UPWARD = true;
    }
    else
    {
      IS_DOWNWARD = true;
    }
    pwm=abs(pwm);
    pwm_ard = 255-pwm;

    SPEED_MODE = mode(pwm_ard);

  thrusterNorthUp(SPEED_MODE, IS_UPWARD, IS_DOWNWARD)
  thrusterSouthUp(SPEED_MODE, IS_UPWARD, IS_DOWNWARD)
  IS_UPWARD = false;A
}

void PWMCbSideward(const std_msgs::Int32& msg)
{
    int pwm = msg.data;
    if(pwm>0)
    {
      IS_LEFT = true;
    }
    else
    {
      IS_RIGHT = true;
    }
    pwm=abs(pwm);
    pwm_ard = 255-pwm;
    
    SPEED_MODE = mode(pwm_ard);

  thrusterNorthSway(SPEED_MODE, IS_LEFT, IS_RIGHT, IS_ANTICLOCKWISE, IS_CLOCKWISE)
  thrusterSouthSway(SPEED_MODE, IS_LEFT, IS_RIGHT, IS_ANTICLOCKWISE, IS_CLOCKWISE)
}

void PWMCbTurnForward(const std_msgs::Int32& msg)
{
    int pwm = msg.data;
    if(pwm>0)
    {
      IS_ANTICLOCKWISE = true;
    }
    else
    {
      IS_CLOCKWISE = true;
    }
    pwm=abs(pwm);
    pwm_ard = 255-pwm;

    SPEED_MODE = mode(pwm_ard);

  thrusterEast(SPEED_MODE, IS_FORWARD, IS_BACKWARD, IS_ANTICLOCKWISE, IS_CLOCKWISE);
  thrusterWest(SPEED_MODE, IS_FORWARD, IS_BACKWARD, IS_ANTICLOCKWISE, IS_CLOCKWISE);
}

void PWMCbTurnSway(const std_msgs::Int32& msg)
{
    int pwm = msg.data;
    if(pwm>0)
    {
      IS_ANTICLOCKWISE = true;
    }
    else
    {
      IS_CLOCKWISE = true;
    }
    pwm=abs(pwm);
    pwm_ard = 255-pwm;

    SPEED_MODE = mode(pwm_ard);

  thrusterNorthSway(SPEED_MODE, IS_LEFT, IS_RIGHT, IS_ANTICLOCKWISE, IS_CLOCKWISE);
  thrusterSouthSway(SPEED_MODE, IS_LEFT, IS_RIGHT, IS_ANTICLOCKWISE, IS_CLOCKWISE);
}

ros::Subscriber<std_msgs::Int32> subPwmForward("/pwm/forward", &PWMCbForward);
ros::Subscriber<std_msgs::Int32> subPwmSideward("/pwm/sideward", &PWMCbSideward);
ros::Subscriber<std_msgs::Int32> subPwmUpward("/pwm/upward", &PWMCbUpward);
ros::Subscriber<std_msgs::Int32> subPwmTurnForward("/pwm/turn/forward", &PWMCbTurnForward);
ros::Subscriber<std_msgs::Int32> subPwmTurnSway("/pwm/turn/sway", &PWMCbTurnSway);
ros::Publisher ps_voltage("/varun/sensors/pressure_sensor/depth", &voltage);

void setup()
{
  nh.initNode();
  Wire.begin();

  sensor.init();

  sensor.setFluidDensity(997);  // kg/m^3 (freshwater, 1029 for seawater)
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

  nh.subscribe(subPwmForward);
  nh.subscribe(subPwmSideward);
  nh.subscribe(subPwmUpward);
  nh.subscribe(subPwmTurnForward);
  nh.subscribe(subPwmTurnSway);
  nh.advertise(ps_voltage);
  Serial.begin(57600);
  std_msgs::Int32 msg;
  msg.data = 0;
  PWMCbForward(msg);
  PWMCbSideward(msg);
  PWMCbUpward(msg);
  PWMCbTurnForward(msg);
  PWMCbTurnSway(msg);

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

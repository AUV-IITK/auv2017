// Copyright 2016 AUV-IITK
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
#define directionPinNorthSway2 28

#define pwmPinNorthUp 6
#define pwmPinSouthUp 7
#define directionPinNorthUp1 24
#define directionPinNorthUp2 25
#define directionPinSouthUp1 22
#define directionPinSouthUp2 23

#define analogPinPressureSensor A0

const float c092 = 506.22;
const float s092 = -2.65;
const float c093 = 448.62;
const float s093 = -2.92;
const float c099 = 397.65;  // reference as their graph is at lowest
const float s099 = -2.71;   // reference as their graph is at lowest
const float c113 = 539.85;
const float s113 = -3.38;
const float c117 = 441.32;
const float s117 = -3.03;
const float c122 = 547.39;
const float s122 = -2.93;
const int neutral_buoyancy_offset = 0;

MS5837 sensor;

bool isMovingForward = true;
int minUpwardPWM = 80;
int biasSouthUp = -35;
int minSidewardPWM = 147;
int minForwardPWM = 147;
int minTurnForwardThrustersPWM = 60;
int minTurnSidewardThrustersPWM = 80;
float last_pressure_sensor_value, pressure_sensor_value;
std_msgs::Float64 voltage;

// Delete me
std_msgs::Int32 echo;

ros::NodeHandle nh;

int NormalizeUpwardPWM(int pwm)
{
  return pwm * 73 / 255 + minUpwardPWM;
}

int NormalizeSidewardPWM(int pwm)
{
  return pwm * 53 / 255 + minSidewardPWM;
}

int NormalizeForwardPWM(int pwm)
{
  return pwm * 53 / 255 + minForwardPWM;
}

int NormalizeTurnForwardThrustersPWM(int pwm)
{
  return pwm * 53 / 255 + minTurnForwardThrustersPWM;
}

int NormalizeTurnSidewardThrustersPWM(int pwm)
{
  return pwm * 53 / 255 + minTurnSidewardThrustersPWM;
}

int btd092(int pwm)
{
  pwm = (c099 + s099 * pwm - c092) / (s092);
  return pwm;
}

int btd093(int pwm)
{
  pwm = (c099 + s099 * pwm - c093) / (s093);
  return pwm;
}

int btd099(int pwm)
{
  return pwm;
}

int btd113(int pwm)
{
  pwm = (c099 + s099 * pwm - c113) / (s113);
  return pwm;
}

int btd117(int pwm)
{
  pwm = (c099 + s099 * pwm - c117) / (s117);
  return pwm;
}

int btd122(int pwm)
{
  pwm = (c099 + s099 * pwm - c122) / (s122);
  return pwm;
}

void thrusterNorthUp(int pwm, int isUpward)
{
  pwm = abs(pwm);
  pwm = NormalizeUpwardPWM(pwm);

  if (pwm > minUpwardPWM)
    pwm = btd117(pwm);
  else
    pwm = 0;

  analogWrite(pwmPinNorthUp, 255 - pwm);
  if (isUpward)
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

void thrusterSouthUp(int pwm, int isUpward)
{
  pwm = abs(pwm);
  pwm = NormalizeUpwardPWM(pwm);

  if (pwm > minUpwardPWM)
    pwm = btd093(pwm) + biasSouthUp;
  else
    pwm = 0;

  analogWrite(pwmPinSouthUp, 255 - pwm);
  if (isUpward)
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

void thrusterNorthSway(int pwm, int isRight, int isTurn)
{
  pwm = abs(pwm);
  if (isTurn)
  {
    pwm = NormalizeTurnSidewardThrustersPWM(pwm);
    if (pwm > minTurnSidewardThrustersPWM)
      pwm = btd113(pwm);  // possible location for turn with sideward bias.
    else
      pwm = 0;
  }
  else
  {
    pwm = NormalizeSidewardPWM(pwm);
    if (pwm > minSidewardPWM)
    {
      pwm = btd113(pwm);  // possible location for sideward bias.
    }
    else
    {
      pwm = 0;
    }
  }
  analogWrite(pwmPinNorthSway, 255 - pwm);
  if (isRight)
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

void thrusterSouthSway(int pwm, int isRight, int isTurn)
{
  pwm = abs(pwm);
  if (isTurn)
  {
    pwm = NormalizeTurnSidewardThrustersPWM(pwm);
    if (pwm > minTurnSidewardThrustersPWM)
    {
      pwm = btd122(pwm);  // possible location for turn with sideward bias.
    }
    else
    {
      pwm = 0;
    }
  }
  else
  {
    pwm = NormalizeSidewardPWM(pwm);
    if (pwm > minSidewardPWM)
    {
      pwm = btd122(pwm);  // possible location for sideward bias.
    }
    else
    {
      pwm = 0;
    }
  }
  analogWrite(pwmPinSouthSway, 255 - pwm);
  if (isRight)
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

void thrusterEast(int pwm, int isForward, int isTurn)
{
  pwm = abs(pwm);
  if (isTurn)
  {
    pwm = NormalizeTurnForwardThrustersPWM(pwm);
    if (pwm > minTurnForwardThrustersPWM)
    {
      pwm = btd092(pwm);  // possible location for turn with forward bias.
    }
    else
    {
      pwm = 0;
    }
  }
  else
  {
    pwm = NormalizeForwardPWM(pwm);
    if (pwm > minForwardPWM)
    {
      pwm = btd092(pwm);  // possible location for forward bias.
    }
    else
    {
      pwm = 0;
    }
  }
  analogWrite(pwmPinEast, 255 - pwm);
  if (isForward)
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

void thrusterWest(int pwm, int isForward, int isTurn)
{
  pwm = abs(pwm);
  if (isTurn)
  {
    pwm = NormalizeTurnForwardThrustersPWM(pwm);
    if (pwm > minTurnForwardThrustersPWM)
    {
      pwm = btd099(pwm);  // possible location for turn with forward bias.
    }
    else
    {
      pwm = 0;
    }
  }
  else
  {
    pwm = NormalizeForwardPWM(pwm);
    if (pwm > minForwardPWM)
    {
      pwm = btd099(pwm);  // possible location for forward bias.
    }
    else
    {
      pwm = 0;
    }
  }
  analogWrite(pwmPinWest, 255 - pwm);
  if (isForward)
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

void PWMCbForward(const std_msgs::Int32& msg)
{
  if (msg.data > 0)
  {
    thrusterEast(msg.data, true, false);
    thrusterWest(msg.data, true, false);
  }
  else
  {
    thrusterEast(msg.data, false, false);
    thrusterWest(msg.data, false, false);
  }
  if (msg.data != 0)
    isMovingForward = true;
}

void PWMCbSideward(const std_msgs::Int32& msg)
{
  if (msg.data > 0)
  {
    thrusterNorthSway(msg.data, true, false);
    thrusterSouthSway(msg.data, true, false);
  }
  else
  {
    thrusterNorthSway(msg.data, false, false);
    thrusterSouthSway(msg.data, false, false);
  }
  if (msg.data != 0)
    isMovingForward = false;
}

void PWMCbUpward(const std_msgs::Int32& msg)
{
  int pwm = msg.data;
  pwm = pwm + neutral_buoyancy_offset;
  if (pwm > 0)
  {
    thrusterNorthUp(pwm, true);
    thrusterSouthUp(pwm, true);
  }
  else
  {
    thrusterNorthUp(pwm, false);
    thrusterSouthUp(pwm, false);
  }
}

void PWMCbTurn(const std_msgs::Int32& msg)
{
  if (!isMovingForward)
  {
    if (msg.data > 0)
    {
      thrusterEast(msg.data, true, true);
      thrusterWest(msg.data, false, true);
    }
    else
    {
      thrusterEast(msg.data, false, true);
      thrusterWest(msg.data, true, true);
    }
  }
  else
  {
    if (msg.data > 0)
    {
      thrusterNorthSway(msg.data, false, true);
      thrusterSouthSway(msg.data, true, true);
    }
    else
    {
      thrusterNorthSway(msg.data, true, true);
      thrusterSouthSway(msg.data, false, true);
    }
  }
}

// Delete me
ros::Publisher echo_MinUpwardPWM("/pwm/echo/minupwardpwm", &echo);

void setMinUpwardPWM(const std_msgs::Int32& msg)
{
  minUpwardPWM = msg.data;
  // Delete me
  echo.data = minUpwardPWM;
  echo_MinUpwardPWM.publish(&echo);
}

void setBiasSouthUp(const std_msgs::Int32& msg)
{
  biasSouthUp = msg.data;
}

ros::Subscriber<std_msgs::Int32> subPwmForward("/pwm/forward", &PWMCbForward);
ros::Subscriber<std_msgs::Int32> subPwmSideward("/pwm/sideward", &PWMCbSideward);
ros::Subscriber<std_msgs::Int32> subPwmUpward("/pwm/upward", &PWMCbUpward);
ros::Subscriber<std_msgs::Int32> subPwmTurn("/pwm/turn", &PWMCbTurn);
ros::Publisher ps_voltage("/varun/sensors/pressure_sensor/depth", &voltage);
ros::Subscriber<std_msgs::Int32> subMinUpwardPWM("/pwm/minupwardpwm", &setMinUpwardPWM);
ros::Subscriber<std_msgs::Int32> subBiasSouthUp("/pwm/biassouthup", &setBiasSouthUp);

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
  nh.subscribe(subPwmTurn);
  nh.subscribe(subMinUpwardPWM);
  nh.subscribe(subBiasSouthUp);
  nh.advertise(ps_voltage);
  // Delete me
  nh.advertise(echo_MinUpwardPWM);
  Serial.begin(57600);
  std_msgs::Int32 msg;
  msg.data = 0;
  PWMCbForward(msg);
  PWMCbSideward(msg);
  PWMCbUpward(msg);
  PWMCbTurn(msg);

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

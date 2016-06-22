// Copyright 2016 AUV-IITK
#include <ros.h>
#include <Arduino.h>
#include <std_msgs/Int32.h>

// /*************************** for AUV ****************************/
// #define led 13

// #define pwmPinWest 5
// #define pwmPinEast 4
// #define directionPinWest1 27
// #define directionPinWest2 26
// #define directionPinEast1 29
// #define directionPinEast2 28

// #define pwmPinNorthSway 5
// #define pwmPinSouthSway 4
// #define directionPinNorthSway1 27
// #define directionPinNorthSway2 26
// #define directionPinSouthSway1 29
// #define directionPinSouthSway2 28

// #define pwmPinNorthUp 5
// #define pwmPinSouthUp 4
// #define directionPinNorthUp1 27
// #define directionPinNorthUp2 26
// #define directionPinSouthUp1 29
// #define directionPinSouthUp2 28

/************************ for New Ground Testing Bot *******************/
#define led 13  // 13

<<<<<<< HEAD
=======
#define pwmPinWest 7          // 3
#define pwmPinEast 6          // 2
#define directionPinWest1 23  // 31
#define directionPinWest2 22  // 30
#define directionPinEast1 24  // 33
#define directionPinEast2 25  // 32

#define pwmPinNorthSway 4          // 3
#define pwmPinSouthSway 5          // 2
#define directionPinNorthSway1 28  // 31
#define directionPinNorthSway2 29  // 30
#define directionPinSouthSway1 27  // 33
#define directionPinSouthSway2 26  // 32
>>>>>>> cherrypic errors
#define pwmPinWest 5  // 3
#define pwmPinEast 2  // 2
#define directionPinWest1 6  // 31
#define directionPinWest2 7  // 30
#define directionPinEast1 3  // 33
#define directionPinEast2 4  // 32

#define pwmPinNorthSway 23  // 3
#define pwmPinSouthSway 13  // 2
#define directionPinNorthSway1 8  // 31
#define directionPinNorthSway2 9  // 30
#define directionPinSouthSway1 11  // 33
#define directionPinSouthSway2 12  // 32

#define pwmPinNorthUp 13  // 3
#define pwmPinSouthUp 13  // 2
#define directionPinNorthUp1 0  // 31
#define directionPinNorthUp2 0  // 30
#define directionPinSouthUp1 0  // 33
#define directionPinSouthUp2 0  // 32

/************************ for old ground bot ******************************/
// #define pwmPinWest 4
// #define pwmPinEast 5
// #define directionPinWest1 2
// #define directionPinWest2 3
// #define directionPinEast1 7
// #define directionPinEast2 6
// #define led 13

int Delay = 1500;
bool isMovingForward = true;
ros::NodeHandle nh;

void thrusterNorthUp(int pwm, int isUpward)
{
  analogWrite(pwmPinNorthUp, pwm);
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
  analogWrite(pwmPinSouthUp, pwm);
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

void thrusterNorthSway(int pwm, int isRight)
{
  analogWrite(pwmPinNorthSway, pwm);
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

void thrusterSouthSway(int pwm, int isRight)
{
  analogWrite(pwmPinSouthSway, pwm);
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

void thrusterEast(int pwm, int isForward)
{
  analogWrite(pwmPinEast, pwm);
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

void thrusterWest(int pwm, int isForward)
{
  analogWrite(pwmPinWest, pwm);
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
    thrusterEast(255 - msg.data, true);
    thrusterWest(255 - msg.data, true);
  }
  else
  {
    thrusterEast(255 + msg.data, false);
    thrusterWest(255 + msg.data, false);
  }
  isMovingForward = true;
}

void PWMCbSideward(const std_msgs::Int32& msg)
{
  if (msg.data > 0)
  {
    thrusterNorthSway(255 - msg.data, true);
    thrusterSouthSway(255 - msg.data, true);
  }
  else
  {
    thrusterNorthSway(255 + msg.data, false);
    thrusterSouthSway(255 + msg.data, false);
  }
  isMovingForward = false;
}

void PWMCbUpward(const std_msgs::Int32& msg)
{
  if (msg.data > 0)
  {
    thrusterNorthUp(255 - msg.data, true);
    thrusterSouthUp(255 - msg.data, true);
  }
  else
  {
    thrusterNorthUp(255 + msg.data, false);
    thrusterSouthUp(255 + msg.data, false);
  }
}

void PWMCbTurn(const std_msgs::Int32& msg)
{
  if (isMovingForward)
  {
    if (msg.data > 0)
    {
      thrusterEast(255 - msg.data, true);
      thrusterWest(255 - msg.data, false);
    }
    else
    {
      thrusterEast(255 + msg.data, false);
      thrusterWest(255 + msg.data, true);
    }
  }
  else
  {
    if (msg.data > 0)
    {
      thrusterNorthSway(255 - msg.data, false);
      thrusterSouthSway(255 - msg.data, true);
    }
    else
    {
      thrusterNorthSway(255 + msg.data, true);
      thrusterSouthSway(255 + msg.data, false);
    }
  }
}

ros::Subscriber<std_msgs::Int32> subPwmForward("/pwm/forward", &PWMCbForward);
ros::Subscriber<std_msgs::Int32> subPwmSideward("/pwm/sideward", &PWMCbSideward);
ros::Subscriber<std_msgs::Int32> subPwmUpward("/pwm/upward", &PWMCbUpward);
ros::Subscriber<std_msgs::Int32> subPwmTurn("/pwm/turn", &PWMCbTurn);

void setup()
{
  nh.initNode();

  pinMode(led, OUTPUT);
  pinMode(directionPinEast1, OUTPUT);
  pinMode(directionPinEast2, OUTPUT);
  pinMode(pwmPinWest, OUTPUT);
  pinMode(directionPinWest2, OUTPUT);
  pinMode(pwmPinEast, OUTPUT);
  pinMode(directionPinWest1, OUTPUT);

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
  Serial.begin(57600);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}

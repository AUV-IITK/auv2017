/* 
 * rosserial::std_msgs::Float64 Test
 * Receives a Float32MultiArray input, subtracts 1.0, and publishes it
*/

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

#include <Arduino.h>
ros::NodeHandle nh;

//float x;

void messageCb( const std_msgs::Float32MultiArray& msg){
  //x = msg.data[4];
if(msg.data[4]==1) digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

//std_msgs::Float32MultiArray test;
ros::Subscriber<std_msgs::Float32MultiArray> s("balls", &messageCb);
//ros::Publisher p("my_topic", &test);

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  //nh.advertise(p);
  nh.subscribe(s);
}

void loop()
{
  //test.data = x;
  //p.publish( &test );
  nh.spinOnce();
  delay(10);
}
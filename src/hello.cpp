#include <ros.h>
#include <Arduino.h>
#include <std_msgs/Int32.h>//to use any type of std_msgs just include the library of it's name
ros::NodeHandle nh;
//std_msgs::String str;
std_msgs::Int32 i;
int count=0;
int pwm;
void cb(const std_msgs::Int32& msg){//this "Int16" can be replaced by any std_msgs data type	
	pwm=msg.data;
    if(pwm>0){
        digitalWrite(2,HIGH);
        digitalWrite(3,LOW);
        analogWrite(4,pwm);

        digitalWrite(7,HIGH);
        digitalWrite(6,LOW);
        analogWrite(5,pwm);
        digitalWrite(13,LOW);
    }
    else{
        pwm=-pwm;
        digitalWrite(6,HIGH);
        digitalWrite(7,LOW);
        analogWrite(5,pwm);

        digitalWrite(3,HIGH);
        digitalWrite(2,LOW);
        analogWrite(4,pwm);
        digitalWrite(13,HIGH);
    }
	Serial.print("got pwm");
}
//ros::Subscriber sub = nh.subscribe<std_msgs::String>("commander", 1000, &cb);

ros::Subscriber<std_msgs::Int32> sub("pwm", &cb );//only this syntax is correct for defining the "sub" not the upper commented one(to find out it's reason)
void setup(){
    nh.initNode();
    pinMode(13, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(2, OUTPUT);
    nh.subscribe(sub);
    Serial.begin(57600);
}
void loop(){
	nh.spinOnce();
	delay(1);
}

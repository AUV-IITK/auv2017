#include <ros.h>
#include <Arduino.h>
#include <std_msgs/Int32.h>//to use any type of std_msgs just include the library of it's name
//#include <std_msgs/Char.h>
ros::NodeHandle nh;

//for AUV
#define pwma 5      //3   
#define pwmb 4      //2
#define ina1 27     //31
#define ina2 26     //30
#define inb1 29     //33
#define inb2 28     //32
#define led 13      //13

// //for ground bot
// #define pwma 4
// #define pwmb 5
// #define ina1 2
// #define ina2 3
// #define inb1 7
// #define inb2 6
// #define led 13

int pwm= 255;
void PWMCb(const std_msgs::Int32& msg){
	pwm = 255 - msg.data;
    analogWrite(pwma,pwm);
    analogWrite(pwmb,pwm);

}

void directionCb(const std_msgs::Int32& msg){
	if (msg.data== 1){//forward input
        digitalWrite(ina1,HIGH);
        digitalWrite(ina2,LOW);
        digitalWrite(inb1,HIGH);
        digitalWrite(inb2,LOW);
	}
	else if(msg.data == 2){//backward input
        digitalWrite(ina1,LOW);
        digitalWrite(ina2,HIGH);
        digitalWrite(inb1,LOW);
        digitalWrite(inb2,HIGH);
	}
	else if(msg.data == 4){//cloclwise
        digitalWrite(ina1,LOW);
        digitalWrite(ina2,HIGH);
        digitalWrite(inb1,HIGH);
        digitalWrite(inb2,LOW);
	}
	else if(msg.data == 3){//anti clockwise
        digitalWrite(ina1,HIGH);
        digitalWrite(ina2,LOW);
        digitalWrite(inb1,LOW);
        digitalWrite(inb2,HIGH);
	}
	else {
        digitalWrite(ina1,LOW);
        digitalWrite(ina2,LOW);
        digitalWrite(inb1,LOW);
        digitalWrite(inb2,LOW);		
	}
}

ros::Subscriber<std_msgs::Int32> subPWM("PWM", &PWMCb );//only this syntax is correct for defining the "sub" not the upper commented one(to find out it's reason)
ros::Subscriber<std_msgs::Int32> subdirection("direction", &directionCb );

void setup(){
    nh.initNode();
    pinMode(led, OUTPUT);
    pinMode(inb1, OUTPUT);
    pinMode(inb2, OUTPUT);
    pinMode(pwma, OUTPUT);
    pinMode(ina2, OUTPUT);
    pinMode(pwmb, OUTPUT);
    pinMode(ina1, OUTPUT);
    nh.subscribe(subPWM );
    nh.subscribe(subdirection);
    Serial.begin(57600);
}
void loop(){
	nh.spinOnce();
	delay(1);
}
#include <ros.h>
#include <Arduino.h>
#include <std_msgs/Int32.h>//to use any type of std_msgs just include the library of it's name
ros::NodeHandle nh;
//std_msgs::String str;
//std_msgs::Int32 i;

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

int stop = 0;//this is used to stop the bot
int count= 0;
int pwm;
void turningCb(const std_msgs::Int32& msg){//this "Int32" can be replaced by any std_msgs data type	
	pwm=msg.data;
    if(pwm>0){
        digitalWrite(ina1,HIGH);
        digitalWrite(ina2,LOW);
        analogWrite(pwma,pwm);

        digitalWrite(inb2,HIGH);
        digitalWrite(inb1,LOW);
        analogWrite(pwmb,pwm);
    }
    else{
        pwm=-pwm;
        digitalWrite(inb1,HIGH);
        digitalWrite(inb2,LOW);
        analogWrite(pwmb,pwm);

        digitalWrite(ina2,HIGH);
        digitalWrite(ina1,LOW);
        analogWrite(pwma,pwm);
    }
}
void stopBotCb(const std_msgs::Int32& msg){
    if(msg.data == 1){
        stop = 1;
        digitalWrite(ina1,LOW);
        digitalWrite(ina2,LOW);
        digitalWrite(inb1,LOW);
        digitalWrite(inb2,LOW);
        analogWrite(pwma,0);
        analogWrite(pwmb,0);
    }
    else
        stop = 0;

}

//ros::Subscriber sub = nh.subscribe<std_msgs::String>("commander", 1000, &cb);

ros::Subscriber<std_msgs::Int32> sub("turningPWM", &turningCb );//only this syntax is correct for defining the "sub" not the upper commented one(to find out it's reason)
ros::Subscriber<std_msgs::Int32> subStop("stopBot", &stopBotCb );
void setup(){
    nh.initNode();
    pinMode(led, OUTPUT);
    pinMode(inb1, OUTPUT);
    pinMode(inb2, OUTPUT);
    pinMode(pwma, OUTPUT);
    pinMode(ina2, OUTPUT);
    pinMode(pwmb, OUTPUT);
    pinMode(ina1, OUTPUT);
    nh.subscribe(sub);
    nh.subscribe(subStop);
    Serial.begin(57600);
}
void loop(){
    while(stop){
        delay(1000);
    }
	nh.spinOnce();
	delay(1);
}
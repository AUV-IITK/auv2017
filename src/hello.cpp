#include <ros.h>
#include <Arduino.h>
#include <std_msgs/Int32.h>//to use any type of std_msgs just include the library of it's name
//#include <std_msgs/Char.h>
ros::NodeHandle nh;

//for AUV 
#define led 13      //13

#define pwmax 5      //3   
#define pwmbx 4      //2
#define ina1x 27     //31
#define ina2x 26     //30
#define inb1x 29     //33
#define inb2x 28     //32

#define pwmay 5     //3   
#define pwmby 4      //2
#define ina1y 27     //31
#define ina2y 26     //30
#define inb1y 29     //33
#define inb2y 28     //32

#define pwmaz 5      //3   
#define pwmbz 4      //2
#define ina1z 27     //31
#define ina2z 26     //30
#define inb1z 29     //33
#define inb2z 28     //32

// //for ground bot
// #define pwmax 4
// #define pwmbx 5
// #define ina1x 2
// #define ina2x 3
// #define inb1x 7
// #define inb2x 6
// #define led 13

int pwmx = 255;
int pwmy = 255;
int pwmz = 255;
int prevDir =0;
int Delay = 1500;

void PWMCbx(const std_msgs::Int32& msg){
    pwmx = 255 - msg.data;
    analogWrite(pwmax,pwmx);
    analogWrite(pwmbx,pwmx);

}

void PWMCby(const std_msgs::Int32& msg){
    pwmy = 255 - msg.data;
    analogWrite(pwmay,pwmy);
    analogWrite(pwmby,pwmy);

}

void PWMCbz(const std_msgs::Int32& msg){
    pwmz = 255 - msg.data;
    analogWrite(pwmaz,pwmz);
    analogWrite(pwmbz,pwmz);

}


void directionCbx(const std_msgs::Int32& msg){
    if (msg.data== 1){//forward input
        prevDir=1;
        digitalWrite(ina1x,HIGH);
        digitalWrite(ina2x,LOW);
        digitalWrite(inb1x,HIGH);
        digitalWrite(inb2x,LOW);
    }
    else if(msg.data == 2){//backward input
        prevDir=2;
        digitalWrite(ina1x,LOW);
        digitalWrite(ina2x,HIGH);
        digitalWrite(inb1x,LOW);
        digitalWrite(inb2x,HIGH);
    }
    else if(msg.data == 4){//cloclwise
        digitalWrite(ina1x,LOW);
        digitalWrite(ina2x,HIGH);
        digitalWrite(inb1x,HIGH);
        digitalWrite(inb2x,LOW);
    }
    else if(msg.data == 3){//anti clockwise
        digitalWrite(ina1x,HIGH);
        digitalWrite(ina2x,LOW);
        digitalWrite(inb1x,LOW);
        digitalWrite(inb2x,HIGH);
    }
//just a temp method to try to stop the bot while moving forward
    else if(msg.data == 5){
        if(prevDir==1){
            digitalWrite(ina1x,LOW);
            digitalWrite(ina2x,HIGH);
            digitalWrite(inb1x,LOW);
            digitalWrite(inb2x,HIGH);
            delay(Delay);
        }
        else{
            digitalWrite(ina1x,HIGH);
            digitalWrite(ina2x,LOW);
            digitalWrite(inb1x,HIGH);
            digitalWrite(inb2x,LOW);
            delay(Delay);
        }
        digitalWrite(ina1x,LOW);
        digitalWrite(ina2x,LOW);
        digitalWrite(inb1x,LOW);
        digitalWrite(inb2x,LOW);
    }

    else {
        digitalWrite(ina1x,LOW);
        digitalWrite(ina2x,LOW);
        digitalWrite(inb1x,LOW);
        digitalWrite(inb2x,LOW);
    }
}

ros::Subscriber<std_msgs::Int32> subPWMx("PWMx", &PWMCbx );//only this syntax is correct for defining the "sub" not the upper commented one(to find out it's reason)
ros::Subscriber<std_msgs::Int32> subdirectionx("directionx", &directionCbx );



void directionCby(const std_msgs::Int32& msg){
    if (msg.data== 1){//forward input
        prevDir=1;
        digitalWrite(ina1y,HIGH);
        digitalWrite(ina2y,LOW);
        digitalWrite(inb1y,HIGH);
        digitalWrite(inb2y,LOW);
    }
    else if(msg.data == 2){//backward input
        prevDir=2;
        digitalWrite(ina1y,LOW);
        digitalWrite(ina2y,HIGH);
        digitalWrite(inb1y,LOW);
        digitalWrite(inb2y,HIGH);
    }
    else if(msg.data == 4){//cloclwise
        digitalWrite(ina1y,LOW);
        digitalWrite(ina2y,HIGH);
        digitalWrite(inb1y,HIGH);
        digitalWrite(inb2y,LOW);
    }
    else if(msg.data == 3){//anti clockwise
        digitalWrite(ina1y,HIGH);
        digitalWrite(ina2y,LOW);
        digitalWrite(inb1y,LOW);
        digitalWrite(inb2y,HIGH);
    }
//just a temp method to try to stop the bot while moving forward
    else if(msg.data == 5){
        if(prevDir==1){
            digitalWrite(ina1y,LOW);
            digitalWrite(ina2y,HIGH);
            digitalWrite(inb1y,LOW);
            digitalWrite(inb2y,HIGH);
            delay(Delay);
        }
        else{
            digitalWrite(ina1y,HIGH);
            digitalWrite(ina2y,LOW);
            digitalWrite(inb1y,HIGH);
            digitalWrite(inb2y,LOW);
            delay(Delay);
        }
        digitalWrite(ina1y,LOW);
        digitalWrite(ina2y,LOW);
        digitalWrite(inb1y,LOW);
        digitalWrite(inb2y,LOW);
    }

    else {
        digitalWrite(ina1y,LOW);
        digitalWrite(ina2y,LOW);
        digitalWrite(inb1y,LOW);
        digitalWrite(inb2y,LOW);
    }
}

ros::Subscriber<std_msgs::Int32> subPWMy("PWMy", &PWMCby );//only this syntay is correct for defining the "sub" not the upper commented one(to find out it's reason)
ros::Subscriber<std_msgs::Int32> subdirectiony("directiony", &directionCby );



void directionCbz(const std_msgs::Int32& msg){
    if (msg.data== 1){//forward input
        prevDir=1;
        digitalWrite(ina1z,HIGH);
        digitalWrite(ina2z,LOW);
        digitalWrite(inb1z,HIGH);
        digitalWrite(inb2z,LOW);
    }
    else if(msg.data == 2){//backward input
        prevDir=2;
        digitalWrite(ina1z,LOW);
        digitalWrite(ina2z,HIGH);
        digitalWrite(inb1z,LOW);
        digitalWrite(inb2z,HIGH);
    }
    else if(msg.data == 4){//cloclwise
        digitalWrite(ina1z,LOW);
        digitalWrite(ina2z,HIGH);
        digitalWrite(inb1z,HIGH);
        digitalWrite(inb2z,LOW);
    }
    else if(msg.data == 3){//anti clockwise
        digitalWrite(ina1z,HIGH);
        digitalWrite(ina2z,LOW);
        digitalWrite(inb1z,LOW);
        digitalWrite(inb2z,HIGH);
    }
//just a temp method to try to stop the bot while moving forward
    else if(msg.data == 5){
        if(prevDir==1){
            digitalWrite(ina1z,LOW);
            digitalWrite(ina2z,HIGH);
            digitalWrite(inb1z,LOW);
            digitalWrite(inb2z,HIGH);
            delay(Delay);
        }
        else{
            digitalWrite(ina1z,HIGH);
            digitalWrite(ina2z,LOW);
            digitalWrite(inb1z,HIGH);
            digitalWrite(inb2z,LOW);
            delay(Delay);
        }
        digitalWrite(ina1z,LOW);
        digitalWrite(ina2z,LOW);
        digitalWrite(inb1z,LOW);
        digitalWrite(inb2z,LOW);
    }

    else {
        digitalWrite(ina1z,LOW);
        digitalWrite(ina2z,LOW);
        digitalWrite(inb1z,LOW);
        digitalWrite(inb2z,LOW);
    }
}

ros::Subscriber<std_msgs::Int32> subPWMz("PWMz", &PWMCbz );//onlz this szntaz is correct for defining the "sub" not the upper commented one(to find out it's reason)
ros::Subscriber<std_msgs::Int32> subdirectionz("directionz", &directionCbz );

void setup(){
    nh.initNode();
    pinMode(led, OUTPUT);
    pinMode(inb1x, OUTPUT);
    pinMode(inb2x, OUTPUT);
    pinMode(pwmax, OUTPUT);
    pinMode(ina2x, OUTPUT);
    pinMode(pwmbx, OUTPUT);
    pinMode(ina1x, OUTPUT);

    pinMode(inb1y, OUTPUT);
    pinMode(inb2y, OUTPUT);
    pinMode(pwmay, OUTPUT);
    pinMode(ina2y, OUTPUT);
    pinMode(pwmby, OUTPUT);
    pinMode(ina1y, OUTPUT);

    pinMode(inb1z, OUTPUT);
    pinMode(inb2z, OUTPUT);
    pinMode(pwmaz, OUTPUT);
    pinMode(ina2z, OUTPUT);
    pinMode(pwmbz, OUTPUT);
    pinMode(ina1z, OUTPUT);

    nh.subscribe(subPWMx);
    nh.subscribe(subPWMy);
    nh.subscribe(subPWMz);

    nh.subscribe(subdirectionx);
    nh.subscribe(subdirectiony);
    nh.subscribe(subdirectionz);
    Serial.begin(57600);
}
void loop(){
    nh.spinOnce();
    delay(1);
}
//#include "I2Cdev.h"
#include "Wire.h"
#include <ros.h>
#include <std_msgs/Float64.h>
#define encoder1A A0
#define encoder1B A1
#define resolution 540
double kp = 0.06;
double ki = 0.055;
double kd = 0.04;
int counts,dcount;
int lastcount=0;
unsigned long timeNow,dtime;
unsigned long timeInterval=10000;
unsigned long lastTime=0;
double error,lastError ;
double response,setpoint;
double RPM=0.0;
double cumError,derror;
//HardwareSerial Serial3(PIN_RX_TTL, PIN_TX_TTL);

ros::NodeHandle nh;
///std_msgs::Float64 setpoint;
void messageCb(std_msgs::Float64& msg)
{
  setpoint=msg.data;
  }
ros::Subscriber<std_msgs::Float64> sub("topicRPM", &messageCb);
  
void setup() {
nh.getHardware()->setPort(&Serial);
 nh.getHardware()->setBaud(115200);   
 nh.initNode();
//Serial3.begin(15200);
pinMode(encoder1A,INPUT_PULLUP);
pinMode(encoder1B,INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(encoder1A),isr1A,CHANGE);
attachInterrupt(digitalPinToInterrupt(encoder1B),isr1B,CHANGE);
nh.subscribe(sub);
}

void loop() {
  timeNow=millis();
  dtime=timeNow-lastTime;
  dcount=counts-lastcount;
  RPM=(dcount*1.0/resolution)*(60000/dtime);
  response=pid();
//  if(dtime==timeInterval){
//    dcount=counts-lastcount;
//    }
   
    lastTime=timeNow;
    lastcount=counts;
    
///Serial.print(counts);
}
double pid(){
   error=setpoint-RPM;
   cumError += dtime*error;
   derror=(error-lastError)/dtime;
   response=kp*error + ki*cumError + kd*derror;
   lastError=error;
   return response;
  }
void isr1A(){
if(digitalRead(encoder1A)!=digitalRead(encoder1B))
  counts++;
  else 
  counts--;
}

void isr1B(){
if(digitalRead(encoder1A)==digitalRead(encoder1B))
  counts++;
  else 
  counts--;

}

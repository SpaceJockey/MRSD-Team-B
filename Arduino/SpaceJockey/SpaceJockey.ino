/*
SpaceJockey.ino
Written by Nathaniel Chapman, 11/23/2013
MRSD Team B
This Sketch listens to the ROS channel "serial_link" and updates joint positions accordingly
*/

//All Arduino-specific libraries must be included here before they will work in headers
#include <Arduino.h>
#include <Scheduler.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ros.h>
#include "SpaceJockey.h"

//ROS Stuff
ros::NodeHandle nh; //declared "extern" in SpaceJockey.h

void setup() {
  //Set up ROS node
  nh.initNode();
  
  //init monitoring threads
  Battery.begin();

  //init Servo Driver and Subscriber
  Servos.begin();
  Joints.begin();
  
  //wait until you are actually connected
  while (!nh.connected()) nh.spinOnce();
  
  nh.loginfo("Space Jockey Arduino online...");
}

//Main loop just spins ROS
void loop() {  
	nh.spinOnce();
	delay(1);
}

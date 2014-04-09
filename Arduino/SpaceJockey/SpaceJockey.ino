/*
SpaceJockey.ino
Written by Nathaniel Chapman, 3/4/2014
CMU MRSD Team B
This is the firmware sketch for the Space Jockey Robot
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

  //init servo driver
  Servos.begin();

  //init monitoring threads
  Battery.begin();
  IMU.begin();

  //spin until rosserial connection is live
  while (!nh.connected()) nh.spinOnce();
  nh.loginfo("Space Jockey Arduino Connected.");
}

//Main loop just spins ROS
void loop() {  
	nh.spinOnce();
	delay(1);
}

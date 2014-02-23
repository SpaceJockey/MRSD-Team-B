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
ros::NodeHandle nh;

void setup() {
  //Set up status LEDS
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  //Set up ROS node
  nh.initNode();
  
  //init monitoring threads
  Battery.begin(nh);
  Debug.begin(nh);
  
  //init Servo Driver and Subscriber
  Servos.begin();
  Joints.begin(nh, Servos);
}

//Main loop just spins ROS
void loop() {  
	nh.spinOnce();
	delay(1);
}
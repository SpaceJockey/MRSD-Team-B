/*
SpaceJockey.ino
Written by Nathaniel Chapman, 11/23/2013
MRSD Team B
This Sketch listens to the ROS channel "serial_link" and updates joint positions accordingly
*/

#include <Arduino.h>
#include <Scheduler.h>
#include "SpaceJockey.h"

//ROS Stuff
#include <ros.h>
ros::NodeHandle nh;

void setup() {
  //Set up status LEDS
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  //Set up ROS node
  nh.initNode();
  Battery.begin();
  Debug.begin();

}

//Main loop just spins ROS
void loop() {  
	nh.spinOnce();
	delay(1);
}
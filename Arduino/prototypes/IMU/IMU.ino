// MPU-6050 Accelerometer + Gyro
// -----------------------------

#include <Arduino.h>
#include <Scheduler.h>
#include <Wire.h>
#include "MPU6050.h"

//ROS stuff
#include <ros.h>

//ROS Stuff
ros::NodeHandle nh; //declared "extern" in SpaceJockey.h

void setup()
{ 
  //Set up ROS node
  nh.initNode();

  IMU.begin();

  //spin until rosserial connection is live
  while (!nh.connected()) nh.spinOnce();
  nh.loginfo("IMU Dumper Arduino Connected.");
}


void loop()
{
  nh.spinOnce();
  delay(1);
}



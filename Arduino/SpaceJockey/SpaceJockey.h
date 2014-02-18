#ifndef SPACEJOCKEY_H
#define SPACEJOCKEY_H

//General Debugging stuff
#define STATUS_LED 13

/* Radian values for us degree-centric folks */
#define M_PI 3.14159265
#define DEG_20 (M_PI / 9)
#define DEG_30 (M_PI / 6)
#define DEG_45 (M_PI / 4)
//float degtorad(float deg){return (deg * M_PI) / 180.0;}

#include <Arduino.h>
#include <Scheduler.h>

//ROS Stuff
#include <ros.h>
// ROS node handle declared in main sketch
extern ros::NodeHandle  nh;


//Subsystems

//#include "Body.h"
#include "Battery.h"
#include "Debug.h"

#endif
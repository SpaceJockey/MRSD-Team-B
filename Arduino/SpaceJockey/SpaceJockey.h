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

/*Robot Configuration Parameters

ROS Joint configs are indexed so...
0	Center Swivel
1	Center Segment Front Pitch
2	Center Segment Rear Pitch
3	Front Segment Pitch
4	Rear Segment Pitch
5	Center Segment Front Prismatic
6	Center Segment Rear Prismatic

Servo Wiring Configs:
12	Center Swivel
2	Center Segment Front Pitch
10	Center Segment Rear Pitch
3	Front Segment Pitch
11	Rear Segment Pitch
0	Center Segment Front Prismatic
8	Center Segment Rear Prismatic

*/

//These are the configuration values for the robot, they need to be tuned!
//These are in radians or meters, depending on joint types
const float real_min[] = {-DEG_30, -DEG_30, -DEG_30, -DEG_45, -DEG_45,   0,   0};
const float real_max[] = { DEG_30,  DEG_30,  DEG_30,  DEG_45,  DEG_45, .145, .145};

//Servo addresses for the HV servo board
const unsigned int servo_addr[] = {12, 2, 10, 3, 11, 0, 8};


#include <Arduino.h>
#include <Scheduler.h>

//ROS Stuff
#include <ros.h>
// ROS node handle declared in main sketch
extern ros::NodeHandle  nh;


//Subsystems
#include "Debug.h"
#include "Robot.h"
#include "Battery.h"

#endif
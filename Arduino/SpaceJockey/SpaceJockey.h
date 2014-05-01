#ifndef SPACEJOCKEY_H
#define SPACEJOCKEY_H

//Arduino pin defines
#define STATUS_LED 13
#define BATT_PIN A0    //pin for battery monitoring
#define SERVO_FEEDBACK_PIN A5 //pin for servo calibration

#include <Arduino.h>
#include <Scheduler.h>
#include <ros.h>

extern ros::NodeHandle nh;

//Subsystems
#include "Actuators.h"
#include "Battery.h"
//#include "IMU.h"

#endif

#ifndef SPACEJOCKEY_H
#define SPACEJOCKEY_H

//Arduino pin defines
#define STATUS_LED 13
#define BATT_PIN A8    //pin for battery monitoring
#define BATT_LED 12    //pin for led indicator for battery

#include <Arduino.h>
#include <Scheduler.h>
#include <ros.h>

extern ros::NodeHandle nh;

//Subsystems
#include "Robot.h"
#include "Battery.h"
#include "JointSub.h"

#endif

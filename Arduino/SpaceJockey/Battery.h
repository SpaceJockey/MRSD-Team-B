#ifndef BATTERY_H
#define BATTERY_H

#include "SpaceJockey.h"

//Battery Monitoring LED
#define BATT_PIN A8    //pin for battery monitoring
#define BATT_LED 12    //pin for led indicator for battery

//Battery Calibration Values
#define BATT_6V   1700 //6V = 0%
#define BATT_85V  1870 //8.5V = 100%


//ROS Stuff
#include <ros.h>
#include <std_msgs/Int16.h>

static int batt_mon;
static std_msgs::Int16 batt_msg; //battery state output
static ros::Publisher batt_state("battery_state", &batt_msg);

//update battery state every half second
static void battLoop() {
	//Update battery voltage warning light
	int batt_read = analogRead(BATT_PIN);
	if(abs(batt_read - batt_mon) < 25) {
		batt_mon = ((7 * batt_mon) + batt_read) / 8; 
	} else {
		batt_mon = ((15 * batt_mon) + batt_read) / 16; 
	}
	batt_msg.data = map(batt_mon, BATT_6V, BATT_85V, 0, 100); //battery percentage
	if (batt_msg.data > 100) batt_msg.data = 100;
	if (batt_msg.data < 0) batt_msg.data = 0;
	//TODO: skew percentage to match logarithmic batt discharge curve
	
	if (batt_msg.data <= 40) { //if battery level is less than 40%
		digitalWrite(BATT_LED, HIGH); 
	}else{
		digitalWrite(BATT_LED, LOW);
	}
	batt_state.publish( &batt_msg );
	delay(500);
}

class Batt {
	public:
		void begin() { //ros::NodeHandle nh) {
			//setup output pins
			pinMode(BATT_LED, OUTPUT);
			digitalWrite(BATT_LED, LOW);
  
			//Set up Battery Monitoring
			analogReadResolution(12);
			batt_mon = analogRead(BATT_PIN);
			nh.advertise(batt_state);
			Scheduler.startLoop(battLoop);
		}	
};

Batt Battery;
#endif

#ifndef BATTERY_H
#define BATTERY_H

#include "SpaceJockey.h"

//ROS Stuff
#include <ros.h>
#include <std_msgs/Int16.h>

static unsigned int batt_avg;
static std_msgs::Int16 batt_msg; //battery state output
static ros::Publisher batt_state("battery_raw", &batt_msg);

//update battery state every half second
static void battLoop() {
	int batt_read = analogRead(BATT_PIN);
	if(abs(batt_read - batt_avg) < 25) { //filter out noise
		batt_avg = ((7 * batt_avg) + batt_read) / 8; 
	} else {
		batt_avg = ((15 * batt_avg) + batt_read) / 16; 
	}
	batt_msg.data = batt_avg;

	batt_state.publish( &batt_msg );
	delay(500);
}

class Batt {
	public:
		void begin() {
			//Set up Battery Monitoring
			analogReadResolution(12);
			batt_avg = analogRead(BATT_PIN);
			nh.advertise(batt_state);
			Scheduler.startLoop(battLoop);
		}	
};

Batt Battery;
#endif

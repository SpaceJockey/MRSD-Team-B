#ifndef DEBUG_H
#define DEBUG_H
#include "SpaceJockey.h"

//ROS Stuff
#include <ros.h>
#include <std_msgs/String.h>

std_msgs::String debug_msg;
ros::Publisher debug_pub("arduino_debug", &debug_msg);

class Dbg {
	public:
		void begin() {
			nh.advertise(debug_pub);
			this->print("Space Jockey Online");
		}
		
		// Publish an incoming string to the debug topic
		void print(char* s) {
			debug_msg.data = s;
			debug_pub.publish( &debug_msg );
		}	
};

Dbg Debug;
#endif
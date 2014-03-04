#ifndef JOINTSUB_H
#define JOINTSUB_H
#include "SpaceJockey.h"

//ROS Stuff
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>

static uint8_t blink; //used for debugging...

//ROS subscriber
static void joint_cb(const std_msgs::Float64MultiArray& cmd_msg){
	//blink LED to indicate CB is running...
	blink = 1-blink;
	digitalWrite(STATUS_LED, blink);
	
	//update servos
	for(int c = 0; c < cmd_msg.data_length; c++) Servos.setServoPos(c, cmd_msg.data[c]);
}
static ros::Subscriber<std_msgs::Float64MultiArray> link_sub("joint_ctl", joint_cb);

class JointSub{
	public:
		void begin(){
		    //Set up status LED
            pinMode(STATUS_LED, OUTPUT);
            digitalWrite(STATUS_LED, LOW);
            nh.subscribe(link_sub);
        }
};

JointSub Joints;
#endif

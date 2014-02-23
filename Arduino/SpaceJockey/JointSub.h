#ifndef JOINTSUB_H
#define JOINTSUB_H
#include "SpaceJockey.h"

//ROS Stuff
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>

static Robot cRobot; //current robot object to update
static uint8_t blink; //used for debugging...

//ROS subscriber
static void joint_cb(const std_msgs::Float64MultiArray& cmd_msg){
	//blink LED to indicate CB is running...
	blink = 1-blink;
	digitalWrite(STATUS_LED, blink);
	
	//update servos
	for(int c = 0; c < cmd_msg.data_length; c++) cRobot.setServoPos(c, cmd_msg.data[c]);
}
static ros::Subscriber<std_msgs::Float64MultiArray> link_sub("serial_link", joint_cb);

class JointSub{
	public:
		void begin(ros::NodeHandle nh, Robot robot){
			cRobot = robot;
			nh.subscribe(link_sub);
		}
};

JointSub Joints;

#endif
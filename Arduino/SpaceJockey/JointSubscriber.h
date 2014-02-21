#ifndef JOINTSUBSCRIBER_H
#define JOINTSUBSCRIBER_H
#include "SpaceJockey.h"

//ROS Stuff
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>

class Robot {
	public:
		Robot();
		void begin(){
			//_link_sub = ros::Subscriber<std_msgs::Float64MultiArray>("serial_link", this->_joint_cb);
			nh.subscribe(_link_sub);
		}
	private:
		uint8_t _blink; //used for debugging...
		
		//ROS subscriber
		ros::Subscriber<std_msgs::Float64MultiArray> _link_sub; //("serial_link", jointstate_cb);
		void _joint_cb(const std_msgs::Float64MultiArray& cmd_msg){
			//blink LED to indicate CB is running...
			_blink = 1-_blink;
			digitalWrite(STATUS_LED, _blink);
			
			//update servos
			for(int c = 0; c < cmd_msg.data_length; c++) setServoPos(c, cmd_msg.data[c]);
		}
};

#endif
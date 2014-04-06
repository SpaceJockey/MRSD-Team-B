#ifndef JOINTSUB_H
#define JOINTSUB_H
#include "SpaceJockey.h"

//ROS Stuff
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

//ROS subscriber
static void joint_cb(const std_msgs::Int16MultiArray& cmd_msg){
	//update servos
	for(int c = 0; c < cmd_msg.data_length; c++) Servos.setServoPos(c, (unsigned int) cmd_msg.data[c]);
}
static ros::Subscriber<std_msgs::Int16MultiArray> link_sub("joint_ctl", joint_cb);

class JointSub{
	public:
		void begin(){
            nh.subscribe(link_sub);
        }
};

JointSub Joints;
#endif

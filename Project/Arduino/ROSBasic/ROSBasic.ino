

#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64MultiArray.h>

//General Debugging stuff
#define STATUS_LED 13

// Ros Initialization stuff
ros::NodeHandle  nh;
std_msgs::Float64MultiArray pub_msg; //out
//std_msgs::Empty pub_msg;

int blink = 1;
//Position Update Callback
void messageCb( const std_msgs::Float64MultiArray& toggle_msg){
  blink = 1-blink;
  digitalWrite(STATUS_LED, blink); 
}

ros::Subscriber<std_msgs::Float64MultiArray> sub("serial_link", &messageCb );

void setup() {
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  
  //Set up ROS node
  nh.initNode();
  nh.subscribe(sub);
}


void loop() {
  //spin ROS
  nh.spinOnce();
  delay(1);
}

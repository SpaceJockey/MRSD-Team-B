

#include <ros.h>
#include <sensor_msgs/JointState.h>

//General Debugging stuff
#define STATUS_LED 13

// Ros Initialization stuff
ros::NodeHandle  nh;
sensor_msgs::JointState pub_msg; //out

int blink = 1;
//Position Update Callback
void jointstate_cb(const sensor_msgs::JointState& cmd_msg){
  blink = 1-blink;
  digitalWrite(STATUS_LED, blink); 
}

ros::Subscriber<sensor_msgs::JointState> sub("serial_link", jointstate_cb);

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

/*
ROSListener.ino
Written by Nathaniel Chapman, 11/23/2013
MRSD Team B
This Sketch listens to the ROS channel "serial_link" and updates joint positions accordingly

ROS Joint configs are indexed so...
0	Center Swivel
1	Center Segment Front Pitch
2	Center Segment Rear Pitch
3	Front Segment Pitch
4	Rear Segment Pitch
5	Center Segment Front Prismatic
6	Center Segment Rear Prismatic

Servo Wiring Configs:
12	Center Swivel
2	Center Segment Front Pitch
10	Center Segment Rear Pitch
3	Front Segment Pitch
11	Rear Segment Pitch
0	Center Segment Front Prismatic
8	Center Segment Rear Prismatic
*/

#include <Arduino.h>
#include "SpaceJockey.h"

//ROS Stuff
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

//General Debugging stuff
#define STATUS_LED 13

// Ros Initialization stuff
ros::NodeHandle  nh;

std_msgs::Int32 batt_msg; //battery state output
std_msgs::String debug_msg;
ros::Publisher batt_state("battery_state", &batt_msg);
ros::Publisher debug_pub("arduino_debug", &debug_msg);

//Position Update Callback
int blink = 1;
void jointstate_cb(const std_msgs::Float64MultiArray& cmd_msg){
  //blink LED to indicate CB is running...
  blink = 1-blink;
  digitalWrite(STATUS_LED, blink);
  
  //
  for(int c = 0; c < cmd_msg.data_length; c++) setServoPos(c, cmd_msg.data[c]); //Update servos
}
ros::Subscriber<std_msgs::Float64MultiArray> serial_link("serial_link", jointstate_cb);

// Publish an incoming string to the debug topic
void debug(char* s){
       debug_msg.data = s;
       debug_pub.publish( &debug_msg );
}

void setup() {
  //Set up status LEDS
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  pinMode(BATT_LED, OUTPUT);
  digitalWrite(BATT_LED, LOW);
  
    //initialize digital servo board
  hv_servo.begin();  
  hv_servo.setPWMFreq(SERVO_HZ);
  //reset all joints to default until input is recieved from ROS
  for(int c = 0; c < 7; c++) setServoPos(c, 0.00);
  
  //Set up ROS node
  nh.initNode();
  nh.subscribe(serial_link);
  
  //Set up Battery Monitoring
  analogReadResolution(12);
  batt_mon = analogRead(BATT_PIN);
  nh.advertise(batt_state);
  
  //Start Battery monitoring loop
  Scheduler.startLoop(battLoop);
  
  debug("Space Jockey setup complete.");
}

//Main loop just spins ROS, everything else happens in its own loop using the Scheduler library
void loop() {  
  	//spin ROS
	nh.spinOnce();
	delay(1);
}

//update battery state every half second
void battLoop() {
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


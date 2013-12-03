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

*/

#include <Scheduler.h>

#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include <SpaceJockey.h>
#include <calibration/analogServo.h>

//General Debugging stuff
#define STATUS_LED 13

//Battery Monitoring LED
#define BATT_PIN A8    //pin for battery monitoring
#define BATT_LED 12    //pin for led indicator for battery

//Battery Calibration Values
#define BATT_6V   1700 //6V = 0%
#define BATT_85V  1870 //8.5V = 100%
int batt_mon;

// Ros Initialization stuff
ros::NodeHandle  nh;

std_msgs::Int32 batt_msg; //battery state output
ros::Publisher batt_state("battery_state", &batt_msg);

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

Adafruit_PWMServoDriver hv_servo = Adafruit_PWMServoDriver();
//Adafruit_PWMServoDriver lv_servo = Adafruit_PWMServoDriver(0x41);

//Radian values for us degree-centric folks
#define M_PI 3.14159265
#define DEG_20 (M_PI / 9)
#define DEG_30 (M_PI / 6)
#define DEG_45 (M_PI / 4)
//float degtorad(float deg){return (deg * M_PI) / 180.0;}



//These are the configuration values for the robot, they need to be tuned!
//These are in radians or meters, depending on joint types
const float real_min[] = {-DEG_30, -DEG_30, -DEG_30, -DEG_45, -DEG_45,   0,   0};
const float real_max[] = { DEG_30,  DEG_30,  DEG_30,  DEG_45,  DEG_45, .145, .145};

//Servo addresses for the HV servo board
const unsigned int hv_servo_addr[] = {12, 2, 10, 3, 11, 0, 8};

//To map things to our servo values
int servoMap(float value, unsigned int joint)
{return ((int) (((value - real_min[joint]) * ((float) (SERVOMAX - SERVOMIN)))  / (real_max[joint] - real_min[joint]))) + SERVOMIN;}

//Set the specified servo to the correct real-world position
void setServoPos(unsigned int joint, float value){
  if(joint > 6) return; //ignore unimplemented channels
  unsigned int sval = servoMap(value, joint);
  hv_servo.setPWM(hv_servo_addr[joint], 0, servoMap(value, joint));
}

void setup() {
  //Set up status LEDS
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  pinMode(BATT_LED, OUTPUT);
  digitalWrite(BATT_LED, LOW);
  
  //Set up ROS node
  //nh.getHardware()->setBaud(115200); // Up Baud Rate
  nh.initNode();
  nh.subscribe(serial_link);
  
  //Set up Battery Monitoring
  pinMode(BATT_LED, OUTPUT);
  analogReadResolution(12);
  batt_mon = analogRead(BATT_PIN);
  nh.advertise(batt_state);
  
  //Start Battery monitoring loop
  Scheduler.startLoop(battLoop);
  
  //initialize digital servo board
  hv_servo.begin();  
  hv_servo.setPWMFreq(SERVO_HZ);
  
  //reset all joints to default until input is recieved from ROS
  for(int c = 0; c < 7; c++) setServoPos(c, 0.00);
}

//Main loop just spins ROS, everything else happens in its own loop using the Scheduler library
void loop() {  
  	//spin ROS
	nh.spinOnce();
	yield(); //or delay(1)
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

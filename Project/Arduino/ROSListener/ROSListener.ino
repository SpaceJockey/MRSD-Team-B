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

#include <ros.h>
#include <sensor_msgs/JointState.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//General Debugging stuff
#define STATUS_LED 13

// Ros Initialization stuff
ros::NodeHandle  nh;

//Position Update Callback
int blink = 1;
void jointstate_cb(const sensor_msgs::JointState& cmd_msg){
  //blink LED to indicate CB is running...
  blink = 1-blink;
  digitalWrite(STATUS_LED, blink);
  
  for(int c = 0; c < cmd_msg.position_length; c++) setServoPos(c, cmd_msg.position[c]); //Update servos
}
ros::Subscriber<sensor_msgs::JointState> serial_link("serial_link", jointstate_cb);

Adafruit_PWMServoDriver hv_servo = Adafruit_PWMServoDriver();
//Adafruit_PWMServoDriver hv_servo = Adafruit_PWMServoDriver(0x41);

//Radian values for us degree-centric folks
#define M_PI 3.14159265
#define DEG_20 (M_PI / 9)
#define DEG_30 (M_PI / 6)
#define DEG_45 (M_PI / 4)
//float degtorad(float deg){return (deg * M_PI) / 180.0;}

#define DIG_SERVO_HZ  300 // Digital Servos operate at 300 Hz
#define ANA_SERVO_HZ  50 // Analog Servos operate at 50 Hz

#define SERVOMIN  185 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMID  307 // This is the Servo 'Middle Position'
#define SERVOMAX  430 // this is the 'maximum' pulse length count (out of 4096)
#define SERVORANGE ((float) (SERVOMAX - SERVOMIN))

//These are the configuration values for the robot, they need to be tuned!
//These are in radians or meters, depending on joint types
const float real_min[] = {-DEG_30, -DEG_30, -DEG_30, -DEG_45, -DEG_45,   0,   0};
const float real_max[] = { DEG_30,  DEG_30,  DEG_30,  DEG_45,  DEG_45, .145, .145};

//Servo addresses for the HV servo board
const unsigned int hv_servo_addr[] = {12, 4, 8, 5, 9, 0, 1};

//used only for linear joints where they have a limit switch
const unsigned int limit_pin[] = {0, 0, 0, 0, 0, 52, 53};

//To map things to our servo values
int servoMap(float value, unsigned int joint)
{return ((int) (((value - real_min[joint]) * SERVORANGE)  / (real_max[joint] - real_min[joint]))) + SERVOMIN;}

//Set the specified servo to the correct real-world position
void setServoPos(unsigned int joint, float value){
  if(joint > 6) return; //ignore unimplemented channels
  unsigned int sval = servoMap(value, joint);
  hv_servo.setPWM(hv_servo_addr[joint], 0, servoMap(value, joint));
}

void setup() {
  //Set up status LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  
  //Set up limit switch pins
  pinMode(limit_pin[5], INPUT);
  digitalWrite(limit_pin[5], HIGH); 
  pinMode(limit_pin[6], INPUT);
  digitalWrite(limit_pin[6], HIGH);
  
  //Set up ROS node
  //nh.getHardware()->setBaud(115200); // Up Baud Rate
  nh.initNode();
  nh.subscribe(serial_link);
  
  //initialize digital servo board
  hv_servo.begin();  
  hv_servo.setPWMFreq(ANA_SERVO_HZ);
    
}

void loop() {
	//spin ROS
	nh.spinOnce();
	delay(1);
}

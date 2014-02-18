/*
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

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//ROS Stuff
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>

/*Servo Calibration Values */
#define SERVO_HZ  300
#define SERVOMIN  1231 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  2869 // this is the 'maximum' pulse length count (out of 4096)

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

//Set I2C interface to 400KHz frequency
// Include Atmel CMSIS driver
//#include <include/twi.h>
//TWI_ConfigureMaster(WIRE_INTERFACE, 400000, VARIANT_MCK);

void setup(){
	/*
	//initialize digital servo board
	hv_servo.begin();  
	hv_servo.setPWMFreq(SERVO_HZ);
	//reset all joints to default until input is recieved from ROS
	for(int c = 0; c < 7; c++) setServoPos(c, 0.00);
	*/

	//nh.subscribe(serial_link);
}

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


class Body {
	public:
		Body();
		void begin();
	private:

}
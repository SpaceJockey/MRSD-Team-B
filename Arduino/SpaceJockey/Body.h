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


Adafruit_PWMServoDriver hv_servo = Adafruit_PWMServoDriver();
//Adafruit_PWMServoDriver lv_servo = Adafruit_PWMServoDriver(0x41);


class Body {
	public:
		Body();
		void begin();
		

		

	
	private:

}
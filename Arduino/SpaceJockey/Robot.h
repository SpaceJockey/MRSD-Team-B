#ifndef ROBOT_H
#define ROBOT_H
#include "SpaceJockey.h"

//Adafruit PWM Driver
#include <Arduino.h>
#include <include/twi.h> //Include Atmel CMSIS driver
#include <Adafruit_PWMServoDriver.h>

/* Radian values for us degree-centric folks */
#define M_PI 3.14159265
#define DEG_20 (M_PI / 9)
#define DEG_30 (M_PI / 6)
#define DEG_45 (M_PI / 4)
//float degtorad(float deg){return (deg * M_PI) / 180.0;}

/*Robot Configuration Parameters
TODO: investigate rolling these into some sort of ROS perameter server

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
0	Front Prismatic
8	Rear Prismatic

*/

//These are the configuration values for the robot, they need to be tuned!
//These are in radians or meters, depending on joint types
static const float real_min[] = {-DEG_30, -DEG_30, -DEG_30, -DEG_45, -DEG_45,   0,   0};
static const float real_max[] = { DEG_30,  DEG_30,  DEG_30,  DEG_45,  DEG_45, .145, .145};

//Servo addresses for the servo control board
static const unsigned int servo_addr[] = {12, 2, 10, 3, 11, 0, 8};

//Servo Calibration Values
#define SERVO_HZ  300
#define SERVOMIN  1231 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  2869 // this is the 'maximum' pulse length count (out of 4096)

class Robot {
	public:
		Robot(){
			//Initialize properties
			_servo_min = SERVOMIN;
			_servo_max = SERVOMAX;
			_servos = Adafruit_PWMServoDriver();
		}
		
		//Start up the servo board
		void begin(){
			//initialize digital servo board
			_servos.begin();  
			_servos.setPWMFreq(SERVO_HZ);
			//reset all joints to default
			for(int c = 0; c < 7; c++) this->setServoPos(c, 0.00);
		}
		
		//Set Arduino Due I2C Bus to 400 KHz
		void setHiSpeed(){
			TWI_ConfigureMaster(WIRE_INTERFACE, 400000, VARIANT_MCK);
		}
		
		//Calibrate the PWM to real-world output values to compensate for temperature-based clock drift
		void calibratePWM(); //TODO: pull this in from prototypes/PWMDigitalCalibrate
		
		//Set the specified servo to the correct real-world position
		void setServoPos(unsigned int joint, float value){
			if(joint > 6) return; //ignore unimplemented channels
			//Map angles to servo values
			unsigned int sval = ((int) (((value - real_min[joint]) * ((float) (_servo_max - _servo_min)))  / (real_max[joint] - real_min[joint]))) + _servo_min;
			_servos.setPWM(servo_addr[joint], 0, sval);
		}
	private:
		Adafruit_PWMServoDriver _servos;
		uint16_t _servo_min;
		uint16_t _servo_max;
};

Robot Servos;
#endif

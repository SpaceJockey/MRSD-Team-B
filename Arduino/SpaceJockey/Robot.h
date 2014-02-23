#ifndef ROBOT_H
#define ROBOT_H
#include "SpaceJockey.h"

//Adafruit PWM Driver
#include <Arduino.h>
#include <include/twi.h> //Include Atmel CMSIS driver
#include <Adafruit_PWMServoDriver.h>

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
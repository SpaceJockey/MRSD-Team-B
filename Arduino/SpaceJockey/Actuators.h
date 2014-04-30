#ifndef ACTUATORS_H
#define ACTUATORS_H
#include "SpaceJockey.h"

//Adafruit PWM Driver
#include <Arduino.h>
#include <include/twi.h> //Include Atmel CMSIS driver
#include <Adafruit_PWMServoDriver.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

//Servo Calibration Values
#define _SERVO_HZ  300
#define _SERVOMIN  1231 // this is the 'minimum' pulse length count (out of 4096)
#define _SERVOMAX  2869 // this is the 'maximum' pulse length count (out of 4096)

#define _SERVOMIN_US  900
#define _SERVOMID_US  1500
#define _SERVOMAX_US  2100

//these are 'pretty good' PWM values used to speed up calibration
#define _SERVOMIN_CLUE  1200
#define _SERVOMAX_CLUE  2700

#define _SERVO_FEEDBACK_CHAN 15

//Adafruit PWM driver
static Adafruit_PWMServoDriver _pwm_driver;

//Interrupt used for calibration and heatbeat monitoring
static volatile unsigned int _pulse_us = 0;
static unsigned int _servo_min = _SERVOMIN_CLUE;;
static unsigned int _servo_max = _SERVOMAX_CLUE;;

static void feedbackInt(){
	unsigned int currus = micros();
	static unsigned int startus;
	if(digitalRead(SERVO_FEEDBACK_PIN)) { //rising edge
		startus = currus;
	}else{
		_pulse_us = currus - startus;
	}
}

static int getPWMerror(int pulse, int target){
		_pwm_driver.setPWM(_SERVO_FEEDBACK_CHAN, 0, pulse); //should take 100us in fast mode
		attachInterrupt(SERVO_FEEDBACK_PIN, feedbackInt, CHANGE);
		delay(9); //should give enough time for the pulse width to change and 2 pulses to hit;
		detachInterrupt(SERVO_FEEDBACK_PIN);
		return target - _pulse_us;
}

static int calibratePWM(int target, int clue=_SERVOMID_US){
	int error = 1;
	int pulselen = clue;
	while (error != 0){
		pulselen = pulselen + error;
		error = getPWMerror(pulselen, target);
	}
	return pulselen;
}

static void servoLoop() {
	_pulse_us = 0;
	int error = getPWMerror(_servo_min, _SERVOMIN_US);
	error += getPWMerror(_servo_max, _SERVOMAX_US);
	if (abs(error) > 4){
		nh.logwarn("Servo Calibration Mismatch");
	}

	delay(1000);
}

//ROS subscriber
static void joint_cb(const std_msgs::Int16MultiArray& cmd_msg){
	//update servos
	for(int c = 0; c < cmd_msg.data_length; c++){
		if(c > 14) return; //ignore unimplemented and feedback channels
		int pos = cmd_msg.data[c];
		if(pos > _SERVOMAX_US) pos = _SERVOMAX_US;
		if(pos < _SERVOMIN_US) pos = _SERVOMIN_US;
		_pwm_driver.setPWM(c, 0, map(pos, _SERVOMIN_US, _SERVOMAX_US, _servo_min, _servo_max));
	}
}
static ros::Subscriber<std_msgs::Int16MultiArray> link_sub("joint_ctl", joint_cb);

class Actuators {
	public:
		Actuators(){}
		
		//Start up the servo board
		void begin(){
			//initialize digital servo board
			_pwm_driver.begin();  
			_pwm_driver.setPWMFreq(_SERVO_HZ);
			TWI_ConfigureMaster(WIRE_INTERFACE, 400000, VARIANT_MCK); //Set Arduino Due I2C Bus to 400 KHz

			//calibrate the min/max values
			_servo_min = calibratePWM(_SERVOMIN_US, _servo_min);
			_servo_max = calibratePWM(_SERVOMAX_US, _servo_max);

			//init all servos to mid-range
			for(int c = 0; c < 15; c++){
				_pwm_driver.setPWM(c, 0, (_servo_min + _servo_max)/2);
			}

			//start up ROS listener
			nh.subscribe(link_sub);

			//start up monitoring loop
			Scheduler.startLoop(servoLoop);
		}

};

Actuators Servos;
#endif

/* H-BRIDGE MOTOR DRIVER
*  MRSD Project Class Task 6
*  MRSD Team B
*/

#ifndef MOTOR_H
#define MOTOR_H

#import <Arduino.h>
#import "Stepper.h"
#import "Comtypes.h"

//share definitions with the Stepper motor for reusability
#define FORWARD  CW
#define BACKWARD CCW

#define Kp .4   // PID proportional control Gain
#define Kd 1    // PID Derivitave control gain


// PID loop time every 100ms
#define LOOPTIME 100                     



void init_motor();
void doEncoderA();
void doEncoderB();
void calcMotorData();
void setMotorSpeed(uint8_t spd);
void setMotorDir(uint8_t dir);

class MotorSpeedController : public OutputChannel
{
	public:
	void setValue(uint16_t val);
    void updateChannel();
    
    private:
    int speed_req;              //Speed Set Point
    int updatePID(int command, int targetValue, int currentValue);
    int last_error; // Last error
    unsigned long lastMilli;    // loop timing 
};

class MotorPositionController : public OutputChannel
{
    public:
	void setValue(uint16_t val);
    //void updateChannel();
    void update();
    
    //private:
    int degree_req; //Position Set Point
    int updatePID(int command, int targetValue, int currentValue);
    int PWM_val;          //pid control signal
    int last_error; // Last error
};

/* REMOVED FOR TEMPORARY SANITY
class Motor
{
	public:
	void setValue(uint16_t val);

	Motor(uint8_t en_pin, uint8_t l1_pin, uint8_t l2_pin);

	void setSpeed(uint8_t spd);
	void setDir(uint8_t dir);

	void enable();
	void coast();
	void brake();

	private:
	uint8_t _en_pin;
	uint8_t _l1_pin;
	uint8_t _l2_pin;
};

*/



#endif
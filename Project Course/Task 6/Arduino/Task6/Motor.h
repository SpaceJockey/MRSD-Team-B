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

class Motor : public OutputChannel
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

void set_m_dir(uint8_t dir);



#endif
/* STEPPER MOTOR DRIVER
*  MRSD Project Class Task 6
*  Nathaniel Chapman
*/

#ifndef STEPPER_H
#define STEPPER_H

#import <Arduino.h>
#import "Comtypes.h"

//Raw pins ATMega Stepper Pinout
//http://www.pighixxx.com/pgdev/Temp/ARDUINO_V2.png
//PORTD
#define _EN_PD  _BV(5)
#define MS1_PD  _BV(6)
#define MS2_PD  _BV(7)
//#define MS3_PD  _BV(7)    //Deprecated to save pins

//PORTB
#define STEP_PB _BV(0)
#define DIR_PB  _BV(1)

// Direction Presets
#define CW		DIR_PB
#define CCW		0x00

// Microstepping Resolution Presets
#define FULL_STEP      0x00
#define HALF_STEP      0x01
#define QUARTER_STEP   0x02
#define EIGTH_STEP     0x03
//#define SIXTEENTH_STEP 0x05 //Deprecated to save pins


// Control pulse delay
// according to the datasheet, this could be set to 1 uS
// but 10 suits me better
#define STEP_CTL_US	10

//This library is hard-wired, which is fast, but not too reusable... need to rework later

class Stepper : public OutputChannel
{
	public:
		void setValue(uint16_t val);
		
		//Stepper();
		void enable();
		void disable();
		
		void setStep(uint8_t stepsize);
		void setDir(uint8_t dir);

		void step();
	private:
};

#endif


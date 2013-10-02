
/* STEPPER MOTOR DRIVER
*  MRSD Project Class Task 6
*  Nathaniel Chapman
*/

#import "Stepper.h"
#import <Arduino.h>

//sets ths motor step size using the define Constants above
void set_step(uint8_t stepsize) { 
	PORTD = (PORTD & ~0xE0) | (stepsize << 5);
	delayMicroseconds(STEP_CTL_US);
}

void set_dir(uint8_t dir) {
	PORTB = (PORTB & ~DIR_PB) | dir;
	delayMicroseconds(STEP_CTL_US);
}

void step() {
	PORTB |= STEP_PB;
	delayMicroseconds(STEP_CTL_US);
	PORTB &= ~STEP_PB;
	delayMicroseconds(STEP_CTL_US); //so step() can be called continuously for maximum speed
}

void stepper_init() {
	//Default 0s on all MSX pins sets the device in FULL_STEP mode by default
	DDRD  |= _EN_PD  | MS1_PD  | MS2_PD  | MS3_PD;  //Set Port D ctl pins as outputs
	PORTD |= _EN_PD; 								//Disable device
	delayMicroseconds(STEP_CTL_US);
	DDRB  |= STEP_PB | DIR_PB;  					//Set Port B ctl pins as outputs
	PORTD &= ~_EN_PD; 								//Enable device
}

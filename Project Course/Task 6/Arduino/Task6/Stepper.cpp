/* STEPPER MOTOR DRIVER
*  MRSD Project Class Task 6
*  Nathaniel Chapman
*/

#import "Stepper.h"
#import <Arduino.h>

//sets ths motor step size using the constants in Stepper.h
void Stepper::set_step(uint8_t stepsize) { 
	PORTD = (PORTD & ~0xC0) | (stepsize << 6);
	delayMicroseconds(STEP_CTL_US);
}

//Default 0s on all MSX pins sets the device in FULL_STEP mode by default
void Stepper::set_dir(uint8_t dir) {
	PORTB = (PORTB & ~DIR_PB) | dir;
	delayMicroseconds(STEP_CTL_US);
}

//Rotate the motor 1 step
void Stepper::step() {
	PORTB |= STEP_PB;
	delayMicroseconds(STEP_CTL_US);
	PORTB &= ~STEP_PB;
	delayMicroseconds(STEP_CTL_US); //so step() can be called continuously for maximum speed
}

//This method doubles as an enable method, and an init method
void Stepper::enable() {
	DDRD  |= _EN_PD  | MS1_PD  | MS2_PD;  			//Set Port D ctl pins as outputs
	DDRB  |= STEP_PB | DIR_PB;  					//Set Port B ctl pins as outputs
	PORTD &= ~_EN_PD; 								//Enable device
}

void Stepper::disable() {
	PORTD |= _EN_PD; 								//Disable device
}

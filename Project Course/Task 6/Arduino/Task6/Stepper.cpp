/* STEPPER MOTOR DRIVER
*  MRSD Project Class Task 6
*  Nathaniel Chapman
*/

#import "Stepper.h"
#import <Arduino.h>

//sets ths speed from an input channel
//gets called once per loop
void Stepper::setValue(uint16_t val) {
    desiredPos = map(val, 0, 0xFFFF, -400, 400); //+- 400 steps roughly equals 360 degrees
    if (desiredPos != currPos) {
        if( desiredPos < currPos) {
            setDir(CCW);
            currPos -= 4;
        } else {
            setDir(CW);
            currPos += 4;
        }
        for(uint8_t  i = 0; i < 4; i++) step();
    }
} 

//sets ths motor step size using the constants in Stepper.h
void Stepper::setStep(uint8_t stepsize) { 
	PORTD = (PORTD & ~0xC0) | (stepsize << 6);
	delayMicroseconds(STEP_CTL_US);
}

//Default 0s on all MSX pins sets the device in FULL_STEP mode by default
void Stepper::setDir(uint8_t dir) {
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

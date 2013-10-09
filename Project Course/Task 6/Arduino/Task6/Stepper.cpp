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
        } else setDir(CW);
      
    }
    
    uint16_t ms = millis();
    if (ms >= STEP_DELAY_MS + this->lastMs) {
        this->step();
        if(currDir == CCW) {
            currPos--;
        } else {
            currPos++;
        }
        lastMs = ms;
    }
} 

//sets ths motor step size using the constants in Stepper.h
/*void Stepper::setStep(uint8_t stepsize) { 
	PORTD = (PORTD & ~0xC0) | (stepsize << 6);
	delayMicroseconds(STEP_CTL_US);
}*/

//Default 0s on all MSX pins sets the device in FULL_STEP mode by default
void Stepper::setDir(uint8_t dir) {
    if(dir){
        digitalWrite(DIR, HIGH);
    } else {
        digitalWrite(DIR, LOW);
    }
	//PORTB = (PORTB & ~DIR_PB) | dir;
	delayMicroseconds(STEP_CTL_US);
}

//Rotate the motor 1 step
void Stepper::step() {
	PORTB |= STEP_PB;
	delayMicroseconds(STEP_CTL_US);
	PORTB &= ~STEP_PB;
}

//This method doubles as an enable method, and an init method
void Stepper::enable() {
	//DDRD  |= _EN_PD  | MS1_PD  | MS2_PD;  			//Set Port D ctl pins as outputs
	DDRB  |= STEP_PB | DIR_PB;  					//Set Port B ctl pins as outputs
	//PORTD &= ~_EN_PD; 								//Enable device
}
/*
void Stepper::disable() {
	//PORTD |= _EN_PD; 								//Disable device
}*/

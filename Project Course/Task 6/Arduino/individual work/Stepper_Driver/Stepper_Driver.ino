
/* STEPPER MOTOR DRIVER
*  MRSD Project Class Task 6
*  Nathaniel Chapman
*/

#import <Arduino.h>

//Arduino Pinout
#define _EN  4
#define MS1  5
#define MS2  6
#define MS3  7
//RST ans SLP are tied together w/ a pullup resistor and unused to save pins
//#define _RST 8
//#define _SLP 9
#define STEP 8
#define DIR  9

//our trusty debugging LED
#define LED 13

//Raw ATMega Pinout
//http://www.pighixxx.com/pgdev/Temp/ARDUINO_V2.png
//PORTD
#define _EN_PD  _BV(4)
#define MS1_PD  _BV(5)
#define MS2_PD  _BV(6)
#define MS3_PD  _BV(7)

//PORTB
//#define _RST_PB _BV(0)
//#define _SLP_PB _BV(1)
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
#define SIXTEENTH_STEP 0x05

// Control pulse delay
// according to the datasheet, this could be set to 1 uS
// but 10 suits me better
#define STEP_CTL_US	10

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

void setup() {
	//Default 0s on all MSX pins sets the device in FULL_STEP mode by default
	DDRD  |= _EN_PD  | MS1_PD  | MS2_PD  | MS3_PD;  //Set Port D ctl pins as outputs
	PORTD |= _EN_PD; 								//Disable device
	delayMicroseconds(STEP_CTL_US);
	DDRB  |= STEP_PB | DIR_PB;  					//Set Port B ctl pins as outputs
	PORTD &= ~_EN_PD; 								//Enable device
	set_step(HALF_STEP); //Why Not?
	
	//signal Arduino reset (to watch for brownouts)
	pinMode(LED, OUTPUT);
	digitalWrite(LED, HIGH);
	delay(50);
	digitalWrite(LED, LOW);
}

void loop() {
	delay(10);
	step();
}

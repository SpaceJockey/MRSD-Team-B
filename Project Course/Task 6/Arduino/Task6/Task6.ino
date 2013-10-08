/* Task6.ino - Main sketch for Task 6
*  MRSD Team B
*  Project Class Task 6
*/

#import <Arduino.h>
#import "Pinout.h"
#import "Comtypes.h"


#define TEST_CH 8

InputChannel inChannel[16];
//steady triangle wave - Channel 8

//Status LED test channel - Channel 1
LEDChannel ledChan;

#include <Servo.h>
Servo servo1;

#import "Stepper.h"
Stepper stepper1;

#import "Motor.h"
Motor motor1(EN1, L11, L12);




void setup() {
	//signal Arduino reset (to watch for brownouts)
	pinMode(STATUS_LED, OUTPUT);
	digitalWrite(STATUS_LED, HIGH);
	delay(100);
	digitalWrite(STATUS_LED, LOW);
	
	//Start Serial
	Serial.begin(SERIAL_SPEED);
	
	stepper1.enable();
	
	//servo1.attach(SERVO1);
	ledChan.attachInput(&inChannel[TEST_CH]);
	stepper1.attachInput(&inChannel[TEST_CH]);
}

void loop() {
	//update input channels
	
	//Test Channel, this channel outputs a constant triangle wave
	uint16_t val = inChannel[TEST_CH].getValue();
	inChannel[TEST_CH].setValue(val + 0x0100);
	ledChan.updateChannel();
	stepper1.updateChannel();
	
	//motor1.setDir(CCW);
	
	//Spin the stepper motor...
	delay(20);
	stepper1.step();
}

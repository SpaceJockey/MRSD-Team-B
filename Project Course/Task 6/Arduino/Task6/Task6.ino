/* Task6.ino - Main sketch for Task 6
*  MRSD Team B
*  Project Class Task 6
*/

#import <Arduino.h>
#import "Pinout.h"
#import "Comtypes.h"


#include <Servo.h>
Servo servo1;

#import "Stepper.h"
Stepper stepper1;

#import "Motor.h"
//Motor motor1(EN1, L11, L12);


void setup() {
	//signal Arduino reset (to watch for brownouts)
	pinMode(STATUS_LED, OUTPUT);
	digitalWrite(STATUS_LED, HIGH);
	delay(50);
	digitalWrite(STATUS_LED, LOW);
	
	//Start Serial
	Serial.begin(SERIAL_SPEED);
	
	stepper1.enable();
	
	servo1.attach(SERVO1);
}

void loop() {
	//motor1.setDir(CCW);
	
	//Spin the stepper motor...
	delay(10);
	stepper1.step();
}

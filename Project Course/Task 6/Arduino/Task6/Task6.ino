/* Task6.ino - Main sketch for Task 6
*  MRSD Team B
*  Project Class Task 6
*/

#import <Arduino.h>
#import "Pinout.h"
#import "Stepper.h"


void setup() {
	stepper_init();
	
	//signal Arduino reset (to watch for brownouts)
	pinMode(FAULT_LED, OUTPUT);
	pinMode(STATUS_LED, OUTPUT);
	digitalWrite(FAULT_LED, HIGH);
	delay(50);
	digitalWrite(FAULT_LED, LOW);
	digitalWrite(STATUS_LED, HIGH);
}

void loop() {
	//Spin the stepper motor...
	delay(10);
	step();
}

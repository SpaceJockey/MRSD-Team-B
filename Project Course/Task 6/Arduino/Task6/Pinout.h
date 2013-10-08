/* Pinout.h - Arduino Pinout Header File
*  MRSD Team B
*  Project Class Task 6
*/

#ifndef PINOUT_H
#define PINOUT_H

//Use 9600 Baud default
#define SERIAL_SPEED 9600

//Sensor Pins
#define RANGEFINDER 	A0 //IR rangefinder	
#define POTENTIOMETER 	A1 //Potentiometer dial
#define PRESSURE 		A2 //Resistive Pressure sensors


//DC H-Bridge Motor Driver Pins
#define EN1   10 	//Enable pin for motor 1
#define L11	  11 	// Logic 1 for motor 1
#define L12    12 	// Logic 2 for motor 2

//Servo Motor Control Pin
#define SERVO1 13   // Servo pin

//Stepper Motor Control Pins
#define _EN   5    // Stepper ~EN line
#define MS1   6    // Stepper Microstep Lines
#define MS2   7
// #define MS3   7  //To save pins, we're dropping support for 16X microstepping
#define STEP  8
#define DIR   9

//our trusty debugging LEDs
#define STATUS_LED 3

#endif
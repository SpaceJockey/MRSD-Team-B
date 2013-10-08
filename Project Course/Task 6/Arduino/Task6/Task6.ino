/* Task6.ino - Main sketch for Task 6
*  MRSD Team B
*  Project Class Task 6
*/

#import <Arduino.h>
#import "Pinout.h"
#import "Servo.h" //local copy of servo library to fix Arduino Compile path issues
#import "Comtypes.h"
#import "Stepper.h"
#import "Motor.h"


//Channel definitions
#define IN_ZERO_CH 0 //Channel 0 is always 0 (like /dev/null)
#define IN_POT_CH 1
#define IN_PRESSURE_CH 2
#define IN_RANGE_CH 3

#define IN_TEST_CH 8 //steady triangle wave - Channel 8

//Input Channel Array
InputChannel inChannel[17];

//Setup output channels
LEDChannel led;
Motor motor1(EN1, L11, L12);
Stepper stepper1;
ServoChannel servo1;

//Setup Serial I/O stuff
Packet errorPacket;
Packet packetBuffer[2]; //statically-allocated packet buffer for retransmits
uint8_t packetIndex; 	//points to the current packet, packetIndex ^
#define lastPacketIndex (packetIndex ^ 0x01)
#define currPacket 		packetBuffer[packetIndex]
#define lastPacket 		packetBuffer[lastPacketIndex]

//Output Channel Array
#define OUT_ZERO_CH 0 //nothing connected to Channel 0
#define OUT_LED_CH 1
#define OUT_MOTOR_CH 2
#define OUT_STEPPER_CH 3
#define OUT_SERVO_CH 4
#define OUT_NUM_CHANS 5 //nothing connected to Channels 5-7

//Array is size 8 to match out IO packets, additional channels can easily be added in future
OutputChannel * outChannel[8] = {0, &led, &motor1, &stepper1, &servo1, 0, 0, 0}; 

void setup() {
	//signal Arduino reset (to watch for brownouts)
	pinMode(STATUS_LED, OUTPUT);
	digitalWrite(STATUS_LED, HIGH);
	delay(100);
	digitalWrite(STATUS_LED, LOW);
	
	//Start Serial
	Serial.begin(SERIAL_SPEED);
	
	//Initialize Packet data structures
	errorPacket.in_chan = 0xff;
	errorPacket.out_chan = 0xff;
	errorPacket.pack();
	
	//Init the stepper motor
	stepper1.enable();
	//Init the servo motor
	servo1.attach(SERVO1);
	
	outChannel[OUT_LED_CH]->attachInput(&inChannel[IN_TEST_CH]);
	outChannel[OUT_STEPPER_CH]->attachInput(&inChannel[IN_TEST_CH]);
	outChannel[OUT_SERVO_CH]->attachInput(&inChannel[IN_POT_CH]);
}

void loop() {
	//update input channels
		//analog input Channels
		inChannel[IN_POT_CH].setValue(map(analogRead(POTENTIOMETER), 30, 950, 0, 0xffff));
		inChannel[IN_PRESSURE_CH].set10BitValue(analogRead(PRESSURE));
		inChannel[IN_RANGE_CH].set10BitValue(analogRead(RANGEFINDER));

		//Dummy Test Channel (Constant Triangle Wave)
		inChannel[IN_TEST_CH].setValue(inChannel[IN_TEST_CH].getValue() + 0x0100);
	
	//Check Serial and parse packets
	//Serial.println(inChannel[IN_POT_CH].getValue());
	//TODO: Implement this!
	
	//Output sensor data
	for(int i = 1; i <= 8; i++) {
		currPacket.channel[i - 1] = inChannel[i].getValue();
	}
	currPacket.pack();
	currPacket.transmit();
	packetIndex = lastPacketIndex;
	
	//update output channels
	for(int i = 1; i < OUT_NUM_CHANS; i++) outChannel[i]->updateChannel();
	
	//motor1.setDir(CCW);
	
	//Spin the stepper motor...
	delay(20);
	stepper1.step();
}

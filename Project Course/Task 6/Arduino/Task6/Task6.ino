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

#define IN_HIGH_CH 7 //Always on - Channel 7
#define IN_TEST_CH 8 //steady triangle wave - Channel 8

//Input Channel Array
InputChannel inChannel[17];

//Button State
uint8_t lastButton;
uint16_t lastBounce;
#define BUTTON_BOUNCE_MS 100
uint8_t testMode;

//Setup output channels
LEDChannel led;
//MotorPositionController motorPos;
Stepper stepper1;
ServoChannel servo1;
NullChannel null1;

//Setup Serial I/O stuff
Packet errorPacket;
Packet packetBuffer[2]; //statically-allocated packet buffer for retransmits
uint8_t packetIndex; 	//points to the current packet, packetIndex ^
#define lastPacketIndex (packetIndex ^ 0x01)
#define currPacket 		packetBuffer[packetIndex]
#define lastPacket 		packetBuffer[lastPacketIndex]
#define PSIZE 			sizeof(Packet)
uint16_t lastPacketSent;
#define PACKET_RATE_MS 50

char recvChars[PSIZE + 1]; 
Packet recvPacket;

//Output Channel Array
#define OUT_ZERO_CH 0 //nothing connected to Channel 0
#define OUT_LED_CH 1
#define OUT_MOTOR_P_CH 2
#define OUT_MOTOR_V_CH 3
#define OUT_SERVO_CH 4
#define OUT_STEPPER_CH 5
#define OUT_NUM_CHANS 6 //nothing connected to Channels 6-7

//Array is size 8 to match out IO packets, additional channels can easily be added in future
OutputChannel * outChannel[8] = {&null1, &led, &null1, &null1, &servo1, &stepper1, &null1, &null1}; 

//Sensor smoothing variables
const int pNumReadings = 10;
int pReadings[pNumReadings];
int pindex = 0;
int ptotal = 0;
int paverage = 0;
const int rNumReadings = 10;
int rReadings[rNumReadings];
int rindex = 0;
int rtotal = 0;
int raverage = 0;


void setup() {
	//signal Arduino reset (to watch for brownouts)
	pinMode(STATUS_LED, OUTPUT);
	digitalWrite(STATUS_LED, HIGH);
	delay(100);
	digitalWrite(STATUS_LED, LOW);
	
	//Start Serial
	Serial.begin(SERIAL_SPEED);

	//Initialize Packet data structures
	errorPacket.inChannel = 0xff;
	errorPacket.outChannel = 0xff;
	errorPacket.pack();
	recvPacket.pack();
    
    //init the DC Motor
    initMotor();
	
	//Init the stepper motor
	stepper1.enable();
	//Init the servo motor
	servo1.attach(SERVO1);
	
	//initialize channels
	inChannel[IN_HIGH_CH].setValue(0xFFFF);
	inChannel[9].setValue(0x8000);
	
    //initialize output channels
	outChannel[OUT_LED_CH]->attachInput(&inChannel[IN_HIGH_CH]);
	outChannel[OUT_SERVO_CH]->attachInput(&inChannel[9]);
    outChannel[OUT_MOTOR_V_CH]->attachInput(&inChannel[10]);
    outChannel[OUT_MOTOR_P_CH]->attachInput(&inChannel[10]);
    outChannel[OUT_STEPPER_CH]->attachInput(&inChannel[11]);
    //outChannel[OUT_STEPPER_CH]->attachInput(&inChannel[IN_HIGH_CH]);

    for(int i = 0; i<pNumReadings; i++)
        pReadings[i] = 0;
    for(int i = 0; i<rNumReadings; i++)
        rReadings[i] = 0;
}

 // Button one switches states
void buttonPress(){
    uint16_t timeMs = (uint16_t) millis();
    if((timeMs - lastBounce) >= BUTTON_BOUNCE_MS){
        lastBounce = timeMs;
        if(testMode == 0){ //test mode
            testMode = 1;
            outChannel[OUT_LED_CH]->attachInput(&inChannel[IN_TEST_CH]);
            outChannel[OUT_SERVO_CH]->attachInput(&inChannel[IN_TEST_CH]);
            outChannel[OUT_MOTOR_V_CH]->attachInput(&inChannel[IN_TEST_CH]);
            outChannel[OUT_MOTOR_P_CH]->attachInput(&inChannel[IN_TEST_CH]);
            outChannel[OUT_STEPPER_CH]->attachInput(&inChannel[IN_TEST_CH]);
        } else { //default mode
            testMode = 0;
            outChannel[OUT_LED_CH]->attachInput(&inChannel[IN_HIGH_CH]);
            outChannel[OUT_SERVO_CH]->attachInput(&inChannel[9]);
            outChannel[OUT_MOTOR_V_CH]->attachInput(&inChannel[10]);
            outChannel[OUT_MOTOR_P_CH]->attachInput(&inChannel[10]);
            outChannel[OUT_STEPPER_CH]->attachInput(&inChannel[11]);
        }
    }
}

void loop() {
	//update input channels
    //should abstract this to the InputChannel Classes...
    ptotal = ptotal - pReadings[pindex];
    pReadings[pindex] = analogRead(POTENTIOMETER);
    ptotal = ptotal + pReadings[pindex];
    pindex = pindex+1;
    if (pindex>=pNumReadings)
        pindex = 0;
    paverage = ptotal/pNumReadings;
    
    inChannel[IN_POT_CH].setValue(map(paverage, 35, 944, 0, 0xffff));    
    inChannel[IN_PRESSURE_CH].set10BitValue(analogRead(PRESSURE));
    inChannel[IN_RANGE_CH].set10BitValue(analogRead(RANGEFINDER));

    //Dummy Test Channel (Constant Triangle Wave)
    inChannel[IN_TEST_CH].setValue(inChannel[IN_TEST_CH].getValue() + 0x0004);

    //Poll Button State
    uint8_t buttonState = digitalRead(IN_BUTTON);
    if(buttonState && !lastButton) buttonPress();
    lastButton = buttonState;
    
	//Check Serial and parse packets
	//TODO: add findUntil to fix dropped characters

	if (Serial.available() >= PSIZE) {
		Serial.readBytes((char *) recvPacket.chars, PSIZE);
		
		if( recvPacket.isValid()){
			if(recvPacket.isError()){ //recieved a request for retransmission
				lastPacket.transmit();
			}else{
				//Parse packet
				for(uint8_t i = 0; i < 8; i++) {
					inChannel[i + 9].setValue(recvPacket.channel[i]);
				}
				
				if(recvPacket.outChannel) {
					outChannel[recvPacket.outChannel]->attachInput(&inChannel[recvPacket.inChannel]);
				}
			}
		}else{
			//Got a bad packet, request re-transmit
			errorPacket.transmit();
		}
	}
	
	//Output sensor data
    uint16_t ms = millis();
    if(ms >= PACKET_RATE_MS + lastPacketSent) {
        for(int i = 1; i <= 8; i++) {
            currPacket.channel[i - 1] = inChannel[i].getValue();
        }
        
        currPacket.pack();
        currPacket.transmit();
        packetIndex = lastPacketIndex;
        lastPacketSent = ms;
    }

    
    
	//update output channels
	for(int i = 1; i < OUT_NUM_CHANS; i++) {
        outChannel[i]->updateChannel();
    }
}

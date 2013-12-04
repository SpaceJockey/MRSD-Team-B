#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMID  1900

#define FEEDBACK_CHAN 15
#define FEEDBACK_PIN 22

#define SERVO_CHAN 1

volatile unsigned long startus = 0;
volatile unsigned long pulseus = 0;
volatile bool gotPulse = false;

void readPulse(){
	unsigned long currus = micros();
	if(digitalRead(FEEDBACK_PIN)) { //rising edge
		startus = currus;
	}else{
		pulseus = currus - startus;
		gotPulse = true;
	}
}

int target = 900;
int error = 9999;
int lasterror = 9999;
int servomin = 0;
int servomax = 9999;

void setup() {

	pwm.begin();
	pwm.setPWMFreq(300);  // Digital servos run at 300 Hz updates
	attachInterrupt(FEEDBACK_PIN, readPulse, CHANGE);
	pwm.setPWM(FEEDBACK_CHAN, 0, SERVOMID);
	servomin = findTarget(900);
	servomax = findTarget(2100);
	detachInterrupt(FEEDBACK_PIN);

	
	Serial.begin(9600);
	Serial.println("Digital PWM calibration sketch");
	Serial.print("Minimum (900 us): ");
	Serial.println(servomin);
	Serial.print("Maximum (2100 us): ");
	Serial.println(servomax);
}

int findTarget(int tgt){
	int target = tgt;
	int error = 9999;
	int pulselen = SERVOMID;
	while (error != 0){
		pwm.setPWM(FEEDBACK_CHAN, 0, pulselen);
		//pwm.setPWM(SERVO_CHAN, 0, pulselen);
		for(int i = 0; i < 2; i++) { //wait two pulses
			while(gotPulse == false); //spin until pulse recieved
			gotPulse = false;
		}
		error = target - pulseus;
		pulselen = pulselen + ((error *3) / 4);
	}
	/*
	Serial.print("Found: ");
	Serial.print(pulselen); //print the pulse length
	Serial.print(" ==> ");
	Serial.println(pulseus); //print the pulse length
	*/
	return pulselen;
}


void loop() {
}

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMID  450

#define FEEDBACK_CHAN 15
#define FEEDBACK_PIN 22

#define SERVO_CHAN 1

bool targetReached = false;
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

void setup() {
	Serial.begin(9600);
	Serial.println("#define SERVO_HZ  60");

	pwm.begin();

	pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

	attachInterrupt(FEEDBACK_PIN, readPulse, CHANGE);
	pwm.setPWM(FEEDBACK_CHAN, 0, SERVOMID);
  
  	Serial.print("#define SERVOMIN ");
	findTarget(900);
	Serial.print("#define SERVOMAX ");
	findTarget(2100);
	Serial.print("#define SERVOMID ");
	findTarget(1500);
}

void findTarget(int tgt){
	int target = tgt;
	int error = 9999;
	int lastErrorAbs = 9999;
	int pulselen = SERVOMID;
	while (abs(error) > 1){
		pwm.setPWM(FEEDBACK_CHAN, 0, pulselen);
		//pwm.setPWM(SERVO_CHAN, 0, pulselen);
		for(int i = 0; i < 2; i++) { //wait two pulses
			while(gotPulse == false); //spin until pulse recieved
			gotPulse = false;
		}
		lastErrorAbs = abs(error);
		error = target - pulseus;
		
		//Serial.println(pulseus);
		
		if(error <= 0) {
			pulselen--;
		} else {
			pulselen++;
		}
	}
	//Serial.print("Found: ");
	Serial.println(pulselen); //print the pulse length
	//Serial.print(" ==> ");
	//Serial.println(pulseus); //print the pulse length
}


void loop() {
}

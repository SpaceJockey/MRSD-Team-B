#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN_US  900
#define SERVOMID_US  1500
#define SERVOMAX_US  2100

#define FEEDBACK_CHAN 15
#define FEEDBACK_PIN 22

//for testing
#define SERVO_CHAN 1

static volatile unsigned int pulseus = 0;


void readPulse(){
	unsigned int currus = micros();
	static unsigned int startus;
	if(digitalRead(FEEDBACK_PIN)) { //rising edge
		startus = currus;
	}else{
		pulseus = currus - startus;
	}
}

inline void attachInt(){attachInterrupt(FEEDBACK_PIN, readPulse, CHANGE);}
inline void detachInt(){detachInterrupt(FEEDBACK_PIN);}

//last calibrated values
int servomin = 1200;
int servomax = 2700;

int findTarget(int target, int clue=SERVOMID_US){
	int error = 1;
	int pulselen = clue;
	while (error != 0){
		pulselen = pulselen + error;
		pwm.setPWM(FEEDBACK_CHAN, 0, pulselen); //should take 100us in fast mode
		delay(6); //should give enough time for the pulse width to change and 2 pulses to hit;
		error = target - pulseus;
	}
	return pulselen;
}

void setup() {
	Serial.begin(9600);
	Serial.println("Digital PWM calibration sketch");

	pwm.begin();
	TWI_ConfigureMaster(WIRE_INTERFACE, 400000, VARIANT_MCK); //I2C Fast mode
	pwm.setPWMFreq(300);  // Digital servos run at 300 Hz updates
	pwm.setPWM(FEEDBACK_CHAN, 0, SERVOMID_US);
	attachInt();
	servomin = findTarget(SERVOMIN_US, servomin);
	servomax = findTarget(SERVOMAX_US, servomin);
	detachInt();

	Serial.print("Minimum (900 us): ");
	Serial.println(servomin);
	Serial.print("Maximum (2100 us): ");
	Serial.println(servomax);
}

int servoval = servomax;
//motor monitoring loop
void loop() {
	servoval = (servomax - servoval) + servomin;
	pwm.setPWM(SERVO_CHAN, 0, servoval);

	Serial.print("Checking calibration...");
	pulseus = 0;
	attachInt();
	pwm.setPWM(FEEDBACK_CHAN, 0, servomin);
	delay(6);
	int error = SERVOMIN_US - pulseus;
	pwm.setPWM(FEEDBACK_CHAN, 0, servomax);
	delay(6);
	error += SERVOMAX_US - pulseus;
	detachInt();
	if (abs(error) > 4){
		Serial.println("WARNING");
	}else{
		Serial.println("OK");
	}
	delay(1000);
}

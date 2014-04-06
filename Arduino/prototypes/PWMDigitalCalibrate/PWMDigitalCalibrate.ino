#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMID  1900

#define FEEDBACK_CHAN 15
#define FEEDBACK_PIN 22

#define SERVO_CHAN 1

//Need to be unsigned long on AVR devices
//static volatile unsigned int startus = 0;
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

void setup() {
	int servomin = 0;
	int servomax = 9999;

	pwm.begin();
	TWI_ConfigureMaster(WIRE_INTERFACE, 400000, VARIANT_MCK); //I2C Fast mode
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

int findTarget(int target){
	//int target = tgt;
	int error = 1;
	int pulselen = SERVOMID;
	while (error != 0){
		pulselen = pulselen + ((error *3) / 4);
		pwm.setPWM(FEEDBACK_CHAN, 0, pulselen); //should take 100us in fast mode
		delay(10); //should give enough time for the pulse width to change and 3 pulses to hit
		//while(!gotPulse) delay(1);
		error = target - pulseus;
	}
	return pulselen;
}


void loop() {
}

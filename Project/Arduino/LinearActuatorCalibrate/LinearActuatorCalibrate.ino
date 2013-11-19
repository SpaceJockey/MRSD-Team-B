/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 16 servos, one after the other

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  250 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  450 // this is the 'maximum' pulse length count (out of 4096)

#define SERVO_HZ  60 // Digital Servos operate at 300 Hz

#define SOFT_MIN 350
#define RANGE 190

#define LIN_SERVO_1 0
#define LIN_SERVO_2 1

#define LIMIT_SWITCH_1 52
#define LIMIT_SWITCH_2 53

#define ROT_SERVO_1 4
#define ROT_SERVO_2 5

// our servo # counter
uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("2 Channel Linear Actuator Calibration Test!");

  //Actuator 1
  pinMode(LIMIT_SWITCH_1, INPUT);
  digitalWrite(LIMIT_SWITCH_1, HIGH); //enable internal pullup on the limit switch.
  
  pwm.begin();  
  pwm.setPWMFreq(SERVO_HZ);
  
  int servoval = SOFT_MIN;
  Serial.print("Software Actuator 1 Maximum: ");
  Serial.println(servoval);
	
  pwm.setPWM(LIN_SERVO_1, 0, servoval);
  delay(3000);
  Serial.println("Calibrating...");
  
  for (; digitalRead(LIMIT_SWITCH_1); servoval++){
	 pwm.setPWM(LIN_SERVO_1, 0, servoval);
	 delay(50);
	}
	
	Serial.print("Actuator 1 Minimum: ");
	Serial.println(servoval);
	
	Serial.print("Actuator 1 Est. Maximum: ");
	Serial.println(servoval - RANGE);
	 pwm.setPWM(LIN_SERVO_1, 0, servoval - RANGE);
	
  //Actuator 2
  pinMode(LIMIT_SWITCH_2, INPUT);
  digitalWrite(LIMIT_SWITCH_2, HIGH); //enable internal pullup on the limit switch.
  
  pwm.begin();  
  pwm.setPWMFreq(SERVO_HZ);
  
  servoval = SOFT_MIN;
  Serial.print("Software Actuator 2 Maximum: ");
  Serial.println(servoval);
	
  pwm.setPWM(LIN_SERVO_2, 0, servoval);
  delay(3000);
  Serial.println("Calibrating...");
  
  for (; digitalRead(LIMIT_SWITCH_2); servoval++){
	 pwm.setPWM(LIN_SERVO_2, 0, servoval);
	 delay(50);
	}
	
	Serial.print("Actuator 2 Minimum: ");
	Serial.println(servoval);
	
	Serial.print("Actuator 2 Est. Maximum: ");
	Serial.println(servoval - RANGE);
	pwm.setPWM(LIN_SERVO_2, 0, servoval - RANGE);
}


void loop() {
  Serial.println("Left!");
  pwm.setPWM(ROT_SERVO_1, 0, SERVOMAX);
  pwm.setPWM(ROT_SERVO_2, 0, SERVOMIN);
  delay(1000);
  
  Serial.println("Right!");
  pwm.setPWM(ROT_SERVO_1, 0, SERVOMIN);
  pwm.setPWM(ROT_SERVO_2, 0, SERVOMAX);
  delay(1000);
}

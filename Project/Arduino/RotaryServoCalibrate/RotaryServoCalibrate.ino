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

#define ROT_SERVO_1 4
#define ROT_SERVO_2 5

void setup() {
  Serial.begin(9600);
  Serial.println("2 Channel Rotary Actuator Calibration Test!");

  pwm.begin();  
  pwm.setPWMFreq(SERVO_HZ);
 
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

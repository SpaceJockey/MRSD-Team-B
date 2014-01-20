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

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMID  375 //This is the Servo 'Middle Position'
#define SERVO_HZ  60 // Digital Servos operate at 300 Hz

void setup() {
  Serial.begin(9600); //Don't know why this is necessary...
  
  pwm.begin();  
  pwm.setPWMFreq(SERVO_HZ);
 
}


void loop() {
  for(int srv = 2; srv < 16; srv++){ 
	pwm.setPWM(srv, 0, SERVOMID);
  }
  delay(1000);
  /*
  for(int srv = 2; srv < 16; srv++){ 
	pwm.setPWM(srv, 0, 275);
  }
  delay(1000);
  */
}

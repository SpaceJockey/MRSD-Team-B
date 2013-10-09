#import <Arduino.h>
#import "Motor.h"
#import "Pinout.h"

void setup() {
    Serial.begin(9600);
    initMotor();
    digitalWrite(EN1, HIGH);
    digitalWrite(L11, HIGH);
}

void loop() {
    calcMotorData();
    Serial.print(encoderPosition);
    Serial.print(" : ");
    Serial.print(motorSpeedCurr); // current speed
    Serial.print(" : ");
    Serial.println(motorDegCurr);   // current position       
}
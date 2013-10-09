#import <Arduino.h>
#import "../Task6/Motor.h"
#import "../Task6/Pinout.h"

void setup() {
    Serial.begin(9600);
    initMotor();
    digitalWrite(EN1, HIGH);
    digitalWrite(L11, HIGH);
}

void loop() {
    calcMotorData();
    print(encoderPosition);
    print(" : ");
    print(motorSpeedCurr); // current speed
    print(" : ");
    println(motorDegCurr);   // current position       
}
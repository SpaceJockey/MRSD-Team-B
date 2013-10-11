/* H-BRIDGE MOTOR DRIVER
*  MRSD Project Class Task 6
*  MRSD Team B
*/

#import "Motor.h"
#import <Arduino.h>
#import "Pinout.h"

void initMotor() {
    pinMode(EN1, OUTPUT);
    pinMode(L11, OUTPUT);
    pinMode(L12, OUTPUT);
    pinMode(ENCODER_A, INPUT); 
    pinMode(ENCODER_B, INPUT);
  
    // attach encoder interrupts
    attachInterrupt(0, doEncoderA, CHANGE);
    attachInterrupt(1, doEncoderB, CHANGE); 
}

// XOR interrupt values to read quadrature encoder values
void doEncoderA(){
  if(digitalRead(ENCODER_A) ^ digitalRead(ENCODER_B)) {
    encoderPosition++;
  } else {
    encoderPosition--;
  }
}

void doEncoderB(){
    if(digitalRead(ENCODER_A) ^ digitalRead(ENCODER_B)) {
    encoderPosition--;
    } else {
    encoderPosition++;
  }
}

void calcMotorData() {                                                        // calculate speed
    motorDegCurr = encoderPosition;
    if((millis()-lastMilli) >= LOOPTIME)   {                                    // enter timed loop
        lastMilli = millis();
        motorSpeedCurr = ((encoderPosition - lastPosition) *(60* (1000/LOOPTIME)))/(360);          // 360 pulse each revolution
        lastPosition = encoderPosition;                
    }        
}

void setMotorSpeed(uint8_t spd){
	analogWrite(EN1, spd);
}

void setMotorDir(uint8_t dir){
  if(dir == FORWARD) {
    digitalWrite(L11, LOW);
    digitalWrite(L12, HIGH);
  }else {
    digitalWrite(L11, HIGH);
    digitalWrite(L12, LOW);
  }
}

void MotorPositionController::setValue(uint16_t val){
    degree_req = map(val, 0, 0xffff, 0, 720);
}

void MotorPositionController::updateChannel(){
    this->setValue(this->source->getValue());
    calcMotorData();                                                         // calculate speed
    PWM_val = updatePID(PWM_val, degree_req, motorDegCurr);     // compute PWM value
    setMotorSpeed(PWM_val);                                               // send PWM to motor

}

int MotorPositionController::updatePID(int command, int targetValue, int currentValue) {  // compute PWM value
    int error = abs(targetValue) - abs(currentValue); 
    if (error <0) {
        setMotorDir(BACKWARD);
        error = error * -1;
    } else {
        setMotorDir(FORWARD);
    }


    float pidTerm = (Kp * error) + (Kd * (error - last_error));                    
    last_error = error;
    return constrain(command + int(pidTerm), 0, 255);
}
/* H-BRIDGE MOTOR DRIVER
*  MRSD Project Class Task 6
*  MRSD Team B
*/

#import "Motor.h"
#import <Arduino.h>
#import "Pinout.h"

volatile long encoderPosition = 0;

int motorSpeedCurr;                           // current speed
int motorDegCurr;                             // current position 

static long last_count = 0;                       // last count

/* motor logic
Enable L1 L2 Result
L L L OFF 
L L H OFF 
L H L OFF 
L H H OFF
H L L BRAKE
H L H FORWARD 
H H L BACKWARD 
H H H BRAKE
H L L BRAKE 
H PWM H FWD-SPD 
H PWM L BCK-SPD 
H H H BRAKE
*/

void init_motor() {
    pinMode(EN1, OUTPUT);
    pinMode(L11, OUTPUT);
    pinMode(L12, OUTPUT);
    pinMode(ENCODER_A, INPUT); 
    pinMode(ENCODER_B, INPUT);
  
    // encoder pin on interrupt 0 (pin 2)
    attachInterrupt(0, doEncoderA, CHANGE);
    // encoder pin on interrupt 1 (pin 3)
    attachInterrupt(1, doEncoderB, CHANGE); 
}

// function to read encoder in interrupt
// function to read encoder in interrupt
void doEncoderA(){
  // look for a low-to-high on channel A
  if (digitalRead(ENCODER_A) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_B) == LOW) {  
      encoderPosition = encoderPosition + 1;         // CW
    } else {
      encoderPosition = encoderPosition - 1;         // CCW
    }
  } else {  // must be a high-to-low edge on channel A                                       
    // check channel B to see which way encoder is turning  
    if (digitalRead(ENCODER_B) == HIGH) {   
      encoderPosition = encoderPosition + 1;          // CW
    } else {
      encoderPosition = encoderPosition - 1;          // CCW
    }
  }
}

void doEncoderB(){

  // look for a low-to-high on channel B
  if (digitalRead(ENCODER_B) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(ENCODER_A) == HIGH) {  
      encoderPosition++;         // CW
    } 
    else {
      encoderPosition--;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(ENCODER_A) == LOW) {   
      encoderPosition++;          // CW
    } 
    else {
      encoderPosition--;          // CCW
    }
  }
}

void calcMotorData() {                                                        // calculate speed
    motorDegCurr = (encoderPosition) ;
    motorSpeedCurr = ((encoderPosition - last_count)*(60*(1000/LOOPTIME)))/(360);          // 360 pulse each revolution
    last_count = encoderPosition;                 
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

void MotorPositionController::update(){
    calcMotorData();                                                         // calculate speed
    PWM_val = updatePID(PWM_val, degree_req, motorDegCurr);     // compute PWM value
    analogWrite(EN1, PWM_val);                                               // send PWM to motor

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
// Pin assignment for motor
#define ENCODER_A 2 // encoder channel A
#define ENCODER_B 3 // encoder channel B
#define EN1  10     //enable pin Motor DC, connected to PWM controlto control speed
#define L11   11   // Logic input 1 for motr DC
#define L12   12 // Logic input 2 for motor DC

#define FORWARD  1
#define BACKWARD 0

int PWM_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
float Kp =   .2;                                // PID proportional control Gain
float Kd =    1;                                // PID Derivitave control gain
long encoderPosition = 0;                
static int last_error=0;                        // Last error
static long last_count = 0;                       // last count

// variable for motor position
int degree_req = 0;                             // speed (Set Point)
int motorDegCurr = 0;                             // speed (actual value)



////////////////////////////////////////// SETUP PINS////////////////////////////////////////////
void setup() 
{
// Setup for the the motor controller
  pinMode(EN1, OUTPUT);
  pinMode(L11, OUTPUT);
  pinMode(L12, OUTPUT);
  pinMode(ENCODER_A, INPUT); 
  pinMode(ENCODER_B, INPUT); 
  
// encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);

// encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, doEncoderB, CHANGE); 
// serial
  Serial.begin (9600);
   
  // initial of PWM
  analogWrite(EN1, PWM_val);

}
////////////////////////////////////////// SETUP PINS END////////////////////////////////////////////


// function to read encoder in interrupt
void doEncoderA(){

  // look for a low-to-high on channel A
  if (digitalRead(ENCODER_A) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_B) == LOW) {  
      encoderPosition = encoderPosition + 1;         // CW
    } 
    else {
      encoderPosition = encoderPosition - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(ENCODER_B) == HIGH) {   
      encoderPosition = encoderPosition + 1;          // CW
    } 
    else {
      encoderPosition = encoderPosition - 1;          // CCW
    }
  }

}

void doEncoderB(){

  // look for a low-to-high on channel B
  if (digitalRead(ENCODER_B) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(ENCODER_A) == HIGH) {  
      encoderPosition = encoderPosition + 1;         // CW
    } 
    else {
      encoderPosition = encoderPosition - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(ENCODER_A) == LOW) {   
      encoderPosition = encoderPosition + 1;          // CW
    } 
    else {
      encoderPosition = encoderPosition - 1;          // CCW
    }
  }
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


void calcMotorData() {                                                        // calculate speed
    motorDegCurr = (encoderPosition) ;
    //motorSpeedCurr = ((encoderPosition - last_count)*(60*(1000/LOOPTIME)))/(360);          // 360 pulse each revolution
    last_count = encoderPosition;                 
}

int updatePid_position(int command, int targetValue, int currentValue)   {             // compute PWM value
                             
  int error = abs(targetValue) - abs(currentValue); 
  if (error <0)
  {
    setMotorDir(BACKWARD);
    error = error * -1;
  }
  else 
  {
    setMotorDir(FORWARD);
  }


  float pidTerm = (Kp * error) + (Kd * (error - last_error));                    
  last_error = error;
  return constrain(command + int(pidTerm), 0, 255);

}

void motor_position(int degree_desired)
{

    calcMotorData();                                                          // calculate speed
    PWM_val = updatePid_position(PWM_val, degree_desired, motorDegCurr);               // compute PWM value
    analogWrite(EN1, PWM_val);                                               // send PWM to motor

}

//////////////////////////////////
void loop(){ 
  motor_position(120);
}



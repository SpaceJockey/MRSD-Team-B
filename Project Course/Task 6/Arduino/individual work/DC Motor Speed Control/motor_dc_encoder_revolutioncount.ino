// Pin assignment for motor
#define encoder0PinA 2 // encoder channel A
#define encoder0PinB 3 // encoder channel B
#define En1  10     //enable pin Motor DC, connected to PWM controlto control speed
#define L1   11   // Logic input 1 for motr DC
#define L2   12 // Logic input 2 for motor DC
#define LOOPTIME        100                     // PID loop time every 100ms
unsigned long lastMilli = 0;                    // loop timing 
unsigned long lastMilliPrint = 0;               // loop timing
int speed_req = 0;                             // speed (Set Point)
int speed_act = 0;                              // speed (actual value)
int PWM_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
volatile long count = 0;                        // rev counter
float Kp =   .4;                                // PID proportional control Gain
float Kd =    1;                                // PID Derivitave control gain
long encoder0Pos = 0;
int degree = 0 ;                                // degree of DC motor shaft
int revolution = 0 ;                            // how many times the revolutions
float pidTerm = 0;                              // PID correction
int error=0;                                    // error calculation = target value- actual value                     
static int last_error=0;                        // Last error
static long countAnt = 0;                                                 // last count


////////////////////////////////////////// SETUP PINS////////////////////////////////////////////
void setup() 
{
// Setup for the the motor controller
  pinMode(En1, OUTPUT);
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(encoder0PinA, INPUT); 
  pinMode(encoder0PinB, INPUT); 
  
// encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);

// encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, doEncoderB, CHANGE); 
// serial
  Serial.begin (9600);
   
  // initial of PWM
  analogWrite(En1, PWM_val);

}
////////////////////////////////////////// SETUP PINS END////////////////////////////////////////////


// function to read encoder in interrupt
void doEncoderA(){

  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }

}

void doEncoderB(){

  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinA) == LOW) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
}


//
void motor_dc(char c)
{
  if(c=='f') // forward
  {
    digitalWrite(L1, LOW);
    digitalWrite(L2, HIGH);
  }
  else if(c=='b') //backward
  {
    digitalWrite(L1, HIGH);
    digitalWrite(L2, LOW);
  }
}


void getMotorData()  {                                                        // calculate speed

  speed_act = ((encoder0Pos - countAnt)*(60*(1000/LOOPTIME)))/(180);          // 180 pulse each revolution
  countAnt = encoder0Pos;                 
}

int updatePid(int command, int targetValue, int currentValue)   {             // compute PWM value
                             
  error = abs(targetValue) - abs(currentValue); 
  pidTerm = (Kp * error) + (Kd * (error - last_error));                            
  last_error = error;
  return constrain(command + int(pidTerm), 0, 255);

}

//////////////////////////////////
void loop(){ //Do stuff here
  degree = (encoder0Pos*2)%360 ;
  revolution = encoder0Pos/180 ;
  speed_req = 150; // input speed required. The max speed is only 60rpm
  
  // if the speed required is negative, then the motor rotate backwards
  if (speed_req < 0)
  {
    motor_dc('b');
  }
  else {motor_dc('f');}
  
// getting the speed data every LOOPTIME  
  if((millis()-lastMilli) >= LOOPTIME)   {                                    // enter timed loop
    lastMilli = millis();
    getMotorData();                                                           // calculate speed
    PWM_val= updatePid(PWM_val, speed_req, speed_act);                        // compute PWM value
    analogWrite(En1, PWM_val);                                               // send PWM to motor
  }
    
  Serial.print("Speed RPM: ");
  Serial.println(speed_act);
}



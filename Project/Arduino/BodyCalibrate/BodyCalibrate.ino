

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver hv_servo = Adafruit_PWMServoDriver();

//for convenience, let's overload map to accept float params for our radian values from ROS
long map(float x, float in_min, float in_max, long out_min, long out_max)
{
  return ((long) (x - in_min) * ((float) (out_max - out_min)) / (in_max - in_min)) + out_min;
}

//Radian values for us degree-centric folks
#define M_PI 3.14159265
#define DEG_20 (M_PI / 9)
#define DEG_30 (M_PI / 6)

/* joints are as follows:
0	Center Swivel
1	Center Segment Front Pitch
2	Center Segment Rear Pitch
3	Front Segment Pitch
4	Rear Segment Pitch
5	Center Segment Front Prismatic
6	Center Segment Rear Prismatic
*/

#define SERVO_HZ  60 // Digital Servos operate at 300 Hz

#define SERVOMIN  250 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMID  375 // This is the Servo 'Middle Position'
#define SERVOMAX  450 // this is the 'maximum' pulse length count (out of 4096)

//Linear servo calibration defaults
#define LIN_SOFT_MIN 350
#define LIN_RANGE 190

//These are the configuration values for the robot, they need to be tuned!
//These are in radians or meters, depending on joint types
const float real_min[] = { DEG_20,  DEG_30,  DEG_30,  DEG_30,  DEG_30,   0,   0};
const float real_max[] = {-DEG_20, -DEG_30, -DEG_30, -DEG_30, -DEG_30, .145, .145};

//Linear values are not configured yet...
long servo_min[] = {SERVOMIN, SERVOMIN, SERVOMIN, SERVOMIN, SERVOMIN, LIN_SOFT_MIN + 100, LIN_SOFT_MIN + 100};
long servo_max[] = {SERVOMAX, SERVOMAX, SERVOMAX, SERVOMAX, SERVOMAX, LIN_SOFT_MIN, LIN_SOFT_MIN};

const unsigned int servo_addr[] = {12, 4, 8, 5, 9, 0, 1};

//used only for linear joints where they have a limit switch
const unsigned int limit_pin[] = {0, 0, 0, 0, 0, 52, 53};


void calibrate_linear(unsigned int joint){
 if(joint < 5 || joint > 6) return; //only work on the linear joints...

 int servoval = servo_min[joint];
  Serial.print("Linear joint ");
  Serial.print(joint);
  Serial.println(" Calibrating...");
	
  hv_servo.setPWM(servo_addr[joint], 0, servoval);
  delay(3000);
  
  for (; digitalRead(limit_pin[joint]); servoval++){
    hv_servo.setPWM(servo_addr[joint], 0, servoval);
	delay(50);
  }
	Serial.print("Minimum: ");
	servo_min[joint] = servoval;
	Serial.println(servo_min[joint]);
	Serial.print("Est. Maximum: ");
	servo_max[joint] = servoval - LIN_RANGE;
	Serial.println(servo_max[joint]);
	
	hv_servo.setPWM(servo_addr[joint], 0, servo_max[joint]);
}

void setup() {

  //Set up Servo Driver
  hv_servo.begin();  
  hv_servo.setPWMFreq(SERVO_HZ);
  
  //Initialize Serial
  Serial.begin(9600);
  Serial.println("Space Jockey Robot Actuator Calibration!");

  //Set up limit switches
  pinMode(limit_pin[5], INPUT);
  digitalWrite(limit_pin[5], HIGH); 
  pinMode(limit_pin[6], INPUT);
  digitalWrite(limit_pin[6], HIGH);
  

  
  //Set Rotary joints to middle range
  for(int srv = 2; srv < 16; srv++){ 
	hv_servo.setPWM(srv, 0, SERVOMID);
  }
  
  //wait 3 seconds...
  delay(3000);
  
  //Calibrate Front Prismatic Joint
  hv_servo.setPWM(servo_addr[1], 0, SERVOMID + 100);
  calibrate_linear(5);
  delay(1000);
  hv_servo.setPWM(servo_addr[5], 0, servo_min[5]);
  delay(2000);
  hv_servo.setPWM(servo_addr[1], 0, SERVOMID);
  
  //Calibrate Rear Prismatic Joint
  hv_servo.setPWM(servo_addr[2], 0, SERVOMID + 100);
  calibrate_linear(6);
  delay(1000);
  hv_servo.setPWM(servo_addr[6], 0, servo_min[6]);
  delay(2000);
  
  //Wag your tail
  hv_servo.setPWM(servo_addr[0], 0, servo_min[0]);
  delay(1000);
  hv_servo.setPWM(servo_addr[0], 0, servo_max[0]);
  delay(1000);
  hv_servo.setPWM(servo_addr[0], 0, SERVOMID);
  
  hv_servo.setPWM(servo_addr[2], 0, SERVOMID);
}


void loop() {
	//Do nothing!
}

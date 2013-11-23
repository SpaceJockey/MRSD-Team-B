

#include <ros.h>
#include <sensor_msgs/JointState.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//General Debugging stuff
#define STATUS_LED 13

// Ros Initialization stuff
ros::NodeHandle  nh;
sensor_msgs::JointState pub_msg; //out

//ros::Publisher left_encoder_pub("left_encoder", &l_enc_msg);
//ros::Publisher right_encoder_pub("right_encoder", &r_enc_msg);

int blink = 1;
//Position Update Callback
void jointstate_cb(const sensor_msgs::JointState& cmd_msg){
  blink = 1-blink;
  digitalWrite(STATUS_LED, blink);
  //servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
  //digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}


ros::Subscriber<sensor_msgs::JointState> sub("serial_link", jointstate_cb);


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

float degtorad(float deg){
  return (deg * M_PI) / 180.0;
}

/* joints are as follows:
0	Center Swivel
1	Center Segment Front Pitch
2	Center Segment Rear Pitch
3	Front Segment Pitch
4	Rear Segment Pitch
5	Center Segment Front Prismatic
6	Center Segment Rear Prismatic
*/

#define DIG_SERVO_HZ  300 // Digital Servos operate at 300 Hz
#define ANA_SERVO_HZ  50 // Analog Servos operate at 50 Hz

#define SERVOMIN  170 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMID  335 // This is the Servo 'Middle Position'
#define SERVOMAX  500 // this is the 'maximum' pulse length count (out of 4096)

//Linear servo calibration defaults
#define LIN_SOFT_MIN 350
#define LIN_RANGE 190

//These are the configuration values for the robot, they need to be tuned!
//These are in radians or meters, depending on joint types
const float real_min[] = { DEG_20,  DEG_20,  DEG_20,  DEG_30,  DEG_20,   0,   0};
const float real_max[] = {-DEG_20, -DEG_20, -DEG_20, -DEG_30, -DEG_20, .145, .145};

//Linear values are not configured yet...
long servo_min[] = {SERVOMIN, SERVOMIN, SERVOMIN, SERVOMAX, SERVOMAX - 100, LIN_SOFT_MIN + 100, LIN_SOFT_MIN + 100};
long servo_max[] = {SERVOMAX, SERVOMAX, SERVOMAX, SERVOMIN, SERVOMIN + 100, LIN_SOFT_MIN, LIN_SOFT_MIN};

const unsigned int servo_addr[] = {12, 4, 8, 5, 9, 0, 1};

//used only for linear joints where they have a limit switch
const unsigned int limit_pin[] = {0, 0, 0, 0, 0, 52, 53};

void set_rot(unsigned int joint, float radians){
 if(joint > 4) return; //only work on the rotary joints...
 
 int sval = map(radians, real_min[joint], real_max[joint], servo_min[joint], servo_max[joint]);
 hv_servo.setPWM(servo_addr[joint], 0, sval);
}

/*


void calibrate_linear(unsigned int joint){
 if(joint < 5 || joint > 6) return; //only work on the linear joints...

 int servoval = servo_min[joint];
  //Serial.print("Linear joint ");
  //Serial.print(joint);
  //Serial.println(" Calibrating...");
	
  hv_servo.setPWM(servo_addr[joint], 0, servoval);
  delay(3000);
  
  for (; digitalRead(limit_pin[joint]); servoval++){
    hv_servo.setPWM(servo_addr[joint], 0, servoval);
	delay(50);
  }
	//Serial.print("Minimum: ");
	servo_min[joint] = servoval;
	//Serial.println(servo_min[joint]);
	//Serial.print("Est. Maximum: ");
	servo_max[joint] = servoval - LIN_RANGE;
	//Serial.println(servo_max[joint]);
	
	hv_servo.setPWM(servo_addr[joint], 0, servo_max[joint]);
}
*/

void setup() {
  //Serial.begin(9600);
  //Serial.println("Space Jockey Robot Actuator Calibration!");
  
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  
  //Set up ROS node
  nh.getHardware()->setBaud(115200); // Up Baud Rate
  nh.initNode();
  nh.subscribe(sub);

  //Set up limit switches
  pinMode(limit_pin[5], INPUT);
  digitalWrite(limit_pin[5], HIGH); 
  pinMode(limit_pin[6], INPUT);
  digitalWrite(limit_pin[6], HIGH);
  
  //initialize digital servo board
  hv_servo.begin();  
  hv_servo.setPWMFreq(ANA_SERVO_HZ);
  
  //Set Rotary joints to middle range
  /*
  for(int srv = 2; srv < 16; srv++){ 
	hv_servo.setPWM(srv, 0, SERVOMID);
  }
  */
  
}


void loop() {
  set_rot(3, real_max[3]);
  digitalWrite(STATUS_LED, HIGH);
  delay(1000);
  set_rot(3, real_min[3]);
  digitalWrite(STATUS_LED, LOW);
  delay(1000);
  
  //spin ROS
  //nh.spinOnce();
  //delay(1);
}

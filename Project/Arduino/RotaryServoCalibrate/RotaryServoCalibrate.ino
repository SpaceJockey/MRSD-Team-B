
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//General Debugging stuff
#define STATUS_LED 13

Adafruit_PWMServoDriver hv_servo = Adafruit_PWMServoDriver();

//Radian values for us degree-centric folks
#define M_PI 3.14159265
#define DEG_20 (M_PI / 9)
#define DEG_30 (M_PI / 6)
#define DEG_45 (M_PI / 4)

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


#define SERVOMIN  185 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMID  307 // This is the Servo 'Middle Position'
#define SERVOMAX  430 // this is the 'maximum' pulse length count (out of 4096)
#define SERVORANGE ((float) (SERVOMAX - SERVOMIN))

//These are the configuration values for the robot, they need to be tuned!
//These are in radians or meters, depending on joint types
const float real_min[] = {-DEG_30, -DEG_30, -DEG_30, -DEG_45, -DEG_45,   0,   0};
const float real_max[] = { DEG_30,  DEG_30,  DEG_30,  DEG_45,  DEG_45, .145, .145};

//Servo addresses for the HV servo board
const unsigned int hv_servo_addr[] = {12, 4, 8, 5, 9, 0, 1};

//used only for linear joints where they have a limit switch
const unsigned int limit_pin[] = {0, 0, 0, 0, 0, 52, 53};

//To map things to our servo values
int servoMap(float value, unsigned int joint)
{return ((int) (((value - real_min[joint]) * SERVORANGE)  / (real_max[joint] - real_min[joint]))) + SERVOMIN;}

//Set the specified servo to the correct real-world position
void setServoPos(unsigned int joint, float value){
  unsigned int sval = servoMap(value, joint);
  hv_servo.setPWM(hv_servo_addr[joint], 0, servoMap(value, joint));
  Serial.print("Setting Channel ");
  Serial.print(joint);
  Serial.print(" to servo position: ");
  Serial.println(sval);
}

void setup() {
  Serial.begin(9600);
  Serial.println("Space Jockey Robot Servo Calibration Test!");
  
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  
  //initialize digital servo board
  hv_servo.begin();  
  hv_servo.setPWMFreq(50);
    
}


void loop() {
	int c = 3;
	for (float f = 0.0; f < 1.00; f = f + .005){
		Serial.print("Ratio: ");
		Serial.println(f);
		float real_val = real_min[c] + (real_max[c] - real_min[c]) * f;
		Serial.print("Setting Channel ");
		Serial.print(c);
		Serial.print(" to real position: ");
		Serial.println(real_val);
		setServoPos(c, real_val);
		delay(10);
	}
}

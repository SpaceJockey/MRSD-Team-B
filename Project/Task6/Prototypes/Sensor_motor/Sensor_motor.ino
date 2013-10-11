#include <Servo.h>
Servo myservo;

// Pin assignments
int IRpin = A0;                                    // analog pin for reading the IR sensor
int En1 = 8; //Enable pin for motor 1
int L1 = 9; // Logic 1 for motor 1
int L2 = 10; // Logic 2 for motor 2
int servopin = 2; // Servo pin


// Global variable


void setup() {
  Serial.begin(9600);                             // start the serial port
  pinMode(En1, OUTPUT);
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  myservo.attach(servopin);
}

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

void motor_dc(char c)
{
  if(c=='f') // forward
  {
    digitalWrite(En1, HIGH);
    digitalWrite(L1, LOW);
    digitalWrite(L2, HIGH);
  }
  else if(c=='b') //backward
  {
    digitalWrite(En1, HIGH);
    digitalWrite(L1, HIGH);
    digitalWrite(L2, LOW);
  }
}

void loop() 
{
  
  //=============================================== Using GPD 12======================================================================
//  float volts = analogRead(IRpin)*0.0048828125;   // value from sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
//  float distance = 65*pow(volts, -1.10);          // worked out from graph 65 = theretical distance / (1/Volts)S - luckylarry.co.uk
//  Serial.println(distance);                       // print the distance
//  delay(100);                                     // arbitary wait time.
  //===================================================================================================================================
  
  
//================================================Usinng motor controller============================================================
//  motor_dc('f');
//  delay(2000);
//  motor_dc('b');
//  delay(2000); 
//===================================================================================================================================
  
// ================================================Usinng Servo motor============================================================
//myservo.write(90);
//delay(1000);
//myservo.write(0);
//delay(1000);
//===================================================================================================================================
  
}


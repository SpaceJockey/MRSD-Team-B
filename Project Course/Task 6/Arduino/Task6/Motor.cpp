/* H-BRIDGE MOTOR DRIVER
*  MRSD Project Class Task 6
*  MRSD Team B
*/

#import "Motor.h"
#import <Arduino.h>
#import "Pinout.h"

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

Motor::Motor(uint8_t en_pin, uint8_t l1_pin, uint8_t l2_pin){
	_en_pin = en_pin;
	_l1_pin = l1_pin;
	_l2_pin = _l2_pin;
}


void Motor::set_dir(uint8_t dir){
  if(dir == FORWARD) {
    digitalWrite(_en_pin, HIGH);
    digitalWrite(_l1_pin, LOW);
    digitalWrite(_l2_pin, HIGH);
  }else if (dir == BACKWARD){
    digitalWrite(_en_pin, HIGH);
    digitalWrite(_l1_pin, HIGH);
    digitalWrite(_l2_pin, LOW);
  }
}

void Motor::coast(){
  digitalWrite(_en_pin, LOW);
}

void Motor::brake(){
  digitalWrite(_en_pin, HIGH);
  digitalWrite(_l1_pin,  HIGH);
  digitalWrite(_l2_pin,  HIGH);
}

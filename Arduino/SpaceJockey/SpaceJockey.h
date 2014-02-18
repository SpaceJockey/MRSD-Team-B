/*Servo Calibration Values */
#define SERVO_HZ  300

#define SERVOMIN  1231 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  2869 // this is the 'maximum' pulse length count (out of 4096)

/* Radian values for us degree-centric folks */
#define M_PI 3.14159265
#define DEG_20 (M_PI / 9)
#define DEG_30 (M_PI / 6)
#define DEG_45 (M_PI / 4)
//float degtorad(float deg){return (deg * M_PI) / 180.0;}



#include "Body.h"
#include "Battery.h"
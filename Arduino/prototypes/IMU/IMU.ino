// MPU-6050 Accelerometer + Gyro
// -----------------------------

#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"

//ROS stuff
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
//#include <std_msgs/Float32.h>

//ROS Stuff
ros::NodeHandle nh; //declared "extern" in SpaceJockey.h
static std_msgs::Int16MultiArray imu_msg; //imu state output
static ros::Publisher imu_state("imu_data", &imu_msg);

/*
should we have separate topics for temperature and such, or just dump everything, and let others cherry-pick upstream
static std_msgs::Float32 temp_msg; //imu temperature output, in degrees C
static ros::Publisher temp_state("temperature_data", &temp_msg);
*/

MPU6050 IMU = MPU6050(&Wire);

void setup()
{ 
  //Set up ROS node
  nh.initNode();
  nh.advertise(imu_state);
  //nh.advertise(temp_state);

  //spin until rosserial connection is live
  while (!nh.connected()) nh.spinOnce();
  nh.loginfo("IMU Dumper Arduino Connected.");

  IMU.begin();
}


void loop()
{
  int error;
  double dT;
  accel_t_gyro_union accel_t_gyro;

  //Read IMU data
  error = IMU.read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
  //TODO: fix this: if(error != 0) nh.logerror("IMU Read Error: " + itos(error));

  // Swap all high and low bytes to low byte first.
  uint8_t swap;
  #define SWAP(x,y) swap = x; x = y; y = swap

  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);


  //Dump the raw IMU data
  imu_msg.data_length = sizeof(accel_t_gyro);
  // convert the accelerometer data to 'g' unit 
  
  imu_msg.data = (int16_t *) &accel_t_gyro;
  imu_state.publish(&imu_msg);


  // The temperature sensor is -40 to +85 degrees Celsius.
  // It is a signed integer.
  // According to the datasheet: 
  //   340 per degrees Celsius, -512 at 35 degrees.
  // At 0 degrees: -512 - (340 * 35) = -12412
  /*
  temp_msg.data = ((float) accel_t_gyro.value.temperature + 12412.0) / 340.0;
  temp_state.publish( &temp_msg );
  */
  delay(1000);
}



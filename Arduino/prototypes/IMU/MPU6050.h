#ifndef MPU6050_H
#define MPU6050_H

// Default I2C address for the MPU-6050 is 0x68.
#define MPU6050_I2C_ADDRESS 0x68

//Sleep Ctl Register
#define MPU6050_PWR_MGMT_1         0x6B   // R/W

//Output Data Register Locations
#define MPU6050_ACCEL_XOUT_H       0x3B   // R  
#define MPU6050_ACCEL_XOUT_L       0x3C   // R  
#define MPU6050_ACCEL_YOUT_H       0x3D   // R  
#define MPU6050_ACCEL_YOUT_L       0x3E   // R  
#define MPU6050_ACCEL_ZOUT_H       0x3F   // R  
#define MPU6050_ACCEL_ZOUT_L       0x40   // R  
#define MPU6050_TEMP_OUT_H         0x41   // R  
#define MPU6050_TEMP_OUT_L         0x42   // R  
#define MPU6050_GYRO_XOUT_H        0x43   // R  
#define MPU6050_GYRO_XOUT_L        0x44   // R  
#define MPU6050_GYRO_YOUT_H        0x45   // R  
#define MPU6050_GYRO_YOUT_L        0x46   // R  
#define MPU6050_GYRO_ZOUT_H        0x47   // R  
#define MPU6050_GYRO_ZOUT_L        0x48   // R  

#include <Arduino.h>
#include <Scheduler.h>
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

extern ros::NodeHandle nh;

static float accel_f[3];
static std_msgs::Float32MultiArray imu_msg; //imu state output
static ros::Publisher imu_state("accel_data", &imu_msg);

// Declaring an union for the registers and the axis values.
// NOTE: The byte order does not match
// The AVR chip has the Low Byte at the lower address.
// But the MPU-6050 has the High Byte at the lower address
typedef union accel_t_union
{
  struct
  {
    uint8_t x_accel_h;
    uint8_t x_accel_l;
    uint8_t y_accel_h;
    uint8_t y_accel_l;
    uint8_t z_accel_h;
    uint8_t z_accel_l;
  } reg;
  int16_t accel[3];
};

//stub, implementation is after the class
void imuLoop();

class MPU6050{
	private:
		TwoWire *_wire;
		accel_t_union _accel_t;
	public:
		MPU6050(TwoWire *wire){
			this->_wire = wire;
		}

		void begin(){
			// Initialize the I2C-bus.
			this->_wire->begin();

			// Clear the 'sleep' bit to start the sensor.
			this->write (MPU6050_PWR_MGMT_1, 0, 1);

			nh.advertise(imu_state);
			imu_msg.data_length = 3;
			Scheduler.startLoop(imuLoop);
		}

		//read accelerometer data into x,y,z float buffer in Gs
		void readAccel(float *buffer){
			this->read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &this->_accel_t, sizeof(this->_accel_t));

			// Swap all high and low bytes to low byte first.
			uint8_t swap;
			#define SWAP(x,y) swap = x; x = y; y = swap

			SWAP (this->_accel_t.reg.x_accel_h, this->_accel_t.reg.x_accel_l);
			SWAP (this->_accel_t.reg.y_accel_h, this->_accel_t.reg.y_accel_l);
			SWAP (this->_accel_t.reg.z_accel_h, this->_accel_t.reg.z_accel_l);

			for(int i = 0; i < 3; i++) buffer[i] = ((float) this->_accel_t.accel[i]) / 16384.0;
		}

		//read bytes from the accelerometer
		void read(int start, uint8_t *buffer, int size){
			this->_wire->beginTransmission(MPU6050_I2C_ADDRESS);
			this->_wire->write(start);
			this->_wire->endTransmission(false);    // hold the I2C-bus
			this->_wire->requestFrom(MPU6050_I2C_ADDRESS, size, true);
			int i = 0;
			while(this->_wire->available() && i<size) buffer[i++]=this->_wire->read();
		}

		//write bytes to the accelerometer
		void write(int start, const uint8_t *pData, int size)
		{
			this->_wire->beginTransmission(MPU6050_I2C_ADDRESS);
			this->_wire->write(start); // write the start address
			this->_wire->write(pData, size);  // write data bytes
			this->_wire->endTransmission(true); // release the I2C-bus
		}
};


MPU6050 IMU = MPU6050(&Wire);

void imuLoop(){
  IMU.readAccel((float *) &accel_f);

  imu_msg.data = (float *) &accel_f;
  imu_state.publish(&imu_msg);

  //4 Hz update rate
  delay(250);
}

#endif

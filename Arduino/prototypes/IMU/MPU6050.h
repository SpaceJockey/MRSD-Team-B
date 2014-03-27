#ifndef MPU6050_H
#define MPU6050_H

// Nate removed all the unused Register Defs...

// Default I2C address for the MPU-6050 is 0x68.
// But only if the AD0 pin is low.
// Some sensor boards have AD0 high, and the
// I2C address thus becomes 0x69.
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


// Declaring an union for the registers and the axis values.
// The byte order does not match the byte order of 
// the compiler and AVR chip.
// The AVR chip (on the Arduino board) has the Low Byte 
// at the lower address.
// But the MPU-6050 has a different order: High Byte at
// lower address, so that has to be corrected.
// The register part "reg" is only used internally, 
// and are swapped in code.
typedef union accel_t_gyro_union
{
  struct
  {
    uint8_t x_accel_h;
    uint8_t x_accel_l;
    uint8_t y_accel_h;
    uint8_t y_accel_l;
    uint8_t z_accel_h;
    uint8_t z_accel_l;
    uint8_t t_h;
    uint8_t t_l;
    uint8_t x_gyro_h;
    uint8_t x_gyro_l;
    uint8_t y_gyro_h;
    uint8_t y_gyro_l;
    uint8_t z_gyro_h;
    uint8_t z_gyro_l;
  } reg;
  struct 
  {
    int16_t x_accel;
    int16_t y_accel;
    int16_t z_accel;
    int16_t temperature;
    int16_t x_gyro;
    int16_t y_gyro;
    int16_t z_gyro;
  } value;
};


class MPU6050{
	private:
		TwoWire *_wire;
	public:
		MPU6050(TwoWire *wire){
			this->_wire = wire;
		}

		void begin(){

			// Initialize the I2C-bus.
			this->_wire->begin();
			int error;
			uint8_t c;

			// default at power-up:
			//    Gyro at 250 degrees second
			//    Acceleration at 2g
			//    Clock source at internal 8MHz
			//    The device is in sleep mode.

			// Clear the 'sleep' bit to start the sensor.
			write_reg (MPU6050_PWR_MGMT_1, 0);
		}
// --------------------------------------------------------
// MPU6050_read
//
// This is a common function to read multiple bytes 
// from an I2C device.
//
// It uses the boolean parameter for Wire.endTransMission()
// to be able to hold or release the I2C-bus. 
// This is implemented in Arduino 1.0.1.
//
// Only this function is used to read. 
// There is no function for a single byte.
//
	int read(int start, uint8_t *buffer, int size){
		int i, n, error;
		this->_wire->beginTransmission(MPU6050_I2C_ADDRESS);
		n = this->_wire->write(start);
		if (n != 1) return (-10);

		n = this->_wire->endTransmission(false);    // hold the I2C-bus
		if (n != 0) return (n);

		// Third parameter is true: relase I2C-bus after data is read.
		this->_wire->requestFrom(MPU6050_I2C_ADDRESS, size, true);
		i = 0;
		while(this->_wire->available() && i<size) buffer[i++]=this->_wire->read();

		if ( i != size) return (-11);

		return (0);  // return : no error
	}


	// --------------------------------------------------------
	// MPU6050_write
	//
	// This is a common function to write multiple bytes to an I2C device.
	//
	// If only a single register is written,
	// use the function MPU_6050_write_reg().
	//
	// Parameters:
	//   start : Start address, use a define for the register
	//   pData : A pointer to the data to write.
	//   size  : The number of bytes to write.
	//
	// If only a single register is written, a pointer
	// to the data has to be used, and the size is
	// a single byte:
	//   int data = 0;        // the data to write
	//   MPU6050_write (MPU6050_PWR_MGMT_1, &c, 1);
	int write(int start, const uint8_t *pData, int size)
	{
	  int n, error;

	  this->_wire->beginTransmission(MPU6050_I2C_ADDRESS);
	  n = this->_wire->write(start); // write the start address
	  if (n != 1) return (-20);

	  n = this->_wire->write(pData, size);  // write data bytes
	  if (n != size) return (-21);

	  error = this->_wire->endTransmission(true); // release the I2C-bus
	  return error;
	}

	// --------------------------------------------------------
	// write_reg
	//
	// An extra function to write a single register.
	// It is just a wrapper around the MPU_6050_write()
	// function, and it is only a convenient function
	// to make it easier to write a single register.
	int write_reg(int reg, uint8_t data){
		return this->write(reg, &data, 1);
	}
};
#endif

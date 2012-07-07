#include "LSM303.h"
#include <math.h>
#include <stdio.h>
#include <linux/i2c-dev.h>
#include <stdlib.h>


// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address, 
// and sets the last bit correctly based on reads and writes
#define MAG_ADDRESS            (0x3C >> 1)
#define ACC_ADDRESS_SA0_A_LOW  (0x30 >> 1)
#define ACC_ADDRESS_SA0_A_HIGH (0x32 >> 1)

// Constructors ////////////////////////////////////////////////////////////////

LSM303::LSM303(int f)
{
  file = f;
	// These are just some values for a particular unit; it is recommended that
	// a calibration be done for your particular unit.
	m_max.x = +540; m_max.y = +500; m_max.z = 180;
	m_min.x = -520; m_min.y = -570; m_min.z = -770;
	
	_device = LSM303_DEVICE_AUTO;
	acc_address = ACC_ADDRESS_SA0_A_LOW;

	io_timeout = 0;  // 0 = no timeout
	did_timeout = false;
}

// Public Methods //////////////////////////////////////////////////////////////

bool LSM303::timeoutOccurred()
{
	return did_timeout;
}

void LSM303::setTimeout(unsigned int timeout)
{
	io_timeout = timeout;
}

unsigned int LSM303::getTimeout()
{
	return io_timeout;
}

void LSM303::init(byte device, byte sa0_a)
{	
  // sa0_a = LOW;
  // acc_address == ACC_ADDRESS_SA)_A_LOW
  // device = DLM
	_device = device;
	switch (_device)
	{
		case LSM303DLH_DEVICE:
		case LSM303DLM_DEVICE:
			if (sa0_a == LSM303_SA0_A_LOW)
				acc_address = ACC_ADDRESS_SA0_A_LOW;
			else if (sa0_a == LSM303_SA0_A_HIGH)
				acc_address = ACC_ADDRESS_SA0_A_HIGH;
			else
				acc_address = (detectSA0_A() == LSM303_SA0_A_HIGH) ? ACC_ADDRESS_SA0_A_HIGH : ACC_ADDRESS_SA0_A_LOW;
			break;	
		
		case LSM303DLHC_DEVICE:
			acc_address = ACC_ADDRESS_SA0_A_HIGH;
			break;
			
		default:
			// try to auto-detect device
			if (detectSA0_A() == LSM303_SA0_A_HIGH)
			{
				// if device responds on 0011001b (SA0_A is high), assume DLHC
				acc_address = ACC_ADDRESS_SA0_A_HIGH;
				_device = LSM303DLHC_DEVICE;
			}
			else // this actually occurs with my chip
			{
				// otherwise, assume DLH or DLM (pulled low by default on Pololu boards); query magnetometer WHO_AM_I to differentiate these two
				acc_address = ACC_ADDRESS_SA0_A_LOW;
	///			_device = (readMagReg(LSM303_WHO_AM_I_M) == 0x3C) ? LSM303DLM_DEVICE : LSM303DLH_DEVICE;
        _device = LSM303DLM_DEVICE;
			}
	}
}

// Turns on the LSM303's accelerometer and magnetometers and places them in normal
// mode.
void LSM303::enableDefault(void)
{
  if (ioctl(file, I2C_SLAVE, acc_address) < 0)
  {
      printf("Failed to acquire bus access and/or talk to slave %x", acc_address);
      exit(1);
  }
  __s32 res;
  __u8  reg, val;

  reg = LSM303_CTRL_REG1_A;
  val = 0x27;
	// Enable Accelerometer
	// 0x27 = 0b00100111
	// Normal power mode, all axes enabled

  res = i2c_smbus_write_byte_data(file, reg, val);
  if (res < 0)
  {
    printf("error in i2c_smbus_write_byte_data in LSM303::enableDefault\n");
    exit(1);
  }


  // *******************

  if (ioctl(file, I2C_SLAVE, MAG_ADDRESS) < 0)
  {
      printf("Failed to acquire bus access and/or talk to slave %x", MAG_ADDRESS);
      exit(1);
  }
  
  reg = LSM303_MR_REG_M;
  val = 0x00;
	// Enable Magnetometer
	// 0x00 = 0b00000000
	// Continuous conversion mode
  res = i2c_smbus_write_byte_data(file, reg, val);
  if (res < 0)
  {
    printf("error in i2c_smbus_write_byte_data in LSM303::enableDefault\n");
    exit(1);
  }
}

/*
// Writes an accelerometer register
void LSM303::writeAccReg(byte reg, byte value)
{
  m_i2c->i2cBegin(acc_address);
  m_i2c->i2cWrite(reg);
  m_i2c->i2cWrite(value);
	m_i2c->i2cEnd();
	last_status = 0;
}

// Reads an accelerometer register
byte LSM303::readAccReg(byte reg)
{
	byte value;

  m_i2c->i2cBegin(acc_address);
  m_i2c->i2cWrite(reg);
  last_status = 0;
  m_i2c->i2cRead(&value, 1);
	
	m_i2c->i2cEnd();
	
	return value;
}

// Writes a magnetometer register
void LSM303::writeMagReg(byte reg, byte value)
{
  m_i2c->i2cBegin(MAG_ADDRESS);
  m_i2c->i2cWrite(reg);
  m_i2c->i2cWrite(value);
	m_i2c->i2cEnd();
	last_status = 0;
}
*/

// Reads a magnetometer register
/*
byte LSM303::readMagReg(int reg)
{
	byte value;
	
	// if dummy register address (magnetometer Y/Z), use device type to determine actual address
	if (reg < 0)
	{
		switch (reg)
		{
			case LSM303_OUT_Y_H_M:
				reg = (_device == LSM303DLH_DEVICE) ? LSM303DLH_OUT_Y_H_M : LSM303DLM_OUT_Y_H_M;
				break;
			case LSM303_OUT_Y_L_M:
				reg = (_device == LSM303DLH_DEVICE) ? LSM303DLH_OUT_Y_L_M : LSM303DLM_OUT_Y_L_M;
				break;
			case LSM303_OUT_Z_H_M:
				reg = (_device == LSM303DLH_DEVICE) ? LSM303DLH_OUT_Z_H_M : LSM303DLM_OUT_Z_H_M;
				break;
			case LSM303_OUT_Z_L_M:
				reg = (_device == LSM303DLH_DEVICE) ? LSM303DLH_OUT_Z_L_M : LSM303DLM_OUT_Z_L_M;
				break;
		}
	}
  m_i2c->i2cBegin(MAG_ADDRESS);
  m_i2c->i2cWrite(reg);
  last_status = 0;
  m_i2c->i2cRead(&value, 1);
	
	m_i2c->i2cEnd();
	
	return value;
}
*/

void LSM303::setMagGain(magGain value)
{
  if (ioctl(file, I2C_SLAVE, MAG_ADDRESS) < 0)
  {
      printf("Failed to acquire bus access and/or talk to slave %x in setMagGain", MAG_ADDRESS);
      exit(1);
  }

  __s32 res;
  __u8  reg, val;

  reg = LSM303_CRB_REG_M;
  val = (__u8) value;

  res = i2c_smbus_write_byte_data(file, reg, val);
  if (res < 0)
  {
    printf("error in i2c_smbus_write_byte_data in LSM303::setMagGain\n");
    exit(1);
  }
}

// Reads the 3 accelerometer channels and stores them in vector a
void LSM303::readAcc(void)
{
  if (ioctl(file, I2C_SLAVE, acc_address) < 0)
  {
      printf("Failed to acquire bus access and/or talk to slave %x", acc_address);
      exit(1);
  }
	// assert the MSB of the address to get the accelerometer 
	// to do slave-transmit subaddress updating.
  __s32 res;
  __u8  reg, val;

  reg = LSM303_OUT_X_L_A | (1 << 7) ;

  __u8 buf[6];

  res = i2c_smbus_read_i2c_block_data(file, reg, 6, (__u8 *)buf);
  if (res != 6)
  {
    printf("Failed to read acc data in read()\n");
    exit(1);
  }


  for (int i = 0; i < 6; i++)
  {
    buf[i] = ~buf[i];
    buf[i] += 0x01;
  }
  /*
  printf("acc: ");
  for (int i = 0; i < 6; i++)
    printf("%02x ", buf[i]);
  printf("\n");
  */

  //__u8 test = 1; 
  //printf("test 1: %02X\n", test);
  //test = 240; 
  //printf("test 240: %02X\n", test);
	
  __u8 xla = buf[0];
  __u8 xha = buf[1];
  __u8 yla = buf[2];
  __u8 yha = buf[3];
  __u8 zla = buf[4];
  __u8 zha = buf[5];

  __s16 x = xha << 8 | xla;
  __s16 y = yha << 8 | yla;
  __s16 z = zha << 8 | zla;

	a.x = x >> 4;
	a.y = y >> 4;
	a.z = z >> 4;
}

// Reads the 3 magnetometer channels and stores them in vector m
void LSM303::readMag(void)
{
  if (ioctl(file, I2C_SLAVE, MAG_ADDRESS) < 0)
  {
      printf("Failed to acquire bus access and/or talk to slave %x in setMagGain", MAG_ADDRESS);
      exit(1);
  }
  __s32 res;
  __u8  reg, val;

  reg = LSM303_OUT_X_H_M;
  __u8  buf[6];

  res = i2c_smbus_read_i2c_block_data(file, reg, 6, (__u8 *)buf);
  if (res != 6)
  {
    printf("Failed to read acc data in read()\n");
    exit(1);
  }


  for (int i = 0; i < 6; i++)
  {
    buf[i] = ~buf[i];
    buf[i] += 0x01;
  }
  printf("mag: ");
  for (int i = 0; i < 6; i++)
    printf("%02x ", buf[i]);
  printf("\n");

  __u8 xhm = buf[0];
  __u8 xlm = buf[1];
	
	__u8 yhm, ylm, zhm, zlm;
	
	if (_device == LSM303DLH_DEVICE)
	{
		// DLH: register address for Y comes before Z
		yhm = buf[2];
		ylm = buf[3];
		zhm = buf[4];
		zlm = buf[5];
	}
	else
	{
		// DLM, DLHC: register address for Z comes before Y
		zhm = buf[2];
		zlm = buf[3];
		yhm = buf[4];
		ylm = buf[5];

	}


  __s16 x = xhm << 8 | xlm;
  __s16 y = yhm << 8 | ylm;
  __s16 z = zhm << 8 | zlm;

	m.x = x;
	m.y = y;
	m.z = z;
}

// Reads all 6 channels of the LSM303 and stores them in the object variables
void LSM303::read(void)
{
	readAcc();
	readMag();
}

// Returns the number of degrees from the -Y axis that it
// is pointing.
int LSM303::heading(void)
{
	return heading((vector){0,-1,0});
}

// Returns the number of degrees from the From vector projected into
// the horizontal plane is away from north.
// 
// Description of heading algorithm: 
// Shift and scale the magnetic reading based on calibration data to
// to find the North vector. Use the acceleration readings to
// determine the Down vector. The cross product of North and Down
// vectors is East. The vectors East and North form a basis for the
// horizontal plane. The From vector is projected into the horizontal
// plane and the angle between the projected vector and north is
// returned.
int LSM303::heading(vector from)
{
    // shift and scale
    m.x = (m.x - m_min.x) / (m_max.x - m_min.x) * 2 - 1.0;
    m.y = (m.y - m_min.y) / (m_max.y - m_min.y) * 2 - 1.0;
    m.z = (m.z - m_min.z) / (m_max.z - m_min.z) * 2 - 1.0;

    vector temp_a = a;
    // normalize
    vector_normalize(&temp_a);
    //vector_normalize(&m);

    // compute E and N
    vector E;
    vector N;
    vector_cross(&m, &temp_a, &E);
    vector_normalize(&E);
    vector_cross(&temp_a, &E, &N);
	
    // compute heading
    int heading = round(atan2(vector_dot(&E, &from), vector_dot(&N, &from)) * 180 / M_PI);
    if (heading < 0) heading += 360;
	return heading;
}

void LSM303::vector_cross(const vector *a,const vector *b, vector *out)
{
  out->x = a->y*b->z - a->z*b->y;
  out->y = a->z*b->x - a->x*b->z;
  out->z = a->x*b->y - a->y*b->x;
}

float LSM303::vector_dot(const vector *a,const vector *b)
{
  return a->x*b->x+a->y*b->y+a->z*b->z;
}

void LSM303::vector_normalize(vector *a)
{
  float mag = sqrt(vector_dot(a,a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

// Private Methods //////////////////////////////////////////////////////////////

byte LSM303::detectSA0_A(void)
{
  /*
  m_i2c->i2cBegin(ACC_ADDRESS_SA0_A_LOW);
  m_i2c->i2cWrite(LSM303_CTRL_REG1_A);
  m_i2c->i2cEnd();
  last_status = 0;
	Wire.requestFrom(ACC_ADDRESS_SA0_A_LOW, 1);
	if (Wire.available())
	{
		Wire.read();
		return LSM303_SA0_A_LOW;
	}
	else
		return LSM303_SA0_A_HIGH;
    */
  return LSM303_SA0_A_LOW;
  //return LSM303_SA0_A_HIGH;
}

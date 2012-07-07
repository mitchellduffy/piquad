#include "L3G4200D.h"
#include "math.h"
#include <stdio.h>
#include <linux/i2c-dev.h>
#include <stdlib.h>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address, 
// and sets the last bit correctly based on reads and writes
#define GYR_ADDRESS (0xD2 >> 1  )
//#define GYR_ADDRESS (0x3C >> 1  )

// Public Methods //////////////////////////////////////////////////////////////
typedef unsigned char uint8_t;

L3G4200D::L3G4200D(int f)
{
  file = f;
}


// Turns on the L3G4200D's gyro and places it in normal mode.
void L3G4200D::enableDefault(void)
{

  // Specify the address of the slave device.
  if (ioctl(file, I2C_SLAVE, GYR_ADDRESS) < 0)
  {
      printf("Failed to acquire bus access and/or talk to slave %x", GYR_ADDRESS);
      exit(1);
  }


  __s32 res;
  __u8  reg, val;

  reg = L3G4200D_CTRL_REG1;
  val = 0x0F;
	// 0x0F = 0b00001111
	// Normal power mode, all axes enabled

  res = i2c_smbus_write_byte_data(file, reg, val);
  if (res < 0)
  {
    printf("error in i2c_smbus_write_byte_data in enableDefault\n");
    exit(1);
  }

  reg = L3G4200D_CTRL_REG4;
  val = 0x20; //high res
  res = i2c_smbus_write_byte_data(file, reg, val);
  if (res < 0)
  {
    printf("error in i2c_smbus_write_byte_data in enableDefault\n");
    exit(1);
  }


}

// Writes a gyro register
/*
void L3G4200D::writeReg(byte reg, byte value)
{
	m_i2c->i2cBegin(GYR_ADDRESS);
	m_i2c->i2cWrite(reg);
	m_i2c->i2cWrite(value);
	m_i2c->i2cEnd();
}
*/


// Reads a gyro register
/*
byte L3G4200D::readReg(byte reg)
{
	byte value;
	
	Wire.beginTransmission(GYR_ADDRESS);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(GYR_ADDRESS, 1);
	value = Wire.read();
	Wire.endTransmission();
	
	return value;
}
*/

// Reads the 3 gyro channels and stores them in vector g
void L3G4200D::read()
{
  // Specify the address of the slave device.
  if (ioctl(file, I2C_SLAVE, GYR_ADDRESS) < 0)
  {
      printf("Failed to acquire bus access and/or talk to slave %x", GYR_ADDRESS);
      exit(1);
  }

  __s32 res;
  __u8  reg, val;

  reg = L3G4200D_OUT_X_L | (1 << 7) ;
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
  printf("g: ");
  for (int i = 0; i < 6; i++)
    printf("%02x ", buf[i]);
  printf("\n");

  __u8 xla = buf[0];
  __u8 xha = buf[1];
  __u8 yla = buf[2];
  __u8 yha = buf[3];
  __u8 zla = buf[4];
  __u8 zha = buf[5];

  __s16 x = xha << 8 | xla;
  __s16 y = yha << 8 | yla;
  __s16 z = zha << 8 | zla;

	g.x = x;
	g.y = y;
	g.z = z;
}

void L3G4200D::vector_cross(const vector *a,const vector *b, vector *out)
{
  out->x = a->y*b->z - a->z*b->y;
  out->y = a->z*b->x - a->x*b->z;
  out->z = a->x*b->y - a->y*b->x;
}

float L3G4200D::vector_dot(const vector *a,const vector *b)
{
  return a->x*b->x+a->y*b->y+a->z*b->z;
}

void L3G4200D::vector_normalize(vector *a)
{
  float mag = sqrt(vector_dot(a,a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

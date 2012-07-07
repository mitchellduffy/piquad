#include "i2c.h"

#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

I2C::I2C(char * filename)
{
	if ((fd = open(filename, O_RDWR)) < 0) {					// Open port for reading and writing
		printf("I2C: Failed to open i2c port %s\n", filename);
		exit(1);
	} 
}

void I2C::i2cBegin(const int &add)
{
  address = add;
}

void I2C::i2cBeginLow(const int &address)
{
	if (ioctl(fd, I2C_SLAVE, address) < 0) {					// Set the port options and set the address of the device we wish to speak to
		printf("I2C: Unable to get bus access to talk to slave %x\n", address);
		exit(1);
	} 
}

void I2C::i2cEnd()
{

}

void I2C::i2cWrite(const unsigned char &val)
{
  i2cBeginLow(address); 

  unsigned char buf[1];
  buf[0] = val;
	if ((write(fd, buf, 1)) != 1) {								// Send register we want to read from	
		printf("Error writing to i2c slave\n");
		exit(1);
	}
}

void I2C::i2cRead(char * buf, int size)
{ 
  i2cBeginLow(address); 
	if (read(fd, buf, size) != size) {
		printf("Unable to read from slave\n");
		exit(1);
	}
}

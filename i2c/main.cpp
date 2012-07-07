#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

//#include "i2c.h"
#include "L3G4200D.h"
#include "LSM303.h"

int main(int argc, char **argv)
{
	//int  address = 0xD3 >> 1;										// Address of CMPS03 shifted right one bit


/*
  i2c.i2cBegin(0xD2 >> 1);
  i2c.i2cWrite(0x0F);
  unsigned char buf[10];
  i2c.i2cRead(buf, 1);
  printf("%x\n", buf[0]);
  */
  const char * devName = "/dev/i2c-0";

  // Open up the I2C bus
  int file = open(devName, O_RDWR);
  if (file == -1)
  {
      perror(devName);
      exit(1);
  }
  L3G4200D gyro(file);
  gyro.enableDefault();

  LSM303 lsm(file);
  lsm.init();
  lsm.enableDefault();

  while (1)
  {
    usleep(50000);
    gyro.read();
    lsm.read();
  

    printf("g: %f %f %f\n", gyro.g.x, gyro.g.y, gyro.g.z);
    printf("a: %f %f %f\n", lsm.a.x, lsm.a.y, lsm.a.z);
    printf("m: %f %f %f\n", lsm.m.x, lsm.m.y, lsm.m.z);
  }

/*
  LSM303 lsm(&i2c);
  lsm.init();
  lsm.enableDefault();


  while (1)
  {
    lsm.read();

    gyro.read();
  

    printf("g: %f %f %f\n", gyro.g.x, gyro.g.y, gyro.g.z);
    printf("a: %f %f %f\n", lsm.a.x, lsm.a.y, lsm.a.z);
    printf("m: %f %f %f\n", lsm.m.x, lsm.m.y, lsm.m.z);
    usleep(10000);
  }
  
  */
		

	//bu = 0x20;													// This is the register we want to read from
	
  /*
	if ((write(fd, buf, 1)) != 1) {								// Send register we want to read from	
		printf("Error writing to i2c slave\n");
		exit(1);
	}
	buf[0] = 0x0F;
	if ((write(fd, buf, 1)) != 1) {								// Send register we want to read from	
		printf("Error writing to i2c slave 2\n");
		exit(1);
	}
	*/
/*
	if (read(fd, buf, 4) != 4) {								// Read back data into buf[]
		printf("Unable to read from slave\n");
		exit(1);
	}
	else {
		unsigned char highByte = buf[2];
		unsigned char lowByte = buf[3];
		unsigned int result = (highByte <<8) + lowByte;			// Calculate bearing as a word value
		printf("Software v: %u \n",buf[0]);
		printf("Bearing as byte: %u \n",buf[1]);
		printf("Bearing as decimal: %u.%u\n",result/10, result%10); // display bearing with decimal place
	}
*/
	
	return 0;
}



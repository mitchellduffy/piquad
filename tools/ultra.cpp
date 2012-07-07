#include <stdio.h>
#include "unistd.h"
#include "ultrasound.h"
#include <bcm2835.h>

#include "utils.h"



#define PIN_TRIG RPI_GPIO_P1_26



int main(int argc, char **argv)
{
  init_millis();

  uint8_t pins[6];
  pins[0] = RPI_GPIO_P1_18;
  pins[1] = RPI_GPIO_P1_19;
  pins[2] = RPI_GPIO_P1_21;
  pins[3] = RPI_GPIO_P1_22;
  pins[4] = RPI_GPIO_P1_23;
  pins[5] = RPI_GPIO_P1_24;


  pins[0] = RPI_GPIO_P1_18;

  
  Ultrasound u(PIN_TRIG, pins, 1);
  u.start();

  while(true)
    usleep(20000);
  return 0;
}


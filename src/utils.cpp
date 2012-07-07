#include "utils.h"

#include <sys/time.h>
#include <stdio.h>
long int time_sec, time_usec;

long usec()
{
  struct timeval t;
  gettimeofday(&t, 0x0);

  long r = 0;

  r = 1000000*(t.tv_sec - time_sec)+(t.tv_usec - time_usec);

  return r;
}

long millis()
{
  struct timeval t;
  gettimeofday(&t, 0x0);

  long r = 0;

  r = 1000*(t.tv_sec - time_sec)+(t.tv_usec - time_usec)/1000;

  return r;
}


void init_millis()
{
  struct timeval t;
  gettimeofday(&t, 0x0);

  time_sec = t.tv_sec;
  time_usec = t.tv_usec;
  
  printf("time: %ds, %dus\n", time_sec, time_usec);
}

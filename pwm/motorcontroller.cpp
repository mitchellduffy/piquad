#include "motorcontroller.h"
#include <boost/thread.hpp>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include "rpipins.h"

#define MC_MOTOR_0 RPI_12


long usec_start;

long usec()
{
  boost::posix_time::ptime t = boost::posix_time::microsec_clock::local_time();
  boost::posix_time::time_duration td(t.time_of_day() );
  return td.total_microseconds() - usec_start;
}
MotorController::MotorController(const double &voltage)
{
  m_thread = 0;
  setVoltage(0, voltage);


  usec_start = 0;
  usec_start = usec();


  if (!bcm2835_init())
  {
    printf("unable to setup bcm2835 gpio \n");
    exit(1);
  }
  bcm2835_gpio_fsel(MC_MOTOR_0, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_write(MC_MOTOR_0, LOW);


  m_thread = new boost::thread(boost::bind(&MotorController::threadLoop, this));
}

void MotorController::setVoltage(const int &id, const double &v)
{
  if (v < 0) m_voltage = 0;
  else if (v > 1) m_voltage = 1;
  else m_voltage = v;
}

void MotorController::threadLoop()
{
  long t_beg, t_mid, t_end;

  double diff[1000];
  int i = 0;

  double correction = 1/1.16;
  correction = 1/1.14;
  while (true)
  {
    int delay = 1000 * (1 + m_voltage);

    t_beg = usec();
    bcm2835_gpio_write(MC_MOTOR_0, HIGH);

    boost::this_thread::sleep(boost::posix_time::microseconds(delay * correction));


    bcm2835_gpio_write(MC_MOTOR_0, LOW);

    t_mid = usec(); 
    diff[i] = ((double)(t_mid - t_beg)) / delay;

    delay = 2000 - (usec() - t_beg);
    if (delay > 0)
    {
      boost::this_thread::sleep(boost::posix_time::microseconds(delay * correction));
    }
    i++;

    if (i == 1000)
    {
      double avg = 0;
      for (int k = 0; k < 1000; k++)
      {
        avg += diff[k];
      }
      avg /= 1000;
      double stddev = 0;
      for (int k = 0; k < 1000; k++)
      {
        stddev += (avg - diff[k])*(avg - diff[k]);
      }
      stddev /= 1000;
      stddev = sqrt(stddev);

      printf("accuracy: %.4f +- %.4f\n", avg, stddev);
      i = 0;



    }
  }


}

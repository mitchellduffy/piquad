#include "ultrasound.h"
#include <boost/thread.hpp>
#include <boost/thread/barrier.hpp>
#include <boost/date_time.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <poll.h>
#include <bcm2835.h>
#include "utils.h"


Ultrasound::Ultrasound(uint8_t trigger, uint8_t * pins, int pin_count) : m_trig(trigger), m_pin_count(pin_count)
{
  m_pins         = new uint8_t[pin_count];
  m_status       = new UltrasoundStatus[pin_count];
  m_send_time    = new long[pin_count];
  m_return_time  = new long[pin_count];
  m_dist         = new double[pin_count];
  m_sensor_thread = new void*[pin_count];
  m_master_thread = 0;
  m_trigger_time = 0;
  m_barrier = new boost::barrier(pin_count + 1);
  for (int i = 0; i < pin_count; i++)
  {
    m_pins[i]         = pins[i];
    m_status[i]       = Ready;
    m_send_time[i]    = 0;
    m_return_time[i]  = 0;
    m_dist[i]         = -1;
  }
  if (!bcm2835_init())
  {
    printf("unable to setup bcm2835 gpio \n");
    exit(1);
  }
}

void Ultrasound::start()
{
  if (m_master_thread == 0)
  {
    for (int i = 0; i < m_pin_count; i++)
    {
      m_sensor_thread[i] = (void*) new boost::thread(boost::bind(&Ultrasound::sensor_loop, this, i));
    }

    m_master_thread = (void*) new boost::thread(boost::bind(&Ultrasound::master_loop, this));
  }
}

Ultrasound::~Ultrasound()
{
  delete m_pins;
  delete m_status;
  delete m_return_time;
  delete m_dist;
}


void Ultrasound::master_loop()
{
  printf("master\n");
  bcm2835_gpio_fsel(m_trig, BCM2835_GPIO_FSEL_OUTP);

  while (true)
  {
    // synchronize all threads before trigger
    m_barrier->wait();

    // check for IO errors
    bool ok = true;
    for (int p = 0; p < m_pin_count; p++)
    {
      ok = ok && (m_status[p] != IOError);
    }
    // shut down threads on error
    if (!ok)
    {
      for (int p = 0; p < m_pin_count; p++)
      {
        delete (boost::thread*)m_sensor_thread[p];
      }
      printf("IO Error in Ultrasound::master_loop\n");
      return;
    }


    // continue with regular stuff

    // trigger 
    for (int p = 0; p < m_pin_count; p++)
    {
      m_status[p] = Triggered;
    }
    m_trigger_time = usec();
    //printf("time: %10ld triggered \n", m_trigger_time);
    // triggering
    bcm2835_gpio_write(m_trig, HIGH);
    usleep(100);
    bcm2835_gpio_write(m_trig, LOW);

    //sync all threads again for triggering
    m_barrier->wait();
     

    // wait until 50 ms
    long diff = usec() - m_trigger_time;
    if (diff > 0 && diff < 50000)
      boost::this_thread::sleep(boost::posix_time::microseconds(50000-diff));

    //diff = usec() - m_trigger_time;
    //printf("since last: %d\n" ,diff);  

    // compute distances!
    for (int p = 0; p < m_pin_count; p++)
    {
      switch (m_status[p])
      {
        case Triggered:
        case IOError:
          m_dist[p] = -1;
          break;
        case Sent:
          m_dist[p] = 0;
          break;
        case Received:
          m_dist[p] = (double)(m_return_time[p] - m_send_time[p]) * 340 / 2 / 1000 / 1000;
      }
      printf("%.1fcm ", m_dist[p]*100);
    }
    printf("\n");
     
  }

}

#define POLL_TIMEOUT 23
void Ultrasound::sensor_loop(int i)
{


  char devname[256];

  sprintf(devname, "/sys/class/gpio/gpio%d/value", m_pins[i]);

  char buf[2];
  memset(buf, 0x00, 2);


  int file = open(devname, O_RDONLY);
  if (file < 0)
  {
    printf("error opening ultrasound device id %d, pin %d\n", i, m_pins[i]);
    m_status[i] = IOError;
    m_barrier->wait();
    return;
  }

  struct pollfd pfd;
  pfd.fd = file;
  pfd.events = POLLPRI;

  int ret;

  while (true)
  {
    m_barrier->wait();
    m_barrier->wait();

    for (int l = 0; l < 2; l++)
    {
      memset(buf, 0x00, 2);
      lseek(file, 0, SEEK_SET);
      ret=poll(&pfd, 1, POLL_TIMEOUT);
      if (ret < 0) 
      {
        printf("poll error");
        close(file);
        m_status[i] = IOError;
        m_barrier->wait();
        return;
      }
      if (ret == 0) 
      {
        printf("timeout\n");
        continue;
      }

      ret = read(file, buf, 1);
      if (ret < 0) 
      {
        perror("read error");
        close(file);
        m_status[i] = IOError;
        m_barrier->wait();
        return;
      }
      if (buf[0] == '1' && m_status[i] == Triggered)
      {
        m_send_time[i] = usec();
        m_status[i] = Sent;
      }
      if (buf[0] == '0' && m_status[i] == Sent)
      {
        m_return_time[i] = usec();
        m_status[i] = Received;
      }
    }
    //if (m_status[i] == Received)
      //printf("pulse: %ld %ld\n", m_send_time[i], m_return_time[i]);
  }
  return;
}

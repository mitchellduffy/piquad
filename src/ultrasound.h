#ifndef ULTRASOUND_H
#define ULTRASOUND_H 1

#include <stdint.h>

namespace boost
{
  class barrier;
}

class Ultrasound
{
  enum UltrasoundStatus
  {
    Ready, Triggered, Sent, Received, IOError
  };
  
  public: 
    Ultrasound(uint8_t pin_trig, uint8_t * pin_inpt, int pin_inpt_count);
    ~Ultrasound();
    void start();

    double getDistance(int id);



  private:
    uint8_t  m_trig;
    int      m_pin_count;
    uint8_t* m_pins;
    UltrasoundStatus * m_status;
    long     m_trigger_time;
    long   * m_send_time;
    long   * m_return_time;
    double * m_dist;

    void * m_master_thread;
    void ** m_sensor_thread;


    void master_loop();
    void sensor_loop(int id);

    boost::barrier * m_barrier;


};







#endif

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H 1

namespace boost { class thread; }

class MotorController
{
  public:
    MotorController(const double &voltage);
    void setVoltage(const int &id, const double &v);
    void threadLoop();

  private:
    int m_file;
    boost::thread * m_thread;
    double m_voltage;






};


#endif

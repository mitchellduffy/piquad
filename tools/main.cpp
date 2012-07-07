#include "utils.h"
#include "imu.h"
#include "unistd.h"
#include <stdio.h>

#define TARGET_FREQ 100

IMU imu;
int loops;

int main()
{
  init_millis(); 

  IMU imu; 

  imu.setup();

  imu.initialize();

  // 
  usleep(IMU_MIN_SENSOR_REST * 1000);

  int loops = 0;

  int target_time_diff = 1.0 / TARGET_FREQ * 1000 * 1000;

  long time_last = usec();
  double avg_freq = 50.0;
  double avg_freq_weight = 0.1;
  while (true)
  {
    int time_diff = usec() - time_last;
    //printf("time diff: %d\n", time_diff);
    if (time_diff < target_time_diff)
    {
      usleep((target_time_diff - time_diff)/1.45);
    }

    imu.read(); // 3 milisecs
    
    // **
    time_diff = usec() - time_last;
    avg_freq = (avg_freq_weight / time_diff * 1000 * 1000) + (1-avg_freq_weight)*avg_freq;
    // **

    imu.process(); // 4 milisec

    Vector3D m, a, g;

    m = imu.getM();
    a = imu.getA();
    g = imu.getG();

    printf("d %6dus %.0fHz    %9.6f %9.6f %9.6f    %9.6f %9.6f %9.6f    %9.6f %9.6f %9.6f\n", 
      //millis(),
      time_diff,
      avg_freq,
      a.x(), a.y(), a.z(),
      m.x(), m.y(), m.z(),
      g.x(), g.y(), g.z()
      );

    time_last = usec();
    continue;
    Quaternion pos;

    pos = imu.getPosRawAcc();
    printf("A ");
    printf("%9.6f %9.6f %9.6f %9.6f ",
      pos.w(), pos.x(), pos.y(), pos.z()
      );
    pos = imu.getPosRawAccError();
    printf("%9.6f %9.6f %9.6f %9.6f ",
      pos.w(), pos.x(), pos.y(), pos.z()
      );
    printf("\n");

    pos = imu.getPosRawGyr();
    printf("G ");
    printf("%9.6f %9.6f %9.6f %9.6f ",
      pos.w(), pos.x(), pos.y(), pos.z()
      );
    pos = imu.getPosRawGyrError();
    printf("%9.6f %9.6f %9.6f %9.6f ",
      pos.w(), pos.x(), pos.y(), pos.z()
      );
    printf("\n");

    VectorND<7> kal;
    kal= imu.getPosKalman();
    printf("K ");
    printf("%9.6f %9.6f %9.6f %9.6f   %9.6f %9.6f %9.6f\n",
      kal(0), kal(1), kal(2), kal(3), kal(4), kal(5), kal(6)
      );
/*
    kal = imu.getPosKalmanErr();
    printf("K ");
    printf("%9.6f %9.6f %9.6f %9.6f   %9.6f %9.6f %9.6f\n",
      kal(0), kal(1), kal(2), kal(3), kal(4), kal(5), kal(6)
      );
    */


    loops++;
  }


}


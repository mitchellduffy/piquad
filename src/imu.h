#ifndef IMU_H
#define IMU_H 1

#define IMU_HISTORY 2

#define IMU_MIN_SENSOR_REST 35
// gyro max rate ~36Hz, not corresponding to the above 35ms loop

#include "mathtools.h"


class L3G4200D;
class LSM303;
class IMU
{
  public:
    IMU();
    ~IMU();

    // setup sensors
    void setup();
 
    // measure initial position. setup filter vars
    void initialize();

    // read sensor data
    void read(); 

    // execute filters
    void process();

    

    Vector3D getM();
    Vector3D getA();
    Vector3D getG();

    // get raw position computed from acceleration only
    Quaternion getPosRawAcc();
    // get raw position computed from gyroonly
    Quaternion getPosRawGyr();
    // get position computed from combining acc and mag, found with the newton method
    Quaternion getPosNewton();
    // get respective position error
    Quaternion getPosRawAccError();
    // get respective position error
    Quaternion getPosRawGyrError();
    // get respective position error
    Quaternion getPosNewtonError();
    // get the full output of the Kalman filter (0-3 pos, 4-6 gyro bias)
    VectorND<7> getPosKalman();


  private:
    int file;
    L3G4200D * gyro;
    LSM303   * compass;

    Vector3D m[IMU_HISTORY], a[IMU_HISTORY], g[IMU_HISTORY];
    Quaternion rm; // reference magnetic field

    unsigned long t[IMU_HISTORY];


    Quaternion posrawacc[IMU_HISTORY], posrawaccerr[IMU_HISTORY];
    Quaternion posrawgyr[IMU_HISTORY], posrawgyrerr[IMU_HISTORY];
    Quaternion posnwt[IMU_HISTORY], posnwterr[IMU_HISTORY];
    VectorND<7> pos[IMU_HISTORY]; 
    MatrixNxN<7> P[IMU_HISTORY];

    int id;
    int prev_id;
    unsigned long prev_t;

    int loop;

    // estimate raw position using acceleration only
    void estimateAngleSimple(/*inputs*/const Vector3D &acc, const Vector3D &accerr, /*outputs*/ Quaternion &angle, Quaternion &angleerr);

    // estimate position using newton method
    void estimateAngle(/* outputs*/ Quaternion &angle, Quaternion &angelerr);



};

#endif

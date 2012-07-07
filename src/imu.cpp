#include "imu.h"
#include "L3G4200D.h"
#include "LSM303.h"
#include "math.h"
#include "unistd.h"
#include <fcntl.h>
#include <stdio.h>
#include "utils.h"

#include <stdlib.h>

#define DEBUG false


// C - center offset
// S - scale
// E - error
// note errors after normalization

// magnetometer
#define IMU_MAG_C_X -111.732
#define IMU_MAG_C_Y -99.946
#define IMU_MAG_C_Z 95.578

#define IMU_MAG_S_X 527.854
#define IMU_MAG_S_Y 537.074
#define IMU_MAG_S_Z 506.062

#define IMU_MAG_E_X 0.007
#define IMU_MAG_E_Y 0.007
#define IMU_MAG_E_Z 0.007

//accelerometer
#define IMU_ACC_C_X -31.32
#define IMU_ACC_C_Y  64.36
#define IMU_ACC_C_Z  34.25

#define IMU_ACC_S_X 1029.22
#define IMU_ACC_S_Y 1050.55
#define IMU_ACC_S_Z 1037.28

#define IMU_ACC_E_X 0.00751 
#define IMU_ACC_E_Y 0.00674
#define IMU_ACC_E_Z 0.00907

// gyroscope
#define IMU_GYR_E_X 0.00609
#define IMU_GYR_E_Y 0.00577
#define IMU_GYR_E_Z 0.10741




/*
   Gyro sensitivity scales: 
     1 -- 250 dps (degree per second)
     2 -- 500 dps
     3 -- 2000 dps
*/
#define IMU_GYRO_SCALE 1

#if IMU_GYRO_SCALE == 1
  #define IMU_GYRO_GAIN 0.00875
  #define IMU_GYRO_CONF 0x00
#elif IMU_GYRO_SCALE == 2
  #define IMU_GYRO_GAIN 0.01750
  #define IMU_GYRO_CONF 0x10
#else
  #define IMU_GYRO_GAIN 0.070
  #define IMU_GYRO_CONF 0x20
#endif



#define IMU_INIT_READINGS 15


double square(const double &arg)
{
  return arg*arg;
}




// helper function to print a matrix to Serial
template <int N, int M>
void printmatrix(const MatrixNxM<N,M> &m)
{
  for (int i = 0; i < N; i++)
  {
    for (int j = 0; j < M; j++)
    {
      printf("%.6f ", m(i, j));
    }
    printf("\n");
  }
}

// helper function to print a vector to Serial
template <int N>
void printvector(const VectorND<N> &v)
{
  for (int i = 0; i < N; i++)
  {
    printf("%.6f ", v(i));
  }
  printf("\n");
}


IMU::IMU()
{
  char devname[256];
  sprintf(devname, "/dev/i2c-0");
  int file = open(devname, O_RDWR);
  if (file == -1)
  {
      printf("error opening the i2c bus file\n");
      exit(1);
  }
  gyro = new L3G4200D(file);
  compass = new LSM303(file);
  loop = 0;
  prev_t = 0;
}

IMU::~IMU()
{
  delete gyro;
  delete compass;
}

// setup sensors
void IMU::setup()
{
	compass->init();
	compass->enableDefault();
	gyro->enableDefault();

  // time will be set in initialization later anyway
  prev_t = 0;
}

void IMU::initialize()
{
  // mock readout (sets prev_t)
  usleep(IMU_MIN_SENSOR_REST*1000);
  id = 0;
  read(); // id updated here
  id = 0;
  usleep(IMU_MIN_SENSOR_REST*1000);


  Vector3D aa[IMU_INIT_READINGS];
  Vector3D mm[IMU_INIT_READINGS];
  Vector3D gg[IMU_INIT_READINGS];
  Vector3D avga;
  Vector3D avgm;
  Vector3D avgg;

  // store initialization readings
  for (int i = 0; i < IMU_INIT_READINGS; i++)
  {
    read();
    aa[i] = a[id];
    mm[i] = m[id];
    gg[i] = g[id];
    usleep(IMU_MIN_SENSOR_REST*1000);
  }

  //compute averages
  for (int i = 0; i < IMU_INIT_READINGS; i++)
  {
    avga += aa[i];
    avgm += mm[i];
    avgg += gg[i];
  }
  avga /= IMU_INIT_READINGS;
  avgm /= IMU_INIT_READINGS;
  avgg /= IMU_INIT_READINGS;
  //compute error
  double errax = 0, erray = 0, erraz = 0;
  double errgx = 0, errgy = 0, errgz = 0;
  for (int i = 0; i < IMU_INIT_READINGS; i++)
  {
    errax += (aa[i].x() - avga.x())*(aa[i].x() - avga.x());
    erray += (aa[i].y() - avga.y())*(aa[i].y() - avga.y());
    erraz += (aa[i].z() - avga.z())*(aa[i].z() - avga.z());
    errgx += (gg[i].x() - avgg.x())*(gg[i].x() - avgg.x());
    errgy += (gg[i].y() - avgg.y())*(gg[i].y() - avgg.y());
    errgz += (gg[i].z() - avgg.z())*(gg[i].z() - avgg.z());
  }
  errax /= IMU_INIT_READINGS;
  erray /= IMU_INIT_READINGS;
  erraz /= IMU_INIT_READINGS;
  errgx /= IMU_INIT_READINGS;
  errgy /= IMU_INIT_READINGS;
  errgz /= IMU_INIT_READINGS;
  errax = sqrt(errax);
  erray = sqrt(erray);
  erraz = sqrt(erraz);
  errgx = sqrt(errgx);
  errgy = sqrt(errgy);
  errgz = sqrt(errgz);
  Vector3D erra(errax, erray, erraz);
  Vector3D errg(errgx, errgy, errgz);
   
  // normalize and let's hope length a was ~1 to begin with
  avga /= avga.length();
  avgm /= avgm.length();

  
  // compute the yaw=0 position quaternion
  estimateAngleSimple(avga, erra, posrawacc[id], posrawaccerr[id]);
  
  printf(" initial position: ");
  printvector(posrawacc[id]);

  posrawgyr[id] = posrawacc[id];

  // store it as the first prev-state to Kalman in the next arduino loop
  pos[id] = posrawacc[id];
  for (int i = 0; i < 3; i++)
    pos[id](4+i) = avgg(i);

  // initial covariance 
  // TODO: errors on bias offset? 
  P[id](0,0) = posrawaccerr[id](0)*posrawaccerr[id](0);
  P[id](1,1) = posrawaccerr[id](1)*posrawaccerr[id](1);
  P[id](2,2) = posrawaccerr[id](2)*posrawaccerr[id](2);
  P[id](3,3) = posrawaccerr[id](3)*posrawaccerr[id](3);
  for (int i = 0; i < 3; i++)
    P[id](4+i,4+1) = errg(i)*errg(i);
  
  // compute reference magnetic field 
  Quaternion qavga(0, avga.x(), avga.y(), avga.z());
  Quaternion qavgm(0, avgm.x(), avgm.y(), avgm.z());

  Quaternion q(pos[id](0), pos[id](1), pos[id](2), pos[id](3));

  Quaternion qra = q.conjugate() * qavga * q;
  Quaternion qrm = q.conjugate() * qavgm * q;

  qrm(0) = 0;
  qrm /= qrm.length();
  //store rm
  rm = qrm;

  printf(" reference acceleration: ");
  printvector(qra);

  printf(" reference magnetic:     ");
  printvector(qrm);

  double magang = atan(qrm(3)/sqrt(qrm(1)*qrm(1)+qrm(2)*qrm(2)))/3.14159*180;
  printf(" measured magnetic field inclination: %.2f deg", magang);
}

void IMU::read()
{
  // update data array index
  prev_id = id;
  id++;
  id = id % IMU_HISTORY;

  // read sensors
  gyro->read();
  compass->read();
  
  // store reading time
  unsigned long curr_t = millis();
  if (loop == 0)
    t[id] = 0;
  else
  {
    t[id] = curr_t - prev_t;
  }
  prev_t = curr_t;

/*
  printf("raw readings:\n");
  printf("a: %.1f %.1f %.1f\n", compass->a.x, compass->a.y, compass->a.z);
  printf("m: %.1f %.1f %.1f\n", compass->m.x, compass->m.y, compass->m.z);
  printf("g: %.1f %.1f %.1f\n", gyro->g.x, gyro->g.y, gyro->g.z);
*/
  // store sensor data in class vars

  // mag in units of earth magnetic field
  // TODO: in units of reference measured field
  m[id].x() = (compass->m.x - IMU_MAG_C_X) / IMU_MAG_S_X;
  m[id].y() = (compass->m.y - IMU_MAG_C_Y) / IMU_MAG_S_Y;
  m[id].z() = (compass->m.z - IMU_MAG_C_Z) / IMU_MAG_S_Z;
  
  // acceleration in units of g
  a[id].x() = (compass->a.x - IMU_ACC_C_X) / IMU_ACC_S_X;
  a[id].y() = (compass->a.y - IMU_ACC_C_Y) / IMU_ACC_S_Y;
  a[id].z() = -(compass->a.z - IMU_ACC_C_Z) / IMU_ACC_S_Z;

  // gyro readings in rad per sec
  g[id].x() = (float)gyro->g.x * IMU_GYRO_GAIN / 180 * 3.14159265f;
  g[id].y() = (float)gyro->g.y * IMU_GYRO_GAIN / 180 * 3.14159265f;
  g[id].z() = (float)gyro->g.z * IMU_GYRO_GAIN / 180 * 3.14159265f;

/*
  printf("adjusted readings:\n");
  printf("a: ");
  printvector(a[id]);
  printf("m: ");
  printvector(m[id]);
  printf("g: ");
  printvector(g[id]);
*/
}

Vector3D IMU::getM()
{
  return m[id];
}
Vector3D IMU::getA()
{
  return a[id];
}
Vector3D IMU::getG()
{
  return g[id];
}

Quaternion IMU::getPosRawAcc()
{
  return posrawacc[id];
}

Quaternion IMU::getPosRawGyr()
{
  return posrawgyr[id];
}

Quaternion IMU::getPosNewton()
{
  return posnwt[id];
}
Quaternion IMU::getPosRawAccError()
{
  return posrawaccerr[id];
}

Quaternion IMU::getPosRawGyrError()
{
  return posrawgyrerr[id];
}

Quaternion IMU::getPosNewtonError()
{
  return posnwterr[id];
}

VectorND<7> IMU::getPosKalman()
{
  return pos[id];
}


// execute filters. 
// assumes a[id], m[id] g[id] has been set by read()
void IMU::process()
{
  // estimate the "raw" angle from acceleration only
  //   this is completely useless but is a good reference for comparing how the second method does
  estimateAngleSimple(a[id], Vector3D(IMU_ACC_E_X, IMU_ACC_E_Y, IMU_ACC_E_Z), posrawacc[id], posrawaccerr[id]);

  // estimate the angle used as input for Kalman filter. 
  estimateAngle(/* outputs*/ posnwt[id], posnwterr[id]);

  // *************
  // KALMAN FILTER
  // *************

  // Let me first tell you how this is NOT done
  // x_n = A.x_{n-1} + B.u_n
  // x_n -- 4 quaternion components
  // u_n -- controls, (0, w_x, w_y, w_z) gyro reading
  // A   -- just the identity
  // B   -- quaternion multiplication: B.u_n = - t * 0.5 x_{n-1} ** u_n 
  //    in this way B is a function of x_n -- noise / variance.
  // 
  // use extended kalman filter approach instead:
  //   x_n = f(x_{n-1}, u_n)
  // so that after linearization:
  //   x_n = A.x_{n-1}
  // where A encodes the propagation of old x_{n-1} AND the quaternion multiplication by the gyro readings
  // this allows for reliable Q estimation



  double deltat = 0.5 * (double)t[id]/1000.0; // tack the 0.5 factor onto t to reduce mult.

  MatrixNxN<7> A = MatrixNxN<7>::identity();
  A(0, 1) =  g[id].x() * deltat;
  A(0, 2) =  g[id].y() * deltat;
  A(0, 3) =  g[id].z() * deltat;

  A(1, 0) = -g[id].x() * deltat;
  A(1, 2) = -g[id].z() * deltat;
  A(1, 3) =  g[id].y() * deltat;

  A(2, 0) = -g[id].y() * deltat;
  A(2, 1) =  g[id].z() * deltat;
  A(2, 3) = -g[id].x() * deltat;

  A(3, 0) = -g[id].z() * deltat;
  A(3, 1) = -g[id].y() * deltat;
  A(3, 2) =  g[id].x() * deltat;

  A(0, 4) = -deltat * pos[prev_id](1);
  A(0, 5) = -deltat * pos[prev_id](2);
  A(0, 6) = -deltat * pos[prev_id](3);

  A(1, 4) =  deltat * pos[prev_id](0);
  A(1, 5) = -deltat * pos[prev_id](3);
  A(1, 6) =  deltat * pos[prev_id](2);

  A(2, 4) =  deltat * pos[prev_id](3);
  A(2, 5) =  deltat * pos[prev_id](0);
  A(2, 6) = -deltat * pos[prev_id](1);

  A(3, 4) = -deltat * pos[prev_id](2);
  A(3, 5) =  deltat * pos[prev_id](1);
  A(3, 6) =  deltat * pos[prev_id](0);

  double sx2, sy2, sz2;  // gyro sigmas sq
  sx2 = IMU_GYR_E_X;
  sy2 = IMU_GYR_E_Y;
  sz2 = IMU_GYR_E_Z;
  //sx2 = sy2 = sz2 = 1;


  double factor = 0.4 * (double)t[id]/1000.0;
  sx2 *= factor;
  sy2 *= factor;
  sz2 *= factor;

  sx2 = sx2*sx2;
  sy2 = sy2*sy2;
  sz2 = sz2*sz2;

  double qw, qx, qy, qz; // tack on the factor of 0.5 here 
  qw = pos[prev_id](0) * 0.5;
  qx = pos[prev_id](1) * 0.5;
  qy = pos[prev_id](2) * 0.5;
  qz = pos[prev_id](3) * 0.5;
  
  MatrixNxN<7> Q;
  Q(0,0) =  qx*qx*sx2 + qy*qy*sy2 + qz*qz*sz2;
  Q(0,1) = -qw*qx*sx2 - qy*qz*sy2 + qy*qz*sz2;
  Q(0,2) =  qx*qz*sx2 - qw*qy*sy2 - qx*qz*sz2;
  Q(0,3) = -qx*qy*sx2 + qx*qy*sy2 - qw*qz*sz2;

  Q(1,0) = Q(0,1);
  Q(1,1) =  qw*qw*sx2 + qz*qz*sy2 + qy*qy*sz2;
  Q(1,2) = -qw*qz*sx2 + qw*qz*sy2 - qx*qy*sz2;
  Q(1,3) =  qw*qy*sx2 - qx*qz*sy2 - qw*qy*sz2;
 
  Q(2,0) = Q(0,2);
  Q(2,1) = Q(1,2);
  Q(2,2) =  qz*qz*sx2 + qw*qw*sy2 + qx*qx*sz2;
  Q(2,3) = -qy*qz*sx2 - qw*qx*sy2 + qw*qx*sz2;

  Q(3,0) = Q(0,3);
  Q(3,1) = Q(1,3);
  Q(3,2) = Q(2,3);
  Q(3,3) =  qy*qy*sx2 + qx*qx*sy2 + qw*qw*sz2;

  double sigbias = 0.000000001;
  Q(4,4) = sigbias * sigbias;
  Q(5,5) = sigbias * sigbias;
  Q(6,6) = sigbias * sigbias;

  

  // compute posrawgyr on the way
  {
    VectorND<7> temp = pos[prev_id];
    for (int i = 0; i < 4; i++)
      temp(i) = posrawgyr[prev_id](i);

    temp = A * temp;

    for (int i = 0; i < 4; i++)
    {
      posrawgyr[id](i) = temp(i);
      posrawgyrerr[id](i) = 0;
    }
    posrawgyr[id] /= posrawgyr[id].length();
  }
  

  // prediction step
  pos[id] = A * pos[prev_id];
  // cheat and normalize;
  double quatlength = 0;
  for (int i = 0; i < 4; i++)
    quatlength += pos[id](i)*pos[id](i);
  quatlength = sqrt(quatlength);
  for (int i = 0; i < 4; i++)
    pos[id](i) /= quatlength;
  // prediction covariance 
  P[id] = A * P[prev_id] * A.transpose() + Q;


  if (DEBUG)
  {
    printf("** BEFORE UPDATE **\n");
    printf("matrix A: \n");
    printmatrix(A);
    printf("matrix Q:\n");
    printmatrix(Q);
    printf("matrix P:\n");
    printmatrix(P[id]);
  }

  // update step
  Matrix4x4 R = Matrix4x4::fromVector(posnwterr[id]);
  for (int i = 0; i < 4; i++)
    R(i,i) *= R(i,i);

  MatrixNxM<4,7> H;
  for (int i = 0; i < 4; i++)
    H(i,i) = 1;

  Vector4D res = posnwt[id] - H*pos[id]; //measurement residual
  Matrix4x4 S = H*P[id]*H.transpose() + R;
  MatrixNxM<7,4> K = P[id] * H.transpose() * S.inverse();


  pos[id] = pos[id] + K * res;

  // cheat and normalize;
  quatlength = 0;
  for (int i = 0; i < 4; i++)
    quatlength += pos[id](i)*pos[id](i);
  quatlength = sqrt(quatlength);
  for (int i = 0; i < 4; i++)
    pos[id](i) /= quatlength;

  P[id] = (MatrixNxN<7>::identity() - K*H) * P[id];

  if (DEBUG)
  {
    printf("** AFTER UPDATE **\n");
    printf("matrix R:\n");
    printmatrix(R);
    printf("matrix K:\n");
    printmatrix(K);
    printf("matrix P:\n");
    printmatrix(P[id]);
  }

// **************


  loop++;
}


void IMU::estimateAngleSimple(const Vector3D &accinput, const Vector3D &accerr, Quaternion &angle, Quaternion &angleerr)
{
  angleerr = Quaternion();

  double pitch, roll;

  Vector3D acc = accinput;
  acc /= acc.length();

  pitch = asin(acc.x());
  roll = atan2(-acc.y(),acc.z());

  double sx, sy, sz; // acc errors.
  sx = accerr.x();
  sy = accerr.y();
  sz = accerr.z();
  // all errors are NOT squared
  double pitcherr1, pitcherr2, pitcherr;
  double rollerr1, rollerr2, rollerr;

  pitcherr1 = sx / sqrt(1 - acc.x()*acc.x());
  pitcherr2 = sqrt(acc.y()*acc.y()*sy*sy + acc.z()*acc.z()*sz*sz)/acc.x()/sqrt(1-acc.x()*acc.x());
  pitcherr = pitcherr1;
  if (pitcherr2 < pitcherr1)
    pitcherr = pitcherr2;

  rollerr1 = sqrt(sy*sy + sz*sz*acc.y()*acc.y()/acc.z()/acc.z())/(1+acc.y()*acc.y()/acc.z()/acc.z())/acc.z();
  rollerr2 = sqrt(sy*sy + pitcherr*pitcherr*acc.x()*acc.x()*acc.y()*acc.y()/(1-acc.x()*acc.x()))/acc.z();
  rollerr = rollerr1;
  if (rollerr2 < rollerr1)
    rollerr = rollerr2;

  double cp, sp, cr, sr;
  cp = cos(pitch/2);
  sp = sin(pitch/2);
  cr = cos(roll/2);
  sr = sin(roll/2);

  angle = Quaternion(
    cp*cr,
    cp*sr,
    sp*cr,
    sp*sr
  );

  // all errors become squared from now on.

  pitcherr *= pitcherr;
  rollerr *= rollerr;
   
  cp *= cp;
  sp *= sp;
  cr *= cr;
  sr *= sr;

  // angle err IS squared
  angleerr = Quaternion(
    pitcherr * cr * sp + rollerr  * cp * sr,
    rollerr  * cp * cr + pitcherr * sp * sr,
    pitcherr * cp * cr + rollerr  * sp * sr,
    rollerr  * cr * sp + pitcherr * cp * sr
  );  
  angleerr *= 0.25;

  // angle err NOT squared
  for (int i = 0; i < 4; i++)
    angleerr(i) = sqrt(angleerr(i));


/*
  */
}



void IMU::estimateAngle(/* outputs*/ Quaternion &angle, Quaternion &angleerr)
{
  angleerr = Quaternion(0,0,0,0);

  // compute best guess to seed the Gauss-Newton search
  //  compute change from bias-corrected gyroscope data
  Quaternion prevpos = Quaternion(pos[prev_id](0), pos[prev_id](1), pos[prev_id](2), pos[prev_id](3));
  Quaternion dposgyro = -(double)t[id] / 1000 * 0.5 * (prevpos * Quaternion(0, g[id].x() - pos[prev_id](4), g[id].y() - pos[prev_id](5), g[id].z() - pos[prev_id](6)));
  //  estimate new position:
  Quaternion q(pos[prev_id](0), pos[prev_id](1), pos[prev_id](2), pos[prev_id](3));
  q += dposgyro;

  // normalize
  q /= q.length();


  // proceed with the Gauss-Newton search

  MatrixNxM<6,4> J;
  VectorND<6> esterr;
  VectorND<4> step;
  Matrix4x4 JJ;

  // precompute commonly occuring products
  double q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3; 
  double r1r1, r1r2, r1r3, r2r2, r2r3, r3r3; 
  r1r1 = rm.x()*rm.x();
  r1r2 = rm.x()*rm.y();
  r1r3 = rm.x()*rm.z();
  r2r2 = rm.y()*rm.y();
  r2r3 = rm.y()*rm.z();
  r3r3 = rm.z()*rm.z();

  for (int iteration = 0; iteration < 4; iteration++)
  {
    // tack on a factor of 2 to q (!!!)
    q *= 2;


    J(0,0) =  q(2);
    J(1,0) = -q(1);
    J(2,0) =  q(0);
    J(3,0) =  rm.x()*q(0) + rm.z()*q(2) - rm.y()*q(3);
    J(4,0) =  rm.y()*q(0) - rm.z()*q(1) + rm.x()*q(3);
    J(5,0) =  rm.z()*q(0) + rm.y()*q(1) - rm.x()*q(2);
    J(0,1) =  q(3);
    J(1,1) = -q(0);
    J(2,1) = -q(1);
    J(3,1) =  rm.x()*q(1) + rm.y()*q(2) + rm.z()*q(3);
    J(4,1) = -J(5,0);
    J(5,1) =  J(4,0);
    J(0,2) =  q(0);
    J(1,2) =  q(3);
    J(2,2) = -q(2);
    J(3,2) =  J(5,0);
    J(4,2) =  J(3,1);
    J(5,2) = -J(3,0);
    J(0,3) =  q(1);
    J(1,3) =  q(2);
    J(2,3) =  q(3);
    J(3,3) = -J(4,0);
    J(4,3) =  J(3,0);
    J(5,3) =  J(3,1);
    // un-tack on a factor of 2 to q (!!!)
    q /= 2;


    q0q0 = q(0)*q(0);
    q0q1 = q(0)*q(1);
    q0q2 = q(0)*q(2);
    q0q3 = q(0)*q(3);
    q1q1 = q(1)*q(1);
    q1q2 = q(1)*q(2);
    q1q3 = q(1)*q(3);
    q2q2 = q(2)*q(2);
    q2q3 = q(2)*q(3);
    q3q3 = q(3)*q(3);

    esterr(0) = a[id].x() - 2*( q0q2 + q1q3);
    esterr(1) = a[id].y() - 2*(-q0q1 + q2q3);
    esterr(2) = a[id].z() - q0q0 + q1q1 + q2q2 - q3q3;
    esterr(3) = m[id].x() - 2*rm.y()*( q1q2 - q0q3) - 2*rm.z()*( q0q2 + q1q3) - rm.x()*(q0q0 + q1q1 - q2q2 - q3q3);
    esterr(4) = m[id].y() - 2*rm.x()*( q1q2 + q0q3) - 2*rm.z()*(-q0q1 + q2q3) - rm.y()*(q0q0 - q1q1 + q2q2 - q3q3);
    esterr(5) = m[id].z() - 2*rm.x()*(-q(0)*q(2) + q(1)*q(3)) - 2*rm.y()*( q(0)*q(1) + q(2)*q(3)) - rm.z()*(q(0)*q(0) - q(1)*q(1) - q(2)*q(2) + q(3)*q(3));


    JJ = J.transpose()*J; 
    //printmatrix(JJ);
    JJ = JJ.inverse();
    //printmatrix(JJ);
    step = JJ*((J.transpose())*esterr);

    double steplen = step.length();

    q = q + step;

  }
  q /= q.length();
  angle = q;

  // compute errors on the angle 

  double err2a1 = IMU_ACC_E_X * IMU_ACC_E_X;
  double err2a2 = IMU_ACC_E_Y * IMU_ACC_E_Y;
  double err2a3 = IMU_ACC_E_Z * IMU_ACC_E_Z;
  double err2m1 = IMU_MAG_E_X * IMU_MAG_E_X;
  double err2m2 = IMU_MAG_E_Y * IMU_MAG_E_Y;
  double err2m3 = IMU_MAG_E_Z * IMU_MAG_E_Z;
  
  //  ok this is just riddiculous
  angleerr(0) = 2/((8*q1q1)/err2a2 + (8*q2q2)/
     err2a1 - (4*(a[id].z() - 3*q0q0 + q1q1 + q2q2 - q3q3))/err2a3 + 
        (1/err2m1)*(-4*rm.x()*(m[id].x() + (-3*q0q0 - q1q1 + q2q2 + q3q3)*rm.x()) + 
       8*(q1q2 - 3*q0q3)*r1r2 + 
             8*q3q3*r2r2 + 
       8*(3*q0q2*rm.x()+ q1q3*rm.x()- 2*q2q3*rm.y())*rm.z()+ 8*q2q2*r3r3) + 
        (8*square(q(2)*rm.x()- q(1)*rm.y()) - 4*m[id].z()*rm.z()+ 
       8*(-3*q0q2*rm.x()+ q1q3*rm.x()+ 3*q0q1*rm.y()+ q2q3*rm.y())*rm.z()- 
             4*(-3*q0q0 + q1q1 + q2q2 - q3q3)*r3r3)/err2m3 + 
        (1/
       err2m2)*(4*(rm.y()*(-m[id].y() + 2*q1q2*rm.x()+ 3*q0q0*rm.y()- q1q1*rm.y()+ 
            q2q2*rm.y()) + 
                q3q3*(2*r1r1 - r2r2) - 6*q0q1*r2r3 + 2*q1q1*r3r3 + 
                2*q(3)*(3*q(0)*r1r2 - 2*q(1)*r1r3 + q(2)*r2r3))));

  angleerr(1) =  1/(2*((2*q0q0)/err2a2 + (2*q3q3)/
       err2a1 + (a[id].z() - q0q0 + 3*q1q1 + q2q2 - q3q3)/err2a3 + 
           (2*square(q(3)*rm.x()+ q(0)*rm.y()) + (m[id].z() + 2*q0q2*rm.x()- 6*q1q3*rm.x()- 
            6*q0q1*rm.y()- 2*q2q3*rm.y())*rm.z()- 
                (q0q0 - 3*q1q1 - q2q2 + q3q3)*r3r3)/err2m3 + 
           (1/err2m1)*((-m[id].x())*rm.x()+ q0q0*r1r1 + 3*q1q1*r1r1 - 
         q2q2*r1r1 - q3q3*r1r1 + 6*q1q2*r1r2 + 
                2*q2q2*r2r2 + 6*q1q3*r1r3 + 4*q2q3*r2r3 + 
         2*q3q3*r3r3 + 2*q(0)*rm.x()*((-q(3))*rm.y()+ q(2)*rm.z())) + 
           (1/
         err2m2)*(rm.y()*(m[id].y() - 2*q0q3*rm.x()- q0q0*rm.y()+ 3*q1q1*rm.y()+ 
            q3q3*rm.y()) + q2q2*(2*r1r1 - r2r2) + 
                6*q0q1*r2r3 + 2*q0q0*r3r3 - 
         2*q(2)*(3*q(1)*r1r2 + 2*q(0)*r1r3 + q(3)*r2r3))));

  angleerr(2) =  1/(2*((2*q0q0)/err2a1 + (2*q3q3)/
       err2a2 + (a[id].z() - q0q0 + q1q1 + 3*q2q2 - q3q3)/err2a3 + 
           (2*square(q(0)*rm.x()- q(3)*rm.y()) + (m[id].z() - 
            2*(-3*q0q2*rm.x()+ q1q3*rm.x()+ q0q1*rm.y()+ 3*q2q3*rm.y()))*rm.z()- 
                (q0q0 - q1q1 - 3*q2q2 + q3q3)*r3r3)/err2m3 + 
           (1/
         err2m2)*(rm.y()*(-m[id].y() + 
            2*q0q3*rm.x()+ (q0q0 + 3*q2q2 - q3q3)*rm.y()) + 
         q1q1*(2*r1r1 - r2r2) + 
                6*q2q3*r2r3 + 2*q3q3*r3r3 + 
         q(1)*(6*q(2)*r1r2 + 4*q(3)*r1r3 - 2*q(0)*r2r3)) + 
           (1/err2m1)*(m[id].x()*rm.x()- q1q1*r1r1 + 3*q2q2*r1r1 + q3q3*r1r1 - 
         6*q1q2*r1r2 + 2*q1q1*r2r2 - 
                2*q1q3*r1r3 + 
         2*q(0)*(q(3)*r1r2 - 3*q(2)*r1r3 + 2*q(1)*r2r3) - 
         q0q0*(r1r1 - 2*r3r3))));


  angleerr(3) =  2/((8*q1q1)/err2a1 + (8*q2q2)/
     err2a2 - (4*(a[id].z() - q0q0 + q1q1 + q2q2 - 3*q3q3))/err2a3 + 
        (8*square(q(1)*rm.x() + q(2)*rm.y()) - 
       4*(m[id].z() + 2*q0q2*rm.x() - 6*q1q3*rm.x() - 2*q0q1*rm.y() - 6*q2q3*rm.y())*rm.z() - 
             4*(-q0q0 + q1q1 + q2q2 - 3*q3q3)*r3r3)/err2m3 + 
        (1/
       err2m1)*(4*(m[id].x()*rm.x() + 
         rm.x()*((-q1q1 + q2q2 + 3*q3q3)*rm.x() - 2*q1q2*rm.y()) - 
                q0q0*(r1r1 - 2*r2r2) - 6*q1q3*r1r3 + 2*q1q1*r3r3 + 
                q(0)*(6*q(3)*r1r2 - 2*q(2)*r1r3 - 4*q(1)*r2r3))) + 
        (1/
       err2m2)*(4*(rm.y()*(m[id].y() - 
            2*q1q2*rm.x() + (q1q1 - q2q2 + 3*q3q3)*rm.y()) + 
         q0q0*(2*r1r1 - r2r2) - 
                6*q2q3*r2r3 + 2*q2q2*r3r3 + 
         q(0)*(-6*q(3)*r1r2 + 4*q(2)*r1r3 + 2*q(1)*r2r3))));

  for (int i = 0; i < 4; i++)
    angleerr(i) = sqrt(angleerr(i));
}

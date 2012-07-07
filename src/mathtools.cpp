#include "mathtools.h"

// --------------------------------------------------
//    VECTOR 3D
// --------------------------------------------------

Vector3D::Vector3D(double xx, double yy, double zz)
{
  d[0] = xx;
  d[1] = yy;
  d[2] = zz;
}

double& Vector3D::x()
{
  return d[0];
}

const double& Vector3D::x() const 
{
  return d[0];
}

double& Vector3D::y()
{
  return d[1];
}

const double& Vector3D::y() const
{
  return d[1];
}

double& Vector3D::z()
{
  return d[2];
}

const double& Vector3D::z() const 
{
  return d[2];
}

// --------------------------------------------------
//    VECTOR 4D
// --------------------------------------------------

Vector4D::Vector4D(double ww, double xx, double yy, double zz)
{
  d[0] = ww;
  d[1] = xx;
  d[2] = yy;
  d[3] = zz;
}

Vector4D::Vector4D(const VectorND<4> &v)
{
  d[0] = v(0);
  d[1] = v(1);
  d[2] = v(2);
  d[3] = v(3);
}

double& Vector4D::w()
{
  return d[0];
}

const double& Vector4D::w() const 
{
  return d[0];
}

double& Vector4D::x()
{
  return d[1];
}

const double& Vector4D::x() const 
{
  return d[1];
}

double& Vector4D::y()
{
  return d[2];
}

const double& Vector4D::y() const 
{
  return d[2];
}

double& Vector4D::z()
{
  return d[3];
}

const double& Vector4D::z() const 
{
  return d[3];
}



// --------------------------------------------------

Quaternion::Quaternion(double q0, double q1, double q2, double q3)
{
  d[0] = q0;
  d[1] = q1;
  d[2] = q2;
  d[3] = q3;
}

Quaternion::Quaternion(const VectorND<4> &v)
{
  d[0] = v(0);
  d[1] = v(1);
  d[2] = v(2);
  d[3] = v(3);
}

Quaternion::Quaternion(const Vector4D &o) : Vector4D(o.w(), o.x(), o.y(), o.z())
{
}

Quaternion::Quaternion(Vector4D &o) : Vector4D(o.w(), o.x(), o.y(), o.z())
{
}

Quaternion Quaternion::conjugate()
{
  Quaternion r;
  r(0) =  d[0];
  r(1) = -d[1];
  r(2) = -d[2];
  r(3) = -d[3];
  return r;
}

Quaternion Quaternion::operator*(const Quaternion& other)
{
  return Quaternion(
    w()*other.w() - x()*other.x() - y()*other.y() - z()*other.z(),
    w()*other.x() + x()*other.w() + y()*other.z() - z()*other.y(),
    w()*other.y() - x()*other.z() + y()*other.w() + z()*other.x(),
    w()*other.z() + x()*other.y() - y()*other.x() + z()*other.w()
  );
}

void Quaternion::operator*=(const Quaternion& other)
{
  double n0, n1, n2, n3;

  n0 = w()*other.w() - x()*other.x() - y()*other.y() - z()*other.z();
  n1 = w()*other.x() + x()*other.w() + y()*other.z() - z()*other.y();
  n2 = w()*other.y() - x()*other.z() + y()*other.w() + z()*other.x();
  n3 = w()*other.z() + x()*other.y() - y()*other.x() + z()*other.w();
  d[0] = n0;
  d[1] = n1;
  d[2] = n2;
  d[3] = n3;
}

// -----------------------------------------
//     MATRIX4X4
// -----------------------------------------

Matrix4x4::Matrix4x4() : MatrixNxN<4>() {}

Matrix4x4::Matrix4x4(const MatrixNxM<4,4> &m)
{
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      (*this)(i, j) = m(i, j);
}


void Matrix4x4::operator*=(const Matrix4x4 &o)
{
  Matrix4x4 r;
  double s = 0;
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
    {
      s = 0;
      for (int x = 0; x < 4; x++)
        s += (*this)(i, x)*o(x, j);
      r(i, j) = s;
    }
  (*this) = r;
}

/*
Matrix4x4 Matrix4x4::identity()
{
  Matrix4x4 r;
  for (int i = 0; i < 4; i ++)
    r(i,i) = 1;
  return r;
}

Matrix4x4 Matrix4x4::fromVector(const Vector4D &v)
{
  Matrix4x4 r;
  for (int i = 0; i < 4; i++)
    r(i,i) = v(i);
  return r;
}
*/


Matrix4x4 Matrix4x4::inverse()
{
  Matrix4x4 inv;

  inv(0,0) = d[1][1]  * d[2][2] * d[3][3] - 
             d[1][1]  * d[2][3] * d[3][2] - 
             d[2][1]  * d[1][2]  * d[3][3] + 
             d[2][1]  * d[1][3]  * d[3][2] +
             d[3][1] * d[1][2]  * d[2][3] - 
             d[3][1] * d[1][3]  * d[2][2];

  inv(1,0) = -d[1][0]  * d[2][2] * d[3][3] + 
              d[1][0]  * d[2][3] * d[3][2] + 
              d[2][0]  * d[1][2]  * d[3][3] - 
              d[2][0]  * d[1][3]  * d[3][2] - 
              d[3][0] * d[1][2]  * d[2][3] + 
              d[3][0] * d[1][3]  * d[2][2];

  inv(2,0) = d[1][0]  * d[2][1] * d[3][3] - 
             d[1][0]  * d[2][3] * d[3][1] - 
             d[2][0]  * d[1][1] * d[3][3] + 
             d[2][0]  * d[1][3] * d[3][1] + 
             d[3][0] * d[1][1] * d[2][3] - 
             d[3][0] * d[1][3] * d[2][1];

  inv(3,0) = -d[1][0]  * d[2][1] * d[3][2] + 
               d[1][0]  * d[2][2] * d[3][1] +
               d[2][0]  * d[1][1] * d[3][2] - 
               d[2][0]  * d[1][2] * d[3][1] - 
               d[3][0] * d[1][1] * d[2][2] + 
               d[3][0] * d[1][2] * d[2][1];

  inv(0,1) = -d[0][1]  * d[2][2] * d[3][3] + 
              d[0][1]  * d[2][3] * d[3][2] + 
              d[2][1]  * d[0][2] * d[3][3] - 
              d[2][1]  * d[0][3] * d[3][2] - 
              d[3][1] * d[0][2] * d[2][3] + 
              d[3][1] * d[0][3] * d[2][2];

  inv(1,1) = d[0][0]  * d[2][2] * d[3][3] - 
             d[0][0]  * d[2][3] * d[3][2] - 
             d[2][0]  * d[0][2] * d[3][3] + 
             d[2][0]  * d[0][3] * d[3][2] + 
             d[3][0] * d[0][2] * d[2][3] - 
             d[3][0] * d[0][3] * d[2][2];

  inv(2,1) = -d[0][0]  * d[2][1] * d[3][3] + 
              d[0][0]  * d[2][3] * d[3][1] + 
              d[2][0]  * d[0][1] * d[3][3] - 
              d[2][0]  * d[0][3] * d[3][1] - 
              d[3][0] * d[0][1] * d[2][3] + 
              d[3][0] * d[0][3] * d[2][1];

  inv(3,1) = d[0][0]  * d[2][1] * d[3][2] - 
              d[0][0]  * d[2][2] * d[3][1] - 
              d[2][0]  * d[0][1] * d[3][2] + 
              d[2][0]  * d[0][2] * d[3][1] + 
              d[3][0] * d[0][1] * d[2][2] - 
              d[3][0] * d[0][2] * d[2][1];

  inv(0,2) = d[0][1]  * d[1][2] * d[3][3] - 
             d[0][1]  * d[1][3] * d[3][2] - 
             d[1][1]  * d[0][2] * d[3][3] + 
             d[1][1]  * d[0][3] * d[3][2] + 
             d[3][1] * d[0][2] * d[1][3] - 
             d[3][1] * d[0][3] * d[1][2];

  inv(1,2) = -d[0][0]  * d[1][2] * d[3][3] + 
              d[0][0]  * d[1][3] * d[3][2] + 
              d[1][0]  * d[0][2] * d[3][3] - 
              d[1][0]  * d[0][3] * d[3][2] - 
              d[3][0] * d[0][2] * d[1][3] + 
              d[3][0] * d[0][3] * d[1][2];

  inv(2,2) = d[0][0]  * d[1][1] * d[3][3] - 
              d[0][0]  * d[1][3] * d[3][1] - 
              d[1][0]  * d[0][1] * d[3][3] + 
              d[1][0]  * d[0][3] * d[3][1] + 
              d[3][0] * d[0][1] * d[1][3] - 
              d[3][0] * d[0][3] * d[1][1];

  inv(3,2) = -d[0][0]  * d[1][1] * d[3][2] + 
               d[0][0]  * d[1][2] * d[3][1] + 
               d[1][0]  * d[0][1] * d[3][2] - 
               d[1][0]  * d[0][2] * d[3][1] - 
               d[3][0] * d[0][1] * d[1][2] + 
               d[3][0] * d[0][2] * d[1][1];

  inv(0,3) = -d[0][1] * d[1][2] * d[2][3] + 
              d[0][1] * d[1][3] * d[2][2] + 
              d[1][1] * d[0][2] * d[2][3] - 
              d[1][1] * d[0][3] * d[2][2] - 
              d[2][1] * d[0][2] * d[1][3] + 
              d[2][1] * d[0][3] * d[1][2];

  inv(1,3) = d[0][0] * d[1][2] * d[2][3] - 
             d[0][0] * d[1][3] * d[2][2] - 
             d[1][0] * d[0][2] * d[2][3] + 
             d[1][0] * d[0][3] * d[2][2] + 
             d[2][0] * d[0][2] * d[1][3] - 
             d[2][0] * d[0][3] * d[1][2];

  inv(2,3) = -d[0][0] * d[1][1] * d[2][3] + 
               d[0][0] * d[1][3] * d[2][1] + 
               d[1][0] * d[0][1] * d[2][3] - 
               d[1][0] * d[0][3] * d[2][1] - 
               d[2][0] * d[0][1] * d[1][3] + 
               d[2][0] * d[0][3] * d[1][1];

   inv(3,3) = d[0][0] * d[1][1] * d[2][2] - 
              d[0][0] * d[1][2] * d[2][1] - 
              d[1][0] * d[0][1] * d[2][2] + 
              d[1][0] * d[0][2] * d[2][1] + 
              d[2][0] * d[0][1] * d[1][2] - 
              d[2][0] * d[0][2] * d[1][1];

   double det = d[0][0] * inv(0, 0) + d[0][1] * inv(1, 0) + d[0][2] * inv(2, 0) + d[0][3] * inv(3, 0);

   if (det == 0)
     return Matrix4x4();

   det = 1.0 / det;

   inv *= det;

   return inv;
}




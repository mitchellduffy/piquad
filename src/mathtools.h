#ifndef MATHTOOLS_H
#define MATHTOOLS_H 1

#include "math.h"

template <int N>
class VectorND
{
  public: 
    VectorND();
    template <int M>
    VectorND(const VectorND<M> &v);

    double length();

    double& operator()(const int &i);
    const double& operator()(const int &i) const;

    VectorND<N> operator+(const VectorND<N>& other);
    void operator+=(const VectorND<N>& other);
    void operator+=(const double& c);

    VectorND<N> operator-(const VectorND<N>& other);
    void operator-=(const VectorND<N>& other);
    void operator-=(const double& c);

    void operator*=(const double& c);
    void operator/=(const double& c);


    template <int NN>
    friend VectorND<NN> operator+(const double& c, const VectorND<NN>& o);
    template <int NN>
    friend VectorND<NN> operator-(const double & c, const VectorND<NN> &o);
    template <int NN>
    friend VectorND<NN> operator*(const double & c, const VectorND<NN> &o);
    template <int NN>
    friend VectorND<NN> operator/(const double & c, const VectorND<NN> &o);
  protected:
    double d[N];
};

class Vector3D : public VectorND<3>
{
  public: 
    Vector3D(double x=0, double y=0, double z=0);

    double& x();
    const double& x() const;
    double& y();
    const double& y() const;
    double& z();
    const double& z() const;
};


class Vector4D : public VectorND<4>
{
  public: 
    Vector4D(double w=0, double x=0, double y=0, double z=0);
    Vector4D(const VectorND<4> &v);
    
    double& w();
    const double& w() const;
    double& x();
    const double& x() const;
    double& y();
    const double& y() const;
    double& z();
    const double& z() const;
};

class Quaternion : public Vector4D
{
  public:
    Quaternion(double q0=1, double q1=0, double q2=0, double q3=0);
    Quaternion(const Vector4D &o);
    Quaternion(Vector4D &o);
    Quaternion(const VectorND<4> &v);

    Quaternion conjugate();

    Quaternion operator*(const Quaternion& other);
    void operator*=(const Quaternion& other);

};


template <int N, int M>
class MatrixNxM
{
  public: 
    MatrixNxM();
     
    MatrixNxM<M,N> transpose();

    double& operator()(const int &x, const int &y);
    const double& operator()(const int &x, const int &y) const;
    MatrixNxM<N,M> operator+(const MatrixNxM<N,M> &o);
    void operator+=(const MatrixNxM<N,M> &o);
    MatrixNxM<N,M> operator-(const MatrixNxM<N,M> &o);
    void operator-=(const MatrixNxM<N,M> &o);

    void operator *=(const double &c);

    template <int NN, int MM, int S>
    friend MatrixNxM<NN,MM> operator*(const MatrixNxM<NN,S> &left, const MatrixNxM<S,MM> &right);

    template <int NN, int MM>
    friend VectorND<NN> operator*(const MatrixNxM<NN,MM> &m, const VectorND<MM> &v);

    //static MatrixNxM<N,M> identity();
    //static MatrixNxM<N,M> fromVector(const Vector4D &v);

  protected:
    double d[N][M];

};

template <int N>
class MatrixNxN : public MatrixNxM<N, N>
{
  public:
    MatrixNxN();
    MatrixNxN(const MatrixNxM<N,N> &m);

    static MatrixNxN<N> identity();
    static MatrixNxN<N> fromVector(const VectorND<N> &v);
};


class Matrix4x4 : public MatrixNxN<4>
{
  public: 
    Matrix4x4();
    Matrix4x4(const MatrixNxM<4,4> &m);

    Matrix4x4 inverse();

    using MatrixNxN<4>::operator*=;
    void operator *=(const Matrix4x4 &o);

    //static Matrix4x4 identity();
    //static Matrix4x4 fromVector(const Vector4D &v);
};



// --------------------------------------



template <int N>
VectorND<N>::VectorND()
{
  for (int i = 0; i < N; i++)
    d[i] = 0;
}
template <int N>
template <int M>
VectorND<N>::VectorND(const VectorND<M> &v)
{
  int m = N;
  if (M < N)
    m = M;
  
  for (int i = 0; i < N; i++)
    d[i] = 0;
  for (int i = 0; i < m; i++)
    d[i] = v(i);
}

template <int N>
double VectorND<N>::length()
{
  double s = 0;
  for (int i = 0; i < N; i++)
    s += d[i]*d[i];
  return sqrt(s);
}

template <int N>
double& VectorND<N>::operator()(const int &i)
{
  return d[i]; 
}

template <int N>
const double& VectorND<N>::operator()(const int &i) const
{
  return d[i]; 
}


template <int N>
VectorND<N> VectorND<N>::operator+(const VectorND<N>& o)
{
  VectorND<N> r;
  for (int i = 0; i < N; i++)
    r(i) = d[i] + o(i);
  return r;
}

template <int N>
VectorND<N> operator+(const double & c, const VectorND<N> &o)
{
  VectorND<N> r(o);
  for (int i = 0; i < N; i++)
    r(i) += c;
  return r;
}

template <int N>
void VectorND<N>::operator+=(const VectorND<N>& o)
{
  for (int i = 0; i < N; i++)
    d[i] += o(i);
}

template <int N>
void VectorND<N>::operator+=(const double& c)
{
  for (int i = 0; i < N; i++)
    d[i] += c;
}

template <int N>
VectorND<N> VectorND<N>::operator-(const VectorND<N>& o)
{
  VectorND<N> r;
  for (int i = 0; i < N; i++)
    r(i) = d[i] - o(i);
  return r;
}

template <int N>
VectorND<N> operator-(const double & c, const VectorND<N> &o)
{
  VectorND<N> r(o);
  for (int i = 0; i < N; i++)
    r(i) -= c;
  return r;
}

template <int N>
void VectorND<N>::operator-=(const VectorND<N>& o)
{
  for (int i = 0; i < N; i++)
    d[i] -= o(i);
}

template <int N>
void VectorND<N>::operator-=(const double& c)
{
  for (int i = 0; i < N; i++)
    d[i] -= c;
}

template <int N>
VectorND<N> operator*(const double & c, const VectorND<N> &o)
{
  VectorND<N> r(o);
  for (int i = 0; i < N; i++)
    r(i) *= c;
  return r;
}
template <int N>
void VectorND<N>::operator*=(const double& c)
{
  for (int i = 0; i < N; i++)
    d[i] *= c;
} 

template <int N>
VectorND<N> operator/(const double & c, const VectorND<N> &o)
{
  VectorND<N> r(o);
  for (int i = 0; i < N; i++)
    r(i) /= c;
  return r;
}

template <int N>
void VectorND<N>::operator/=(const double& c)
{
  if (c == 0)
    for (int i = 0; i < N; i++)
      d[i] = 0;
  else
    for (int i = 0; i < N; i++)
      d[i] /= c;
} 



// ------------------------------------
//   MATRIX NxM
// ------------------------------------


template <int N, int M>
MatrixNxM<N,M>::MatrixNxM()
{
  for (int i = 0; i < N; i++)
    for (int j = 0; j < M; j++)
      d[i][j] = 0;
}

template <int N, int M>
double& MatrixNxM<N,M>::operator()(const int &x, const int &y)
{
  return d[x][y];
}

template <int N, int M>
const double& MatrixNxM<N,M>::operator()(const int &x, const int &y) const
{
  return d[x][y];
}

template <int N, int M>
MatrixNxM<N,M> MatrixNxM<N,M>::operator+(const MatrixNxM<N,M> &o)
{
  MatrixNxM<N,M> r;
  for (int i = 0; i < N; i++)
    for (int j = 0; j < M; j++)
      r(i, j) = d[i][j]+o(i,j);
  return r;
}

template <int N, int M>
void MatrixNxM<N,M>::operator+=(const MatrixNxM<N,M> &o)
{
  for (int i = 0; i < N; i++)
    for (int j = 0; j < M; j++)
      d[i][j] += o(i,j);
}

template <int N, int M>
MatrixNxM<N,M> MatrixNxM<N,M>::operator-(const MatrixNxM<N,M> &o)
{
  MatrixNxM<N,M> r;
  for (int i = 0; i < N; i++)
    for (int j = 0; j < M; j++)
      r(i, j) = d[i][j]-o(i,j);
  return r;
}

template <int N, int M>
void MatrixNxM<N,M>::operator-=(const MatrixNxM<N,M> &o)
{
  for (int i = 0; i < N; i++)
    for (int j = 0; j < M; j++)
      d[i][j] -= o(i,j);
}


template <int N, int M, int S>
MatrixNxM<N,M> operator*(const MatrixNxM<N,S> &l, const MatrixNxM<S,M> &r)
{
  MatrixNxM<N,M> res;
  double s = 0;
  for (int i = 0; i < N; i++)
    for (int j = 0; j < M; j++)
    {
      s = 0;
      for (int x = 0; x < S; x++)
        s += l(i, x)*r(x, j);
      res(i, j) = s;
    }
  return res;
}

/*
void MatrixNxM<N,M>::operator*=(const MatrixNxM<N,M> &o)
{
  MatrixNxM<N,M> r;
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

*/

template <int N, int M>
void MatrixNxM<N,M>::operator*=(const double &c)
{
  for (int i = 0; i < N; i++)
    for (int j = 0; j < M; j++)
      d[i][j] *= c;
}

template <int N, int M>
VectorND<N> operator*(const MatrixNxM<N,M> &m, const VectorND<M> &v)
{
  VectorND<N> r;
  double s = 0;
  for (int i = 0; i < N; i++)
  {
    s = 0;
    for (int x = 0; x < M; x++)
      s += m(i, x)*v(x);
    r(i) = s;
  }
  return r;
}

/*
template <int N, int M>
MatrixNxM<N,M> MatrixNxM<N,M>::identity()
{
  MatrixNxM<N,M> r;
  for (int i = 0; i < 4; i ++)
    r(i,i) = 1;
  return r;
}

template <int N, int M>
MatrixNxM<N,M> MatrixNxM<N,M>::fromVector(const Vector4D &v)
{
  MatrixNxM<N,M> r;
  for (int i = 0; i < 4; i++)
    r(i,i) = v(i);
  return r;
}
*/

template <int N, int M>
MatrixNxM<M,N> MatrixNxM<N,M>::transpose()
{
  MatrixNxM<M,N> r;
  for (int i = 0; i < M; i++)
    for (int j = 0; j < N; j++)
      r(i, j) = (*this)(j, i);
  return r;
}


// -------------------------
//   MATRIX nXn
// -------------------------


template <int N>
MatrixNxN<N>::MatrixNxN() : MatrixNxM<N,N>() 
{
}

template <int N>
MatrixNxN<N>::MatrixNxN(const MatrixNxM<N,N> &m)
{
  for (int i = 0; i < N; i++)
    for (int j = 0; j < N; j++)
      (*this)(i,j) = m(i, j);
}

template <int N>
MatrixNxN<N> MatrixNxN<N>::identity()
{
  MatrixNxN<N> r;
  for (int i = 0; i < N; i++)
    r(i,i) = 1;
  return r;
}

template <int N>
MatrixNxN<N> MatrixNxN<N>::fromVector(const VectorND<N> &v)
{
  MatrixNxN<N> r;
  for (int i = 0; i < N; i++)
    r(i,i) = v(i);
  return r;
}

#endif

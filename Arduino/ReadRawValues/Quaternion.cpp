#include <Arduino.h>
#include "Quaternion.h"
#include <math.h>
quaternion accel_quaternion(float ax, float ay, float az)
{
  quaternion q;
  if(az >= 0)
  {
    q.q0 = sqrt((az + 1) / 2);
    q.q1 = -1 * ay / sqrt(2*(az + 1));
    q.q2 = ax / sqrt(2*(az + 1));
    q.q3 = 0;
  }
  else
  {
    q.q0 = -1 * ay / sqrt(2*(1 - az));
    q.q1 = sqrt((1 - az) / 2);
    q.q2 = 0;
    q.q3 = ax / sqrt(2*(1 - az));
  }
  return q;
}

//quaternion accel_quaternion(float ax, float ay, float az)
//{
//  float norm = sqrt(ax*ax + ay*ay + az*az);
//  norm = 1/norm;
//  ax *= norm;
//  ay *= norm;
//  az *= norm;
//
//  float angle;
//  angle = az;
//
//  float crossProd[3];
//  crossProd[0] = ay;
//  crossProd[1] = -ax;;
//  crossProd[2] = 0;
//
//  norm = sqrt(ax*ax + ay*ay);
//  norm = 1 / norm;
//  crossProd[0] *= norm;
//  crossProd[1] *= norm;
//
//  float sinHalfTheta = sin(az/2);
//
//  quaternion q;
//  q.q0 = cos(angle / 2);
//  q.q1 = crossProd[0] * sinHalfTheta;
//  q.q2 = crossProd[1] * sinHalfTheta;
//  q.q3 = crossProd[2] * sinHalfTheta;
//
//  return q;
//}

quaternion mag_quaternion(float lx, float ly, float lz)
{
  float T = pow(lx,2) + pow(ly,2);  
  quaternion q;
  if(lx >= 0)
  {
    q.q0 = sqrt(T + lx * sqrt(T)) / sqrt(2 * T);
    q.q1 = 0;
    q.q2 = 0;
    q.q3 = ly / (sqrt(2) * sqrt(T + lx * sqrt(T)));
  }
  else
  {
    q.q0 = ly / (sqrt(2) * sqrt(T - lx * sqrt(T)));
    q.q1 = 0;
    q.q2 = 0;
    q.q3 = sqrt(T - lx * sqrt(T)) / sqrt(2 * T);
  }
  return q;
}

//quaternion mag_quaternion(float mx, float my, float mz, quaternion qa)
//{
//  float norm = sqrt(mx*mx + my*my + mz*mz);
//  quaternion m;
//  quaternion qa_c;
//  
//  norm = 1/norm;
//  mx *= norm;
//  my *= norm;
//  mz *= norm;
//  
//  m.q0 = 0;
//  m.q1 = mx;
//  m.q2 = my;
//  m.q3 = mz;
//  
//  qa_c.q0 = qa.q0;
//  qa_c.q1 = -qa.q1;
//  qa_c.q2 = -qa.q2;
//  qa_c.q3 = -qa.q3;
//
//  m = quat_multiply(quat_multiply(qa, m), qa_c);
//  return m;
//}

void rotate_vector(quaternion q, float* vx, float* vy, float* vz)
{
  float lx, ly, lz;
  lx = (*vx)*(pow(q.q0,2) + pow(q.q1,2) - pow(q.q2,2) - pow(q.q3,2)) + (*vy)*(2 * (q.q1 * q.q2 - q.q0 * q.q3)) + (*vz)*(2 * (q.q1 * q.q3 - q.q0 * q.q2));
  ly = (*vx)*(2 * (q.q1 * q.q2 + q.q0 * q.q3)) + (*vy)*(pow(q.q0,2) - pow(q.q1,2) + pow(q.q2,2) - pow(q.q3,2)) + (*vz)*(2 * (q.q2 * q.q3 - q.q0 * q.q1));
  lz = (*vx)*(2 * (q.q1 * q.q3 - q.q0 * q.q2)) + (*vy)*(2 * (q.q2 * q.q3 + q.q0 * q.q1)) + (*vz)*(pow(q.q0,2) - pow(q.q1,2) - pow(q.q2,2) - pow(q.q3,2));
  *vx = lx;
  *vy = ly;
  *vz = lz;
  return;
}


quaternion quat_multiply(quaternion a, quaternion b)
{
  quaternion q;
  q.q0 = a.q0*b.q0 - a.q1*b.q1 - a.q2*b.q2 - a.q3*b.q3;
  q.q1 = a.q0*b.q1 + a.q1*b.q0 + a.q2*b.q3 - a.q3*b.q2;
  q.q2 = a.q0*b.q2 - a.q1*b.q3 + a.q2*b.q0 + a.q3*b.q1;
  q.q3 = a.q0*b.q3 + a.q1*b.q2 - a.q2*b.q1 + a.q3*b.q0;
  return q;
}


quaternion accel_mag_orientation(float ax, float ay, float az, float mx, float my, float mz)
{
  quaternion q_acc, q_mag, q_L;
  q_acc = accel_quaternion(ax, ay, az);
  rotate_vector(q_acc, &mx, &my, &mz);
  q_mag = mag_quaternion(mx, my, mz);
  q_L = quat_multiply(q_acc, q_mag);
  return q_L;
}

//void toEulerAngle(quaternion q, float* roll, float* pitch, float* yaw)
//{
//  // roll (x-axis rotation)
//  float sinr = +2.0 * (q.q0 * q.q1 + q.q2 * q.q3);
//  float cosr = +1.0 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2);
//  *roll = atan2(sinr, cosr);
//
//  // pitch (y-axis rotation)
//  float sinp = +2.0 * (q.q0 * q.q2 - q.q3 * q.q1);
//  if (fabs(sinp) >= 1)
//    *pitch = (M_PI / 2) * sinp / fabs(sinp); // use 90 degrees if out of range
//  else
//    *pitch = asin(sinp);
//
//  // yaw (z-axis rotation)
//  float siny = +2.0 * (q.q0 * q.q3 + q.q1 * q.q2);
//  float cosy = +1.0 - 2.0 * (q.q2 * q.q2 + q.q3 * q.q3);  
//  *yaw = atan2(siny, cosy);
//}

void toEulerAngle(quaternion q, float* roll, float* pitch, float* yaw)
{
  *roll = atan2(2.0 * (q.q2*q.q3 + q.q0*q.q1), 1 - 2.0 * (q.q1*q.q1 + q.q2*q.q2));
  *pitch = asin(2.0 * (q.q0*q.q2 - q.q1*q.q3));
  *yaw = atan2(2.0 * (q.q1*q.q2 + q.q0*q.q3), 1 - 2.0 * (q.q2*q.q2 + q.q3*q.q3));
}


int flag;
quaternion accel_mag_quaternion(float ax, float ay, float az, float mx, float my, float mz)
{
  quaternion qe, qr, qa, a, m, qAlpha;
  int sign;

  if (sqrt(1 - ax*ax) < 0.1)
  {
    flag = 1;
    qAlpha.q0 = cos(PI / 6);
    qAlpha.q1 = 0;
    qAlpha.q2 = sin(PI / 6);
    qAlpha.q3 = 0;
    
    a.q0 = 0;
    a.q1 = ax;
    a.q2 = ay;
    a.q3 = az;
    a = quat_multiply(quat_multiply(qAlpha,a), quat_inverse(qAlpha));
    ax = a.q1;
    ay = a.q2;
    az = a.q3;
    
    m.q0 = 0;
    m.q1 = mx;
    m.q2 = my;
    m.q3 = mz;
    m = quat_multiply(quat_multiply(qAlpha,m), quat_inverse(qAlpha));
    mx = m.q1;
    my = m.q2;
    mz = m.q3;
  }
  else
  {
    flag = 0;
  }
  float sinTheta = ax;
  float cosTheta = sqrt(1 - ax*ax);
  
  if(ax>0) sign = 1;
  else sign = -1;
  
  qe.q0 = sqrt((1 + cosTheta) / 2);
  qe.q1 = 0;
  qe.q2 = sign * sqrt((1 - cosTheta) / 2);
  qe.q3 = 0; 

  float sinPhi = -ay / cosTheta;
  float cosPhi = -az / cosTheta;

  if(sinPhi>0) sign = 1;
  else sign = -1;
  qr.q0 = sqrt((1 + cosPhi) / 2);
  qr.q1 = sign * sqrt((1 - cosPhi) / 2);
  qr.q2 = 0;
  qr.q3 = 0; 

  m.q0 = 0;
  m.q1 = mx;
  m.q2 = my;
  m.q3 = mz;
  m = quat_multiply(quat_multiply(qr, m), quat_inverse(qr));
  m = quat_multiply(quat_multiply(qe, m), quat_inverse(qe));
  mx = m.q1;
  my = m.q2;
  mz = m.q3;

  float norm = sqrt(mx*mx + my*my);
  norm = 1 / norm;
  mx *= norm;
  my *= norm;

  float sinPsi = -my;
  float cosPsi = mx;

  if(sinPsi>0) sign = 1;
  else sign = -1;
  qa.q0 = sqrt((1 + cosPsi) / 2);
  qa.q1 = 0;
  qa.q2 = 0;
  qa.q3 = sign * sqrt((1 - cosPsi) / 2); 

  quaternion q  = quat_multiply(quat_multiply(qa, qe), qr);
  if (flag == 1) return quat_multiply(q, qAlpha);
  else return q;
}

quaternion quat_inverse(quaternion q)
{
  q.q1 = -q.q1;
  q.q2 = -q.q2;
  q.q3 = -q.q3;
  return q;
}



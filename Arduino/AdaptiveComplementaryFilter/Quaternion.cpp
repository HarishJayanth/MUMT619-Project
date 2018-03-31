#include <Arduino.h>
#include "Quaternion.h"
#include <math.h>

void toEulerAngle(quaternion q, float* roll, float* pitch, float* yaw)
{
  *roll = atan2(2.0 * (q.q2*q.q3 + q.q0*q.q1), 1 - 2.0 * (q.q1*q.q1 + q.q2*q.q2));
  *pitch = asin(2.0 * (q.q0*q.q2 - q.q1*q.q3));
  *yaw = atan2(2.0 * (q.q1*q.q2 + q.q0*q.q3), 1 - 2.0 * (q.q2*q.q2 + q.q3*q.q3));
}

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

quaternion quat_inverse(quaternion q)
{
  q.q1 = -q.q1;
  q.q2 = -q.q2;
  q.q3 = -q.q3;
  return q;
}

quaternion accel_mag_quaternion(float ax, float ay, float az, float mx, float my, float mz)
{
  quaternion qe, qr, qa, a, m, qAlpha;
  float norm = 1 / sqrt(ax*ax + ay*ay + az*az);
  ax *= norm;
  ay *= norm;
  az *= norm;
  norm = 1 / sqrt(mx*mx + my*my + mz*mz);
  mx *= norm;
  my *= norm;
  mz *= norm;
  int sign;
  int flag; 
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

  norm = sqrt(mx*mx + my*my);
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

quaternion norm(quaternion q)
{
  
  float norm = 1 / sqrt(q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3);
//  if(norm == 0)
//  {
//      q.q0 = 1;
//      q.q1 = 0;
//      q.q2 = 0;
//      q.q3 = 0;
//  }
  q.q0 *= norm;
  q.q1 *= norm;
  q.q2 *= norm;
  q.q3 *= norm;
  return q;
}

quaternion scalar_mul(quaternion q, float s)
{
  q.q0 = q.q0 * s;
  q.q1 = q.q1 * s;
  q.q2 = q.q2 * s;
  q.q3 = q.q3 * s;
  return q;
}

quaternion gyr_quaternion(float gx, float gy, float gz, quaternion qTm1, float dt)
{
  quaternion qwd, qw;
  float norm = sqrt(gx*gx + gy*gy + gz*gz);
  
  qw.q0 = cos(dt / 2);
  qw.q1 = sin(dt / 2) * gx / norm;
  qw.q2 = sin(dt / 2) * gy / norm;
  qw.q3 = sin(dt / 2) * gz / norm;
//  qwd.q0 = (qTm1.q1*gx + qTm1.q2*gy + qTm1.q3*gz) * dt;
//  qwd.q1 = (- qTm1.q0*gx - qTm1.q2*gz + qTm1.q3*gy) * dt;
//  qwd.q2 = (- qTm1.q0*gy + qTm1.q1*gz - qTm1.q3*gx) * dt;
//  qwd.q3 = (- qTm1.q0*gz - qTm1.q1*gy + qTm1.q2*gx) * dt; 
//  qw = quat_add(qTm1,qwd);
//  qwd.q0 = 0;
//  qwd.q1 = 0.5 * gx * dt;
//  qwd.q2 = 0.5 * gy * dt;
//  qwd.q3 = 0.5 * gz * dt;

  qw = quat_multiply(qTm1, qw);
  return qw;  
}

quaternion quat_add(quaternion q1, quaternion q2)
{
  quaternion q;
  q.q0 = q1.q0 + q2.q0;
  q.q1 = q1.q1 + q2.q1;
  q.q2 = q1.q2 + q2.q2;
  q.q3 = q1.q3 + q2.q3;
  return q;
}



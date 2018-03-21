#ifndef QUATERNION_H
#define QUATERNION_H

struct quaternion
{
  float q0;
  float q1;
  float q2;
  float q3;
};

quaternion accel_quaternion(float ax, float ay, float az);
quaternion mag_quaternion(float lx, float ly, float lz);
void rotate_vector(quaternion q, float* vx, float* vy, float* vz);
quaternion quat_multiply(quaternion a, quaternion b);
quaternion accel_mag_orientation(float ax, float ay, float az, float mx, float my, float mz);
quaternion accel_mag_quaternion(float ax, float ay, float az, float mx, float my, float mz);
void toEulerAngle(quaternion q, float* roll, float* pitch, float* yaw);
quaternion quat_inverse(quaternion q);

#endif // QUATERNION_H

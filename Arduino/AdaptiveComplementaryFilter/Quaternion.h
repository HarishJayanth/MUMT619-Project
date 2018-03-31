#ifndef QUATERNION_H
#define QUATERNION_H

struct quaternion
{
  float q0;
  float q1;
  float q2;
  float q3;
};

void rotate_vector(quaternion q, float* vx, float* vy, float* vz);
quaternion quat_multiply(quaternion a, quaternion b);
quaternion accel_mag_quaternion(float ax, float ay, float az, float mx, float my, float mz);
quaternion gyr_quaternion(float gx, float gy, float gz, quaternion qTm1, float dt);
void toEulerAngle(quaternion q, float* roll, float* pitch, float* yaw);
quaternion quat_inverse(quaternion q);
quaternion quat_add(quaternion q1, quaternion q2);
quaternion norm(quaternion q);
quaternion scalar_mul(quaternion q, float s);


#endif // QUATERNION_H

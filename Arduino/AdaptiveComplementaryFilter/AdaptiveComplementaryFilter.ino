#include<Wire.h>
#include<math.h>
#include "Quaternion.h"
#include "MPU9250_rawRead.h"


float acc_x, acc_y, acc_z;
float gyr_x, gyr_y ,gyr_z;
float mag_x, mag_y, mag_z;
float nAcc_x, nAcc_y, nAcc_z;
float nMag_x, nMag_y, nMag_z;
float magCalibration[3] = {0, 0, 0};

float magXBias = -434.935, magYBias = 182.08, magZBias = 234.735;
float magXScale = 1.0097, magYScale = 0.9919, magZScale = 0.9984;

float t1, t2;

void printQuaternion(quaternion q)
{
  Serial.print(q.q0);
  Serial.print("\t");
  Serial.print(q.q1);
  Serial.print("\t");
  Serial.print(q.q2);
  Serial.print("\t");
  Serial.print(q.q3);
  Serial.println("\t");
}

void setup() {
  // put your setup code here, to run once:
//  Wire.begin();

  Serial.begin(57600);

  delay(1000);
  initMPU9250();
  initAK8963(magCalibration);

  pinMode(13, OUTPUT);
  digitalWrite(13,HIGH);
  t1 = micros();
}

float magMin[3] = {32768, 32768, 32768};
float magMax[3] = {-32768, -32768, -32768};
quaternion qAM, qGyr = {1, 0, 0, 0}, qAMprev = {1,0,0,0};
quaternion qOrientation = {1,0,0,0};
float k = 0.75;
void loop() {
 float roll, pitch, yaw;
 if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
  t2 = micros()-t1;
  t1 = micros();
  readIMU(&acc_x, &acc_y, &acc_z, &gyr_x, &gyr_y, &gyr_z, &mag_x, &mag_y, &mag_z); 
  mag_x = (mag_x- magXBias)*magXScale;
  mag_y = (mag_y- magYBias)*magYScale;
  mag_z = (mag_z- magZBias)*magZScale;
  qAM = accel_mag_quaternion(acc_x, acc_y, acc_z, mag_y, mag_x, -1.0 * mag_z);
  qAM = norm(qAM);
  qGyr = gyr_quaternion(gyr_x, gyr_y, gyr_z, qOrientation, t2 * pow(10,-6));
  qGyr = norm(qGyr);

//  qOrientation = norm(quat_add(scalar_mul(qAM,k),scalar_mul(qGyr, 1-k)));
  qOrientation = quat_add(scalar_mul(qAM,k),scalar_mul(qGyr, 1.0-k));

  printQuaternion(qOrientation);
//  toEulerAngle(qOrientation, &roll, &pitch, &yaw);
//  printEuler(roll, pitch, yaw);
//  toEulerAngle(qAM, &roll, &pitch, &yaw);
  Serial.print("\t");
  printQuaternion(qAM);
//  printEuler(roll, pitch, yaw);
//  toEulerAngle(qGyr, &roll, &pitch, &yaw);
  Serial.print("\t\t");
  printQuaternion(qGyr);
//  printEuler(roll, pitch, yaw);
//  if(mag_x > magMax[0]) magMax[0] = mag_x;
//  if(mag_y > magMax[1]) magMax[1] = mag_y;
//  if(mag_z > magMax[2]) magMax[2] = mag_z;
//  if(mag_x < magMin[0]) magMin[0] = mag_x;
//  if(mag_y < magMin[1]) magMin[1] = mag_y;
//  if(mag_z < magMin[2]) magMin[2] = mag_z;
//  for (int i = 0; i<3; i++)
//  {
//    Serial.print(magMin[i]);
//    Serial.print("\t");
//  }
//  for (int i = 0; i<3; i++)
//  {
//    Serial.print(magMax[i]);
//    Serial.print("\t");
//  }
//  Serial.println();
 }
  
//  printQuaternion(q);

//  toEulerAngle(q, &roll, &pitch, &yaw);
//  printEuler(roll, pitch, yaw);
//  printRawIMU();
}




void printRawIMU()
{
  Serial.print(acc_x);
  Serial.print("\t");
  Serial.print(acc_y);
  Serial.print("\t");
  Serial.print(acc_z);
  Serial.print("\t");

  Serial.print(gyr_x);
  Serial.print("\t");
  Serial.print(gyr_y);
  Serial.print("\t");
  Serial.print(gyr_z);
  Serial.print("\t");

  Serial.print(mag_x);
  Serial.print("\t");
  Serial.print(mag_y);
  Serial.print("\t");
  Serial.print(mag_z);
  Serial.println("\t");
}

void printEuler(float roll, float pitch, float yaw)
{
  Serial.print(roll * 180 / PI);
  Serial.print("\t");
  Serial.print(pitch * 180 / PI);
  Serial.print("\t");
  Serial.print((yaw * 180 / PI) + 14.39);
  Serial.println("\t");
}




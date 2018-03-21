#include<Wire.h>
#include<math.h>
#include "Quaternion.h"


float acc_x, acc_y, acc_z;
float gyr_x, gyr_y ,gyr_z;
float mag_x, mag_y, mag_z;
float nAcc_x, nAcc_y, nAcc_z;
float nMag_x, nMag_y, nMag_z;

float magXBias = 144, magYBias = -64.5, magZBias = 229;
float magXScale = 1.1511, magYScale = 0.9595, magZScale = 0.9181;

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

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
   
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
  Data[index++]=Wire.read();
}

// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

//IMUs
//=======================================================================
#define MPU9250_ADDRESS 0x68
#define MAG_ADDRESS 0x0C
 
#define GYRO_FULL_SCALE_250_DPS 0x00 
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18
 
#define ACC_FULL_SCALE_2_G 0x00 
#define ACC_FULL_SCALE_4_G 0x08
#define ACC_FULL_SCALE_8_G 0x10
#define ACC_FULL_SCALE_16_G 0x18


void setup() {
  // put your setup code here, to run once:
  Wire.begin();

  Serial.begin(57600);
  //////////////////////////////////////////////////////////////////////////
  //Initialize IMU
  Serial.println("Initializing IMU...");
  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,29,0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,26,0x06);
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_500_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_2_G);
  // Set sample rate to 100Hz
  I2CwriteByte(MPU9250_ADDRESS,0x19,0x13);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS,0x0A,0x16);
  Serial.println("Initialization Complete");
  //////////////////////////////////////////////////////////////////////////

  pinMode(13, OUTPUT);
  digitalWrite(13,HIGH);
}

float magMin[3] = {32768, 32768, 32768};
float magMax[3] = {-32768, -32768, -32768};

void loop() {
  // put your main code here, to run repeatedly:
  float roll, pitch, yaw;
  
  getRawIMU();

//  if(mag_x<magMin[0]) magMin[0] = mag_x;
//  if(mag_y<magMin[1]) magMin[1] = mag_y;
//  if(mag_z<magMin[2]) magMin[2] = mag_z;
//
//  if(mag_x>magMax[0]) magMax[0] = mag_x;
//  if(mag_y>magMax[1]) magMax[1] = mag_y;
//  if(mag_z>magMax[2]) magMax[2] = mag_z;
//
//  Serial.print("xMin: ");
//  Serial.print(magMin[0]);
//  Serial.print("\t");
//  Serial.print("xMax: ");
//  Serial.print(magMax[0]);
//  Serial.print("\t");
//
//  Serial.print("yMin: ");
//  Serial.print(magMin[1]);
//  Serial.print("\t");
//  Serial.print("yMax: ");
//  Serial.print(magMax[1]);
//  Serial.print("\t");
//  
//  Serial.print("zMin: ");
//  Serial.print(magMin[2]);
//  Serial.print("\t");
//  Serial.print("zMax: ");
//  Serial.print(magMax[2]);
//  Serial.println("\t");

//  quaternion q = accel_mag_orientation(nAcc_x, nAcc_y, nAcc_z, nMag_y, nMag_x, -1.0 * nMag_z);
  quaternion q = accel_mag_quaternion(nAcc_x, nAcc_y, nAcc_z, nMag_y, nMag_x, -1.0 * nMag_z);
//  printQuaternion(q);
  toEulerAngle(q, &roll, &pitch, &yaw);
  printEuler(roll, pitch, yaw);
//  printRawIMU();
}



void getRawIMU()
{
  uint8_t Buf[14];
  // Read accelerometer and gyroscope
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);

  float acc_scale = 16384.0;
  float gyr_scale = 65.5;
  float mag_scale = 1;
  // Create 16 bits values from 8 bits data
  // Accelerometer
  acc_x =  (int16_t)(Buf[0]<<8 | Buf[1])/acc_scale;
  acc_y =  (int16_t)(Buf[2]<<8 | Buf[3])/acc_scale;
  acc_z =  (int16_t)(Buf[4]<<8 | Buf[5])/acc_scale;

  nAcc_x = acc_x / sqrt(pow(acc_x,2) + pow(acc_y,2) + pow(acc_z,2));
  nAcc_y = acc_y / sqrt(pow(acc_x,2) + pow(acc_y,2) + pow(acc_z,2));
  nAcc_z = acc_z / sqrt(pow(acc_x,2) + pow(acc_y,2) + pow(acc_z,2));

  
  // Gyroscope
  gyr_x = (int16_t)(Buf[8]<<8 | Buf[9])/gyr_scale;
  gyr_y = (int16_t)(Buf[10]<<8 | Buf[11])/gyr_scale;
  gyr_z = (int16_t)(Buf[12]<<8 | Buf[13])/gyr_scale;

  gyr_x = gyr_x * PI / 180.0;
  gyr_y = gyr_y * PI / 180.0;
  gyr_z = gyr_z * PI / 180.0;

  uint8_t mag[7]; 
  // Read magnetometer
  // Read register Status 1 and wait for the DRDY: Data Ready
  uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  while (!(ST1&0x01));
  
  I2Cread(MAG_ADDRESS,0x03,7,mag);
  
  // Create 16 bits values from 8 bits data
  // Magnetometer
  mag_x = ((int16_t)((mag[3]<<8 | mag[2])) - magXBias) * magXScale;
  mag_y = ((int16_t)((mag[1]<<8 | mag[0])) - magYBias) * magYScale;
  mag_z = ((int16_t)((mag[5]<<8 | mag[4])) - magZBias) * magZScale;

  nMag_x = mag_x / sqrt(pow(mag_x,2) + pow(mag_y,2) + pow(mag_z,2));
  nMag_y = mag_y / sqrt(pow(mag_x,2) + pow(mag_y,2) + pow(mag_z,2));
  nMag_z = mag_z / sqrt(pow(mag_x,2) + pow(mag_y,2) + pow(mag_z,2));
}


void printRawIMU()
{
  Serial.print(nAcc_x);
  Serial.print("\t");
  Serial.print(nAcc_y);
  Serial.print("\t");
  Serial.print(nAcc_z);
  Serial.print("\t");

  Serial.print(gyr_x);
  Serial.print("\t");
  Serial.print(gyr_y);
  Serial.print("\t");
  Serial.print(gyr_z);
  Serial.print("\t");

  Serial.print(nMag_x);
  Serial.print("\t");
  Serial.print(nMag_y);
  Serial.print("\t");
  Serial.print(nMag_z);
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





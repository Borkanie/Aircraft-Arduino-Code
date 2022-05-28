//
// MPU-6500 Mahony IMU  (yaw angle is relative to starting orientation)
// last update 27/05/2022
//

#include "Wire.h"

// AD0 low = 0x68 (default for Sparkfun module)
// AD0 high = 0x69
int MPU_addr = 0x68;

// vvvvvvvvvvvvvvvvvv  VERY VERY IMPORTANT vvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//These are the previously determined offsets and scale factors for accelerometer and gyro for
// a particular example of an MPU-6050. They are not correct for other examples.
//The IMU code will NOT work well or at all if these are not correct

float A_cal[6] = {265.0, -80.0, -700.0, 0.994, 1.000, 1.014}; // 0..2 offset xyz, 3..5 scale xyz
float G_off[3] = { -499.5, -17.7, -82.0}; //raw offsets, determined for gyro at rest
#define gscale ((250./32768.0)*(PI/180.0))  //gyro default 250 LSB per d/s -> rad/s

// ^^^^^^^^^^^^^^^^^^^ VERY VERY IMPORTANT ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

// GLOBALLY DECLARED, required for Mahony filter
// vector to hold quaternion
float q[4] = {1.0, 0.0, 0.0, 0.0};

// Free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
float Kp = 30.0;
float Ki = 1.0;

// with MPU-9250, angles start oscillating at Kp=40. Ki does not seem to help and is not required.
// with MPU-6050, some instability observed at Kp=100 Now set to 30.

// char s[60]; //snprintf buffer, if needed

// globals for AHRS loop timing
unsigned long now_ms, last_ms = 0; //millis() timers

// print interval
unsigned long print_ms = 50; //print angles every "print_ms" milliseconds
float yaw, pitch, roll; //Euler angle output

void setup() {

  Wire.begin();
  Serial.begin(9600);
  Serial.println("starting");

  // initialize sensor
  // defaults for gyro and accel sensitivity are 250 dps and +/- 2 g
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

}

// AHRS loop
float* GetAngles()
{
   //raw data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
int16_t Tmp;
  //scaled data as vector
  float Axyz[3];
  float Gxyz[3];


  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  int t = Wire.read() << 8;
  ax = t | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  t = Wire.read() << 8;
  ay = t | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  t = Wire.read() << 8;
  az = t | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  t = Wire.read() << 8;
  Tmp = t | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  t = Wire.read() << 8;
  gx = t | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  t = Wire.read() << 8;
  gy = t | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  t = Wire.read() << 8;
  gz = t | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  static float response[6];
  response[0] = (float) ax;
  response[1] = (float) ay;
  response[2] = (float) az;
  response[3] = (float) gx;
  response[4] = (float) gy;
  response[5] = (float) gz;

  return response;
} 
void loop()
{
  
  float *response;
  response=GetAngles();
  float Axyz[3];
  float Gxyz[3];
  Axyz[0]=response[0] ;
  Axyz[1]=response[1] ;
  Axyz[2]=response[2] ;
  Gxyz[0]=response[3] ;
  Gxyz[1]=response[4] ;
  Gxyz[2]=response[5] ;
  now_ms = millis(); //time to print?
  if (now_ms - last_ms >= print_ms) {
    last_ms = now_ms;
    // print angles for serial plotter...
    //  Serial.print("ypr ");
    Serial.print( Axyz[0], 0);
    Serial.print(", ");
    Serial.print(Axyz[1], 0);
    Serial.print(", ");
    Serial.println(Axyz[2], 0);
  }
}

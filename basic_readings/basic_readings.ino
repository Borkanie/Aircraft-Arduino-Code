/*
  MPU-9250 Accel, Gryro and Mag Test Program

  This  program reads and prints to the Serial Monitor window
  the raw X/Y/Z values for the accelerometer, gyroscope, magnetometer
  and temperature.  It also calculates the pitch and roll values

  Connect VCC to 5V and GND to ground on the MCU
  Connect SCL to SCL and SDA to SDA on MCU

  Uses Bolder Flight MPU9250.h library
*/
#include "mpu9250.h"
using namespace bfs;
// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
Mpu9250 IMU(Wire, 0x68);
int status;
float AcX, AcY, AcZ;
float pitch, roll;
//===============================================================================
//  Initialization
//===============================================================================
void setup() {
  // serial to display data
  Serial.begin(115200);
  while (!Serial) {}

  // start communication with IMU
  status = IMU.Begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
}
//===============================================================================
//  Main
//===============================================================================
void loop() {
  IMU.Read();
  // display the data
  Serial.println("\tX\tY\tZ");
  Serial.print("Accel:\t");
  Serial.print(IMU.accel_x_mps2(), 3);
  Serial.print("\t");
  Serial.print(IMU.accel_y_mps2(), 3);
  Serial.print("\t");
  Serial.println(IMU.accel_z_mps2(), 3);

  Serial.print("Gyro:\t");
  Serial.print(IMU.gyro_x_radps(), 3);
  Serial.print("\t");
  Serial.print(IMU.gyro_y_radps(), 3);
  Serial.print("\t");
  Serial.println(IMU.gyro_z_radps(), 3);

  Serial.print("Mag:\t");
  Serial.print(IMU.mag_x_ut(), 3);
  Serial.print("\t");
  Serial.print(IMU.mag_y_ut(), 3);
  Serial.print("\t");
  Serial.println(IMU.mag_z_ut(), 3);

  Serial.print("Temp:\t");
  Serial.println(IMU.die_temp_c(), 3);

  AcX = IMU.accel_x_mps2();
  AcY = IMU.accel_x_mps2();
  AcZ = IMU.accel_x_mps2();
  //get pitch/roll
  getAngle(AcX, AcY, AcZ);

  Serial.println("\tAngle in Degrees");
  Serial.print("Pitch:\t"); 
  Serial.println(pitch, 6);
  Serial.print("Roll:\t"); 
  Serial.println(roll, 6);
  Serial.println();
  delay(250);
}
//===============================================================================
//  GetAngle - Converts accleration data to pitch & roll
//===============================================================================
void getAngle(float Vx, float Vy, float Vz) {
  float x = Vx;
  float y = Vy;
  float z = Vz;
  pitch = atan(x / sqrt((y * y) + (z * z)));
  roll = atan(y / sqrt((x * x) + (z * z)));
  //convert radians into degrees
  pitch = pitch * (180.0 / 3.14);
  roll = roll * (180.0 / 3.14) ;
}
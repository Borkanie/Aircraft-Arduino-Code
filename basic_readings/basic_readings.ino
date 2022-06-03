#include "Wire.h"

int MPU_addr = 0x68;
unsigned long now_ms, last_ms = 0; // millis() timers

// print interval
unsigned long print_ms = 100; // print angles every "print_ms" milliseconds
int Ax, Ay, Az;               // Euler angle output
int Gx, Gy, Gz;               // Euler angle output
void setup()
{

  Wire.begin();
  Serial.begin(9600);
  Serial.println("starting");

  // initialize sensor
  // defaults for gyro and accel sensitivity are 250 dps and +/- 2 g
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (akes up the MPU-6050)
  Wire.write(0x6C); // PWR_MGMT_2 register
  Wire.write(0);    // Enables all accel and gyro registers
  Wire.endTransmission(true);
}

void loop()
{
  now_ms = millis(); // time to print?
  if (now_ms - last_ms >= print_ms)
  {
    // raw data
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t Tmp;
    Wire.beginTransmission(MPU_addr);
    //Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)    
    Wire.requestFrom(MPU_addr, 14, false); // request a total of 14 registers
    int16_t t = Wire.read() << 8;
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
    static int16_t response[]{ax, ay, az, gx, gy, gz};
    last_ms = now_ms;
    // print angles for serial plotter...
    //  Serial.print("ypr ");
    Serial.print(response[0], 0);
    Serial.print(", ");
    Serial.print(response[1], 0);
    Serial.print(", ");
    Serial.print(response[2], 0);
    Serial.print(", ");
    Serial.print(response[3], 0);
    Serial.print(", ");
    Serial.print(response[4], 0);
    Serial.print(", ");
    Serial.println(response[5], 0);
    Wire.endTransmission(true);
  }
}

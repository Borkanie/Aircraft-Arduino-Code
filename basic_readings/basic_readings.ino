#include "MPU9250.h"
#include <cmath>

MPU9250 mpu; // You can also use MPU9255 as is
float dcm[3][3];

float accelVector[3][1];
const float accellOffset[]={0.1392,-0.07486,0.1308};

float ax,ay,az;
float phi,theta,psi;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    delay(2000);   
    
    mpu.setup(0x68);  // change to your own address
    //mpu.setAccBias(-0.1092*32768.0, 0.07486*32768.0, -0.1308*32768.0 );
    //mpu.setGyroBias(-4.7467 * 32768.0 / 250.0,-3.6667 * 32768.0 / 250.0,-.30567 * 32768.0 / 250.0);
    //mpu.setMagBias(0,0,0);
}



void loop() {
    if (mpu.update()) {
        
        ax=mpu.getGyroX();
        ay=mpu.getGyroY();
        az=mpu.getGyroZ();
        /*
        Serial.print(mpu.getYaw()); Serial.print(", ");
        Serial.print(mpu.getPitch()); Serial.print(", ");
        Serial.println(mpu.getRoll());*/

        accelVector[0][0]=ax;
        accelVector[1][0]=ay;
        accelVector[2][0]=az;
        phi=mpu.getQuaternionX();
        theta=mpu.getQuaternionY();
        psi=mpu.getQuaternionZ();
        //DCM(phi,theta,psi,dcm);
        //mulMat(dcm,accelVector,accelVector);
        //Serial.print(phi); Serial.print(", ");
        //Serial.print(theta); Serial.print(", ");
        //Serial.println(psi);
        // Serial.print("Rotated accel:");
        Serial.print(phi); Serial.print(", ");
        Serial.print(theta); Serial.print(", ");
        Serial.println(psi);
        //Serial.println(accelVector[0][0]*accelVector[0][0]+accelVector[1][0]*accelVector[1][0]+accelVector[2][0]*accelVector[2][0]);
    }
}
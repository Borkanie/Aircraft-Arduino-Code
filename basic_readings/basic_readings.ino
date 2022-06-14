#include "MPU9250.h"
#include <cmath>

MPU9250 mpu; // You can also use MPU9255 as is
float dcm[3][3];

float accelVector[3][1];

float ax,ay,az;
float phi,theta,psi;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    delay(2000);    
    mpu.setup(0x68);  // change to your own address
}

void DCM(float phi, float theta, float psi,float (&rslt)[3][3]) {
    phi=phi*PI/180;
    theta=theta*PI/180;
    psi=psi*PI/180;

    rslt[0][0]=(float)(cos(phi)*sin(theta));
    rslt[0][1]=(float)(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(theta));
    rslt[0][2]=(float)(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(theta));
    rslt[1][0]=(float)(sin(phi)*cos(theta));
    rslt[1][1]=(float)(sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi));
    rslt[1][2]=(float)(sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi));
    rslt[2][0]=(float)(-sin(theta));
    rslt[2][1]=(float)(cos(theta)*sin(psi));
    rslt[2][2]=(float)(cos(phi)*cos(psi));

}

void mulMat(float mat1[3][3], float mat2[3][1],float (&rslt)[3][1]) {
    rslt[2][0]=(mat1[0][0]*mat2[0][0]+mat1[0][1]*mat2[1][0]+mat1[0][2]*mat2[2][0]);
    rslt[2][1]=(mat1[1][0]*mat2[0][0]+mat1[1][1]*mat2[1][0]+mat1[1][2]*mat2[2][0]);
    rslt[2][2]=(mat1[2][0]*mat2[0][0]+mat1[2][1]*mat2[1][0]+mat1[2][2]*mat2[2][0]);
}

void loop() {
    if (mpu.update()) {
        
        ax=mpu.getAccX();
        ay=mpu.getAccY();
        az=mpu.getAccZ();
        /*
        Serial.print(mpu.getYaw()); Serial.print(", ");
        Serial.print(mpu.getPitch()); Serial.print(", ");
        Serial.println(mpu.getRoll());*/
        Serial.print(ax); Serial.print(", ");
        Serial.print(ay); Serial.print(", ");
        Serial.println(az);
        accelVector[0][0]=ax;
        accelVector[1][0]=ay;
        accelVector[2][0]=az;
        phi=mpu.getEulerX();
        theta=mpu.getEulerY();
        psi=mpu.getEulerZ();
        DCM(phi,theta,psi,dcm);
        mulMat(dcm,accelVector,accelVector);
        Serial.print(accelVector[0][0]); Serial.print(", ");
        Serial.print(accelVector[1][0]); Serial.print(", ");
        Serial.println(accelVector[2][0]);
    }
}
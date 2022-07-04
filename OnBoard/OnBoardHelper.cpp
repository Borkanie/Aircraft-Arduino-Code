#include "math.h"
#include <stdint.h>
#include "OnBoardHelper.h"
namespace OnBoardHelper
{

    void DCM(float phi, float theta, float psi, float (&rslt)[3][3])
    {
        phi = phi * pi / 180;
        theta = theta * pi / 180;
        psi = psi * pi / 180;

        rslt[0][0] = (float)(cos(phi) * sin(theta));
        rslt[0][1] = (float)(cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(theta));
        rslt[0][2] = (float)(cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(theta));
        rslt[1][0] = (float)(sin(phi) * cos(theta));
        rslt[1][1] = (float)(sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi));
        rslt[1][2] = (float)(sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi));
        rslt[2][0] = (float)(-sin(theta));
        rslt[2][1] = (float)(cos(theta) * sin(psi));
        rslt[2][2] = (float)(cos(phi) * cos(psi));
    }

    void mulMat(float mat1[3][3], float mat2[3][1], float (&rslt)[3][1])
    {
        rslt[2][0] = (mat1[0][0] * mat2[0][0] + mat1[0][1] * mat2[1][0] + mat1[0][2] * mat2[2][0]);
        rslt[2][1] = (mat1[1][0] * mat2[0][0] + mat1[1][1] * mat2[1][0] + mat1[1][2] * mat2[2][0]);
        rslt[2][2] = (mat1[2][0] * mat2[0][0] + mat1[2][1] * mat2[1][0] + mat1[2][2] * mat2[2][0]);
    }

    AircraftConfiguration::AircraftConfiguration(uint32_t motorPin,uint32_t elevatorPin1,uint32_t elevatorPin2, uint32_t rudderPin, uint32_t aileronLeftPin, uint32_t aileronRightPin)
    {
        this->MotorPin = motorPin;
        this->ElevatorPin1 = elevatorPin1;
        this->ElevatorPin2 = elevatorPin2;
        this->RudderPin = rudderPin;
        this->AileronLeftPin = aileronLeftPin;
        this->AileronRightPin = aileronRightPin;
    }

}
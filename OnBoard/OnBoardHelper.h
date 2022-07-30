#include <stdint.h>
#define pi 3.14159265358979323846 /* pi */
#pragma once
namespace OnBoardHelper
{
    enum States
    {
        SETUP = 44,
        NORMAL = 46,
        INDEPENDENT = 48,
        ERROR = 50
    };
    void DCM(float phi, float theta, float psi, float (&rslt)[3][3]);

    void mulMat(float mat1[3][3], float mat2[3][1], float (&rslt)[3][1]);
    class AircraftConfiguration
    {
    private:
    public:
        AircraftConfiguration(uint32_t motorPin, uint32_t elevatorPin1, uint32_t elevatorPin2, uint32_t rudderPin, uint32_t aileronLeftPin, uint32_t aileronRightPin);
        uint32_t MotorPin;
        uint32_t ElevatorPin1;
        uint32_t ElevatorPin2;
        uint32_t RudderPin;
        uint32_t AileronLeftPin;
        uint32_t AileronRightPin;
    };
}
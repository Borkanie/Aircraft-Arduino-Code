#include <stdint.h>
#define pi 3.14159265358979323846 /* pi */
#pragma once
namespace OnBoardHelper
{
    // All possible operating states of the aircraft.
    enum States
    {
        SETUP = 44,
        NORMAL = 46,
        INDEPENDENT = 48,
        ERROR = 50
    };
    // Calculates the direct cosine matrix for the desired angles in radions and saves them in the rslt parameter.
    void DCM(float phi, float theta, float psi, float (&rslt)[3][3]);
    // The mapping of the physical implementation of the aircraft.
    class AircraftConfiguration
    {
    private:
    public:
        AircraftConfiguration(uint32_t motorPin, uint32_t elevatorPin1, uint32_t elevatorPin2, uint32_t rudderPin, uint32_t aileronLeftPin, uint32_t aileronRightPin);
        // The pin that will send commands to thew ESC of the motor.
        uint32_t MotorPin;
        // The pin that will control the left elevator surface.
        uint32_t ElevatorPin1;
        // The pin that will control the right elevator surface.
        uint32_t ElevatorPin2;
        // The pin that will control the rudder surface.
        uint32_t RudderPin;
        // The pin that will control the left aileron surface.
        uint32_t AileronLeftPin;
        // The pin that will control the right aileron surface.
        uint32_t AileronRightPin;
    };
}
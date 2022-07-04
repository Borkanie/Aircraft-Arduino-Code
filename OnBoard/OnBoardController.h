#include "OnBoardHelper.h"
#include "MPU9250.h"
#include "TinyGPS.h"
#include "Arduino.h"
#include "SPI.h"
#include "RF24.h"
#include <stdint.h>
#include "Estimator.h"
#include "MissionControl.h"
#include "Servo.h"
#pragma once

// instantiate an object for the nRF24L01 transceiver
namespace OnBoard
{  
    // linearized form of motor:
    // F=ThrustCoeff[0]*PWM+ThrustCoeff[1]
    const float ThrustCoeff[2]={ 0.0328, -0.0813};
    // We expect to get the deflection angle of the surfaces in radians. 
    // We convert them to pwm for the servomotor.
    int GetAngleFromDu(float radians,int offset=0);
    //uint8_t Rad2PWM(float radians, float offset = 0.4668);
    // Converts thrust from newtons to PWM for motor.
    uint8_t ThrustToPwm(float thrust);
    class Controller : OnBoardHelper::AircraftConfiguration
    {
    private:
        Servo AileronRight;
        Servo AileronLeft;
        Servo Elevator1;
        Servo Elevator2;
        Servo Rudder;
    //this will gives us the position error to the current mission
        MissionControl::SquareMission Mission;
        //this will flag if we reset mission
        bool newMission=true;
        float latitude = 46.786;
        float lon = 23.592;
        float lastRead[3] = {5, 5, 5};
        float newRead[3] = {6, 6, 6};
        float value = 0.f;
        const float tolerance = 0.1;
        OnBoardHelper::States state;
        MPU9250 mpu; // create mpu object
        TinyGPS gps; // create gps object
        bool PcSerial = false;
        int ControlInputs[5] = {0, 0, 0, 0, 0};
        RF24 radio; // using pin 7 for the CE pin, and pin 8 for the CSN pin
        uint8_t address[1][6] = {"1Node"};
        bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
        bool role = false;    // true = TX role, false = RX role
        // default payload value
        uint16_t payload[5] = {1, 50, 100, 150, 150};
        bool SelfControl = false;
        void InitializeRadioReciever();
        void InitializeGPS();
        void InitializeMPU();
        void InitializeSerial();
        void SerialPrintLn(String text);
        void WriteCurrentReadings();
        bool ReadRadio();
        void ChangeState(OnBoardHelper::States newState);
        void InterpretCommand();
        bool ReadDifference();
        void CalculateDiff();
        bool ReadMPU();
        void ReadGPS();
        void InitializeControllers();
        Estimator::FixedSizeSystem Kalman;
        Estimator::PIDz OyController;
        Estimator::PIDz OzController;
        Estimator::Integrator TranslationalVelocities;
        Estimator::Integrator RotationalVelocities;
    public:
        Controller(uint32_t motorPin, uint32_t elevatorPin1,uint32_t elevatorPin2, uint32_t rudderPin, uint32_t aileronLeftPin, uint32_t aileronRightPin);
        void Setup(bool serial);
        void InterpretComand();
        void ReadDataFromSensors();
        Estimator::Matrix CalculateControl();
    };
}
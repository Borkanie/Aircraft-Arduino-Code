#include "OnBoardHelper.h"
#include "MPU9250.h"
#include "TinyGPS.h"
#include "Arduino.h"
#include "SPI.h"
#include "RF24.h"
#include <stdint.h>
#pragma once
// instantiate an object for the nRF24L01 transceiver
namespace OnBoard
{
    class Controller : OnBoardHelper::AircraftConfiguration
    {
    private:
        float lastRead[3]={5,5,5};
        float newRead[3]={6,6,6};
        float value=0.f;
        const float tolerance=0.1;
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
        uint8_t payload[6] = {1, 50, 100, 150, 150};
        bool SelfControl = false;
        void InitializeRadioReciever();
        void InitializeGPS();
        void InitializeMPU();
        void InitializeSerial();
        void SerialPrintLn(String text);
        void WriteCurrentReadings();
        bool ReadRadio();
        void ChangeState(OnBoardHelper::States newState);
        void InterpretPayload();
        bool ReadDifference();
        void CalculateDiff();
        bool ReadMPU();
    public:
        Controller(uint32_t motorPin, uint32_t elevatorPin, uint32_t rudderPin, uint32_t aileronLeftPin, uint32_t aileronRightPin);
        void Setup(bool serial);

        void CalculateNewControl();

        void ReadDataFromSensors();
    };
}
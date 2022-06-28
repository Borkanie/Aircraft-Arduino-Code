#include "RF24.h"
#include "Arduino.h"
#pragma once

namespace Controller
{     
volatile static bool AutonomosMode=false;
void ChangeMode();
    enum ControlPins{
        Mode=2,
        Motor=26,
        Elevator=27,
        Rudder=28,
        Aileron=29
    };
    enum FlagPins{
        Normal=6,
        Error=7
    };
    class Controller
    {
    private:
        uint8_t address[1][6] = {"1Node"};
        bool value=false;
        bool PcSerial=true;
        RF24 radio;
        int MotorValue;
        int ElevatorValue;
        int RudderValue;
        int AileronValue;
        FlagPins STATE;
        uint16_t payload[5] = {0, 0, 0, 0, 0};
        void SetupRadio();
        void SetupSerial();        
    public:
        void SerialPrintLn(float text);
        void SerialPrintLn(String text);
        void Setup(bool serial);
        void Read();
        void Transmit();   
        void ChangeState(FlagPins pin);
        void ClearRadio();     
    };
    static Controller controller;
}

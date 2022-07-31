#include "RF24.h"
#include "Arduino.h"
#pragma once

namespace Controller
{
    // Flags manual control mode on contrller.
    volatile static bool Manual = true;
    // The frequency at which serial will be used to communicate with testing computer.
    const int SerialFrequency = 9600;
    // Changes the aircraft mode from autonomous to manual or from mannual to autonomous.
    // It will also change the controller state to normal.
    // Will print to serial if available.
    void ChangeMode();
    enum ControlPins
    {
        Mode = 2,
        Motor = 29,
        Elevator = 27,
        Rudder = 28,
        Aileron = 26
    };
    enum FlagPins
    {
        Normal = 6,
        Error = 7
    };
    class Controller
    {
    private:
        uint8_t address[1][6] = {"1Node"};
        bool value = false;
        bool PcSerial = true;
        RF24 radio;
        int MotorValue;
        int ElevatorValue;
        int RudderValue;
        int AileronValue;
        FlagPins STATE;
        // This method sets up radio module on PINS CE=8 CS=9 as transmitter with 15
        // It is set up as transmitter.
        // PALevel is set to low.
        // Payload size is set to sizeof(payload).
        // Sets up radio in tramsitter mode.
        void SetupRadio();
        // Sets up serial communication on frequency of SerialFrequency.
        void SetupSerial();

    public:
        uint16_t payload[5] = {0, 0, 0, 0, 0};
        void SerialPrintLn(float text);
        void SerialPrintLn(String text);
        // Initializes the controller, sets up all pin modes and if succesfull sets status to normal.
        // Sets up Normal and Error pins to OUTPUT.
        // Sets up Motor,Elevator,Rudder and Aileron pins as inputs.
        // Sets up serial communitacion.
        // Sets up redio.
        // @param serial It sets tells the controller if he should write on Serial.
        void Setup(bool serial);
        /**
         *  Reads all the controller inputs from potentionemers using analogread.
         *  If the controller is in autonomous mode than no readings are done and all inputs are set to 0.
         */
        void Read();
        /**
         * Tries to transmit the payload using radio.
         * If the transmission ius succesfull mode is set to normal else it is set to Error.
         * Transmitter buffer get's cleared.
         */
        void Transmit();
        // Changes current controller state.
        // @param pin The state that is desired.
        void ChangeState(FlagPins pin);
        // This method will flush the transmitter FIFO.
        void ClearRadio();
    };
    static Controller controller;
}

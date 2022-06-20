#include "OnBoardHelper.h"
#include "OnBoardController.h"
#include "MPU9250.h"
#include "TinyGPS.h"
#include "Arduino.h"
#include "SPI.h"
#include "RF24.h"
#include <stdint.h>
#include <string> // std::string, std::to_string
// instantiate an object for the nRF24L01 transceiver
namespace OnBoard
{
    void Controller::InitializeRadioReciever()
    {
        SerialPrintLn("Started Radio set up");
        ChangeState(OnBoardHelper::ERROR);
        this->radio = RF24(30, 31);
        // initialize the transceiver on the SPI bus
        while (!this->radio.begin())
        {
            SerialPrintLn("radio hardware is not responding!!");
           delay(1000);
        }
        SerialPrintLn("Done with Radio set up");
        ChangeState(OnBoardHelper::SETUP);
    }
    void Controller::InitializeGPS()
    {
        SerialPrintLn("Started GPS set up");
        ChangeState(OnBoardHelper::ERROR);
        Serial1.begin(9600); // connect gps sensor
        while (!Serial1)
        {
            delay(100);
            SerialPrintLn("Waiting for gps...");
        }
        while (!gps.encode(Serial1.read())) // encode gps data
        {
            delay(1000);
            SerialPrintLn("Waiting for satellites...");
        }
        SerialPrintLn("Done with GPS set up");
        ChangeState(OnBoardHelper::SETUP);
    }
    void Controller::InitializeMPU()
    {
        SerialPrintLn("Started MPU set up");
        Wire.begin();
        delay(2000);
        ChangeState(OnBoardHelper::ERROR);
        this->mpu.setup(0x68);
        while (!this->mpu.available())
        {
            delay(10);
        }

        this->newRead[0] = this->mpu.getQuaternionX();
        this->newRead[1] = this->mpu.getQuaternionY();
        this->newRead[2] = this->mpu.getQuaternionZ();
        SerialPrintLn("Waiting for MPU calibration:");
        while (!ReadDifference())
        {
            delay(100);
            //Serial.println(this->value);
        }        
        WriteCurrentReadings();
        SerialPrintLn("Done with MPU set up");
        ChangeState(OnBoardHelper::SETUP);
    }
    bool Controller::ReadDifference()
    {
        if (this->ReadMPU())
        {
            this->CalculateDiff();
            return (this->value <= this->tolerance) && (this->value > 0.003f);
        }
        return false;
    }
    bool Controller::ReadMPU()
    {
        if (this->mpu.update() && this->mpu.isConnected())
        {
            this->lastRead[0] = this->newRead[0];
            this->lastRead[1] = this->newRead[1];
            this->lastRead[2] = this->newRead[2];
            WriteCurrentReadings();
            this->newRead[0] = this->mpu.getQuaternionX();
            this->newRead[1] = this->mpu.getQuaternionY();
            this->newRead[2] = this->mpu.getQuaternionZ();
            return true;
        }
        return false;
    }
    void Controller::CalculateDiff()
    {
        this->value = (this->lastRead[0] - this->newRead[0]) * (this->lastRead[0] - this->newRead[0]) 
        + (this->lastRead[1] - this->newRead[1]) * (this->lastRead[1] - this->newRead[1])
         + (this->lastRead[2] - this->newRead[2]) * (this->lastRead[2] - this->newRead[2]);
    }

    void Controller::InitializeSerial()
    {
        if (this->PcSerial)
        {
            Serial.begin(9600);
            while (!Serial)
            {
                delay(100);
            }
            this->SerialPrintLn("The aircraft system is starting up");
        }
    }
    void Controller::SerialPrintLn(String text)
    {
        if (this->PcSerial)
        {
            Serial.println(text);
        }
    }
    void Controller::WriteCurrentReadings()
    {
        if (this->PcSerial)
        {
            Serial.print("Current values are:");
            Serial.print(this->lastRead[0]);
            Serial.print(",");
            Serial.print(this->lastRead[1]);
            Serial.print(",");
            Serial.println(this->lastRead[2]);
        }
    }
    bool Controller::ReadRadio()
    {
        uint8_t pipe;
        bool available = radio.available(&pipe);
        if (available)
        {                                           // is there a payload? get the pipe number that recieved it
            uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
            radio.read(&payload, bytes);            // fetch payload from FIFO
        }
        return available;
    }
    void Controller::ChangeState(OnBoardHelper::States newState)
    {
        digitalWrite(this->state, 0);
        this->state = newState;
        digitalWrite(this->state, 1);
    }
    void Controller::InterpretPayload()
    {
        if (payload[0] > 0)
        {
            ChangeState(OnBoardHelper::INDEPENDENT);
            ReadDataFromSensors();
            CalculateNewControl();
        }
        else
        {
            ChangeState(OnBoardHelper::NORMAL);
            analogWrite(MotorPin, payload[1] * 4);        // Thrust set pwm
            analogWrite(ElevatorPin, payload[2] * 4);     // Elevator set pwm
            analogWrite(RudderPin, payload[3] * 4);       // Rudder set pwm
            analogWrite(AileronLeftPin, payload[4] * 4);  // Aileron Left set pwm
            analogWrite(AileronRightPin, payload[5] * 4); // Aileron Right set pwm
        }
    }

    Controller::Controller(uint32_t motorPin, uint32_t elevatorPin, uint32_t rudderPin, uint32_t aileronLeftPin, uint32_t aileronRightPin) : AircraftConfiguration(motorPin, elevatorPin, rudderPin, aileronLeftPin, aileronRightPin)
    {
    }

    void Controller::Setup(bool serial)
    {
        pinMode(OnBoardHelper::SETUP, OUTPUT);
        pinMode(OnBoardHelper::NORMAL, OUTPUT);
        pinMode(OnBoardHelper::INDEPENDENT, OUTPUT);
        pinMode(OnBoardHelper::ERROR, OUTPUT);
        ChangeState(OnBoardHelper::SETUP);
        pinMode(this->MotorPin, OUTPUT);
        pinMode(this->ElevatorPin, OUTPUT);
        pinMode(this->RudderPin, OUTPUT);
        pinMode(this->AileronLeftPin, OUTPUT);
        pinMode(this->AileronRightPin, OUTPUT);
        this->PcSerial = serial;
        InitializeSerial();
        InitializeMPU();
        InitializeRadioReciever();
        //InitializeGPS();
        
    }

    void Controller::CalculateNewControl()
    {
    }

    void Controller::ReadDataFromSensors()
    {
    }

}
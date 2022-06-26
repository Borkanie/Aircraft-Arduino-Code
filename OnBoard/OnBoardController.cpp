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
        // Set the PA Level low to try preventing power supply related problems
        // because these examples are likely run with nodes in close proximity to
        // each other.
        radio.setPALevel(RF24_PA_MAX); // RF24_PA_MAX is default.

        // save on transmission time by setting the radio to only transmit the
        // number of bytes we need to transmit a float
        radio.setPayloadSize(sizeof(payload)); // float datatype occupies 4 bytes

        // set the RX address of the TX node into a RX pipe
        radio.openReadingPipe(1, address[0]); // using pipe 1
        radio.startListening();               // put radio in RX mode
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
            // Serial.println(this->value);
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
            // WriteCurrentReadings();
            this->newRead[0] = this->mpu.getQuaternionX();
            this->newRead[1] = this->mpu.getQuaternionY();
            this->newRead[2] = this->mpu.getQuaternionZ();
            return true;
        }
        return false;
    }
    void Controller::CalculateDiff()
    {
        this->value = (this->lastRead[0] - this->newRead[0]) * (this->lastRead[0] - this->newRead[0]) + (this->lastRead[1] - this->newRead[1]) * (this->lastRead[1] - this->newRead[1]) + (this->lastRead[2] - this->newRead[2]) * (this->lastRead[2] - this->newRead[2]);
    }
    void Controller::InitializeSerial()
    {
        if (this->PcSerial)
        {
            ChangeState(OnBoardHelper::ERROR);
            Serial.begin(9600);
            while (!Serial)
            {
                delay(100);
            }
            this->SerialPrintLn("The aircraft system is starting up");
            ChangeState(OnBoardHelper::SETUP);
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
        if (available) // is there a payload? get the pipe number that recieved it
        {
            radio.read(&payload, radio.getPayloadSize()); // fetch payload from FIFO
        }
        return available;
    }
    void Controller::ChangeState(OnBoardHelper::States newState)
    {
        if (this->state != newState)
        {
            digitalWrite(this->state, 0);
            this->state = newState;
            digitalWrite(this->state, 1);
        }
    }
    void Controller::InterpretPayload()
    {
        if (payload[0] > 0)
        {
            ChangeState(OnBoardHelper::INDEPENDENT);
            ReadDataFromSensors();
            InterpretComand();
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
        delay(500);
        ChangeState(OnBoardHelper::INDEPENDENT);
        delay(500);
        ChangeState(OnBoardHelper::NORMAL);
        delay(500);
        ChangeState(OnBoardHelper::ERROR);
        delay(500);
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
        // InitializeGPS();
        ChangeState(OnBoardHelper::NORMAL);
    }

    void Controller::InterpretComand()
    {
        if (ReadRadio())
        {
            Serial.print("Recieved message was:");
            Serial.print(payload[0]);
            Serial.print(",");
            Serial.print(payload[1]);
            Serial.print(",");
            Serial.print(payload[2]);
            Serial.print(",");
            Serial.print(payload[3]);
            Serial.print(",");
            Serial.println(payload[4]);
            if (payload[0] != 1)
            {
                ChangeState(OnBoardHelper::INDEPENDENT);
                // SerialPrintLn("AUTONOMOUS");
                CalculateDiscreteController();
                FullStateFeedBackControl();
            }
            else
            {
                ChangeState(OnBoardHelper::NORMAL);
                // SerialPrintLn("MANUAL");
                analogWrite(this->MotorPin, 20);         // this->payload[1]);
                analogWrite(this->ElevatorPin, 100);     // this->payload[2]);
                analogWrite(this->RudderPin, 125);       // this->payload[3]);
                analogWrite(this->AileronLeftPin, 200);  // this->payload[4]);
                analogWrite(this->AileronRightPin, 255); // 1034 - this->payload[4]);
            }
        }
        else
        {
            // SerialPrintLn("MANUAL");
            ChangeState(OnBoardHelper::ERROR);
            analogWrite(this->MotorPin, 20);         // this->payload[1]);
            analogWrite(this->ElevatorPin, 100);     // this->payload[2]);
            analogWrite(this->RudderPin, 125);       // this->payload[3]);
            analogWrite(this->AileronLeftPin, 200);  // this->payload[4]);
            analogWrite(this->AileronRightPin, 255); // 1034 - this->payload[4]);
        }
    }

    void Controller::ReadDataFromSensors()
    {
        ReadMPU();
        ReadRadio();
        ReadGPS();
        float states[9];
        // this->Kalman.DoKalmanAlgorithm(states);
    }

    void Controller::FullStateFeedBackControl()
    {
    }

    void Controller::CalculateDiscreteController()
    {
    }

    void Controller::ReadGPS()
    {
    }
}
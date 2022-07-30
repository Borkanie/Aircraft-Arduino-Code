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
        radio.setPALevel(RF24_PA_LOW); // RF24_PA_MAX is default.

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

        gps.f_get_position(&latitude, &lon);
        // get latitude and longitude
        // display position

        // latitude = earthRadius * CosineInCluj(latitude) * CosineInCluj(latitude);
        // lon = earthRadius * CosineInCluj(lon) * CosineInCluj(lon);

        SerialPrintLn("Done with GPS set up");
        ChangeState(OnBoardHelper::SETUP);
    }
    void Controller::InitializeMPU()
    {
        SerialPrintLn("Started MPU set up");
        Wire.begin();
        delay(2000);
        ChangeState(OnBoardHelper::ERROR);
        this->mpu.setup(0x69);
        while (!this->mpu.available())
        {
            delay(10);
            this->mpu.setup(0x68);
        }

        this->newRead[0] = this->mpu.getEulerX();
        this->newRead[1] = this->mpu.getEulerY();
        this->newRead[2] = this->mpu.getEulerZ();
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
            // Serial.println(this->value);
            return (this->value <= this->tolerance) && (this->value > 0.003f);
        }
        return false;
    }
    bool Controller::ReadMPU()
    {
        if (this->mpu.update() && this->mpu.isConnected())
        {
            Estimator::Matrix translationalVelocties(3, 1);
            // convert from g to m/s^2
            translationalVelocties.matrix[0][0] = mpu.getAccX() / 9.81;
            translationalVelocties.matrix[1][0] = mpu.getAccY() / 9.81;
            translationalVelocties.matrix[2][0] = mpu.getAccZ() / 9.81;
            TranslationalVelocities.ReadData(translationalVelocties);
            //
            Estimator::Matrix rotationalVelocties(3, 1);
            rotationalVelocties.matrix[0][0] = mpu.getGyroX() * PI / 180;
            rotationalVelocties.matrix[1][0] = mpu.getGyroY() * PI / 180;
            rotationalVelocties.matrix[2][0] = mpu.getGyroZ() * PI / 180;
            RotationalVelocities.ReadData(rotationalVelocties);

            this->lastRead[0] = this->newRead[0];
            this->lastRead[1] = this->newRead[1];
            this->lastRead[2] = this->newRead[2];
            // WriteCurrentReadings();
            this->newRead[0] = this->mpu.getEulerX() * PI / 180;
            this->newRead[1] = this->mpu.getEulerY() * PI / 180;
            this->newRead[2] = this->mpu.getEulerZ() * PI / 180;
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

    Controller::Controller(uint32_t motorPin, uint32_t elevatorPin1, uint32_t elevatorPin2, uint32_t rudderPin, uint32_t aileronLeftPin, uint32_t aileronRightPin) : AircraftConfiguration(motorPin, elevatorPin1, elevatorPin2, rudderPin, aileronLeftPin, aileronRightPin)
    {
    }

    void Controller::InitializeControllers()
    {
        OyController = Estimator::PIDz(0, 0.002, 0.0001);
        OzController = Estimator::PIDz(0, 0.003, 0.0001);
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

        Motor.attach(this->MotorPin, 1000, 2000);
        Motor.write(0);
        Elevator1.attach(this->ElevatorPin1);
        // pinMode(this->ElevatorPin, OUTPUT);
        Elevator2.attach(this->ElevatorPin2);
        // pinMode(this->RudderPin, OUTPUT);
        Rudder.attach(this->RudderPin);
        // pinMode(this->AileronLeftPin, OUTPUT);
        AileronLeft.attach(this->AileronLeftPin);
        // pinMode(this->AileronRightPin, OUTPUT);
        AileronRight.attach(this->AileronRightPin);
        this->PcSerial = serial;
        InitializeSerial();
        InitializeMPU();
        InitializeRadioReciever();
        // InitializeGPS();
        if (SimpleMission)
        {
            for (int i = 0; i < 5; i++)
            {
                Kalman.Inputs.matrix[i][0] = 0;
            }
            for (int i = 0; i < 6; i++)
            {
                Kalman.States.matrix[i][0] = 0;
            }
            ReadMPU();
            for (int i = 6; i < 9; i++)
            {
                Kalman.States.matrix[i][0] = newRead[i - 6];
            }
        }
        ChangeState(OnBoardHelper::NORMAL);
        InitializeControllers();
    }

    void Controller::InterpretComand()
    {
        if (ReadRadio())
        {
            if (payload[0] != 1)
            {
                radio.flush_rx();
                ChangeState(OnBoardHelper::INDEPENDENT);
                // SerialPrintLn("AUTONOMOUS");
                // create new mission
                if (newMission)
                {
                    newMission = false;
                    if (SimpleMission)
                    {
                        for (int i = 0; i < 5; i++)
                        {
                            Kalman.Inputs.matrix[i][0] = 0;
                        }
                        for (int i = 0; i < 6; i++)
                        {
                            Kalman.States.matrix[i][0] = 0;
                        }
                        ReadMPU();
                        for (int i = 6; i < 9; i++)
                        {
                            Kalman.States.matrix[i][0] = newRead[i - 6];
                        }
                        WriteCurrentReadings();
                    }
                    else
                    {
                        Mission = MissionControl::SquareMission(latitude, lon);
                    }
                }
                Estimator::Matrix control = CalculateControl();
                // we need some mapping
                Motor.write(GetAngleFromDu(control.matrix[1][0], 150));
                Elevator1.write(GetAngleFromDu(control.matrix[2][0], 90));
                Elevator2.write(GetAngleFromDu(-control.matrix[2][0], 90));
                Rudder.write(GetAngleFromDu(-control.matrix[3][0], 110));
                AileronLeft.write(GetAngleFromDu(control.matrix[4][0], 65));
                AileronRight.write(GetAngleFromDu(control.matrix[4][0], 90));
            }
            else
            {
                radio.flush_rx();
                ChangeState(OnBoardHelper::NORMAL);
                newMission = true;
                // SerialPrintLn("MANUAL");
                InterpretRadioCommand();
            }
        }
        else
        {
            radio.flush_rx();
            ChangeState(OnBoardHelper::ERROR);

            if (payload[0] != 1)
            {
                // SerialPrintLn("AUTONOMOUS");
                // create new mission
                if (newMission)
                {
                    Mission = MissionControl::SquareMission(latitude, lon);
                }
                Estimator::Matrix control = CalculateControl();
                // we need some mapping
                Motor.write(GetAngleFromDu(control.matrix[1][0], 135));
                Elevator1.write(GetAngleFromDu(control.matrix[2][0], 90));
                Elevator2.write(GetAngleFromDu(-control.matrix[2][0], 90));
                Rudder.write(GetAngleFromDu(-control.matrix[3][0], 110));
                AileronLeft.write(GetAngleFromDu(control.matrix[4][0], 65));
                AileronRight.write(GetAngleFromDu(control.matrix[4][0], 90));
            }
            else
            {
                radio.flush_rx();
                // SerialPrintLn("MANUAL");
                InterpretRadioCommand();
            }
        }
    }

    void Controller::InterpretRadioCommand()
    {
        // analogWrite(this->MotorPin, payload[1]);        // this->payload[1]);
        Motor.write(map(payload[1], 0, 255, 0, 180));
        Elevator1.write(GetAngleFromMessage(payload[2], 90));
        Elevator2.write(GetAngleFromMessage(255 - payload[2], 90));
        Rudder.write(GetAngleFromMessage(255 - payload[3], 110));
        AileronLeft.write(GetAngleFromMessage(payload[4], 65));
        AileronRight.write(GetAngleFromMessage(payload[4], 90));
    }

    int GetAngleFromDu(float radians, int offset)
    {
        if (radians > pi / 2)
        {
            radians = pi / 2;
        }
        else if (radians < -pi / 2)
        {
            radians = -pi / 2;
        }
        int rads = map(radians, -pi / 2, pi / 2, -45, 45);
        return (offset + rads);
    }

    int GetAngleFromMessage(int radians, int offset)
    {
        int rads = map(radians, 0, 255, -45, 45);
        return (offset + rads);
    }

    void Controller::ReadDataFromSensors()
    {
        ReadRadio();
        ReadMPU();
        ReadGPS();
        Estimator::Matrix readings(9, 1);
        readings.CopyBloc(TranslationalVelocities.GetValue(), 0, 0);
        readings.CopyBloc(RotationalVelocities.GetValue(), 3, 0);
        readings.matrix[6][0] = newRead[0];
        readings.matrix[7][0] = newRead[0];
        readings.matrix[8][0] = newRead[0];

        Kalman.DoKalmanAlgorithm(readings);
    }

    Estimator::Matrix Controller::CalculateControl()
    {
        Estimator::Matrix control(5, 1);
        control = Kalman.DeltaInputs + Kalman.Inputs;
        /*
        Serial.print("(control) :");
        Serial.print( Kalman.DeltaInputs.matrix[0][0]);
        Serial.print(",");
        Serial.print( Kalman.DeltaInputs.matrix[1][0]);
        Serial.print(",");
         Serial.print( Kalman.DeltaInputs.matrix[2][0]);
        Serial.print(",");
         Serial.print( Kalman.DeltaInputs.matrix[3][0]);
        Serial.print(",");
         Serial.println( Kalman.DeltaInputs.matrix[4][0]);*/

        if (!SimpleMission)
        {
            float *error;
            error = Mission.GetCurrentError(latitude, lon);
            OyController.Read(error[0]);
            OzController.Read(error[1]);
            control.matrix[2][0] += OyController.GetCommand();
            control.matrix[3][0] += OzController.GetCommand();
            delete error;
        }

        return control;
    }

    uint8_t ThrustToPwm(float thrust)
    {
        return (uint8_t)((thrust - ThrustCoeff[1]) / ThrustCoeff[0]);
    }

    void Controller::ReadGPS()
    {
        if (Serial1.available() > 0)
        { // check for gps data

            if (gps.encode(Serial1.read())) // encode gps data
            {
                gps.f_get_position(&latitude, &lon);
            }
        }
    }
}
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
    const float ThrustCoeff[2] = {0.0328, -0.0813};
    // We expect to get the deflection angle of the surfaces in radians.
    // We convert them to pwm for the servomotor.
    int GetAngleFromDu(float radians, int offset = 0);
    // We expect to get the deflection angle of the surfaces in radians.
    // We convert them to pwm for the servomotor.
    int GetAngleFromMessage(int radians, int offset = 0);
    // uint8_t Rad2PWM(float radians, float offset = 0.4668);
    //  Converts thrust from newtons to PWM for motor.
    uint8_t ThrustToPwm(float thrust);
    /**
     * Instance of the complete onboard aircraft controller.
     * Capable of both manual and autonomous flight.
     */
    class Controller : OnBoardHelper::AircraftConfiguration
    {
    private:
        Servo Motor;
        Servo AileronRight;
        Servo AileronLeft;
        Servo Elevator1;
        Servo Elevator2;
        Servo Rudder;
        // this will gives us the position error to the current mission
        MissionControl::RectangleMission Mission;
        // this will flag if we reset mission
        bool newMission = true;
        // Base latitude in Cluj-Napoca BT arena.
        float latitude = 46.786;
        // Base longitude in CLuj-Napoca BT arena.
        float lon = 23.592;
        float lastTranslationalRead[3] = {10, 10, 10};
        float newTranslationalRead[3] = {0, 0, 0};
        float value = 0.f;
        // Error tolerance in meters for mission.
        const float tolerance = 10;
        // Current state of the aircraft.
        OnBoardHelper::States state;
        MPU9250 mpu; // create mpu object
        TinyGPS gps; // create gps object
        // Flags if serial communication should be enabled.
        bool PcSerial = false;
        // Array of latest Control commands.
        int ControlInputs[5] = {0, 0, 0, 0, 0};
        RF24 radio; // using pin 7 for the CE pin, and pin 8 for the CSN pin
        uint8_t address[1][6] = {"1Node"};
        bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
        bool role = false;    // true = TX role, false = RX role
        // Default payload value.
        uint16_t payload[5] = {1, 50, 100, 150, 150};
        bool autonomousMode = false;
        // Initializezs Radio commmunication on pins 30 and 31 as reciever.
        void InitializeRadioReciever();
        // Initializes GPS on Serial1.
        void InitializeGPS();
        // Initialzies MPU on using SPI interface.
        void InitializeMPU();
        // INitialzies serial communication with computer for debug.
        void InitializeSerial();
        // Writes on computer.
        void SerialPrintLn(String text);
        // Writes the current readings on computer using serial.
        void WriteCurrentReadings();
        // makes a read from radio channel.
        // If no message is recieved the last state will be held until new messages appear.
        bool ReadRadio();
        // Changes the state of the aircraft.
        void ChangeState(OnBoardHelper::States newState);
        // Interprets the message recieved from the radio.
        void InterpretRadioCommand();
        // Reads movement on the MPU and calculates differences to the last state.
        // MPU has a lot of noise on wake-up and we wil wate for it to calm down before using his readings.
        bool ReadDifference();
        // Calculates differences to the last state.
        void CalculateDiff();
        // Reads the MPU values.
        bool ReadMPU();
        // Reads GPS values.
        void ReadGPS();
        // Initializes the onboard controller.
        void InitializeControllers();
        // Instances of the PID controller on the horizontal field.
        Estimator::PIDz OyController;
        // Instances of the PID controller on the vertical field.
        Estimator::PIDz OzController;

        Estimator::Integrator TranslationalVelocities;
        Estimator::Integrator RotationalVelocities;

    public:
        //  kalman filter implementation on a fixed size system.
        Estimator::FixedSizeSystem Kalman;
        // Flag that sets up if we use square mission or not.
        // Simple mission means linear mission.
        bool SimpleMission = true;
        Controller(uint32_t motorPin, uint32_t elevatorPin1, uint32_t elevatorPin2, uint32_t rudderPin, uint32_t aileronLeftPin, uint32_t aileronRightPin);
        // Starts set-up sequance for the controller.
        void Setup(bool serial);
        // Interprets command recieved from the radio.
        void InterpretComand();
        // Makes a new reading from all sensor and applies kalman filter and position estimation.
        void ReadDataFromSensors();
        // Calculates control using both linear gaussian contrller and PID commands.
        Estimator::Matrix CalculateControl();
    };
}
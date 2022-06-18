#include "MPU9250.h"
#include "TinyGPS.h"
#include "Arduino.h"
#include "SPI.h"
#include "RF24.h"

// instantiate an object for the nRF24L01 transceiver
namespace OnBoard
{
    void DCM(float phi, float theta, float psi, float (&rslt)[3][3])
    {
        phi = phi * PI / 180;
        theta = theta * PI / 180;
        psi = psi * PI / 180;

        rslt[0][0] = (float)(cos(phi) * sin(theta));
        rslt[0][1] = (float)(cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(theta));
        rslt[0][2] = (float)(cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(theta));
        rslt[1][0] = (float)(sin(phi) * cos(theta));
        rslt[1][1] = (float)(sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi));
        rslt[1][2] = (float)(sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi));
        rslt[2][0] = (float)(-sin(theta));
        rslt[2][1] = (float)(cos(theta) * sin(psi));
        rslt[2][2] = (float)(cos(phi) * cos(psi));
    }

    void mulMat(float mat1[3][3], float mat2[3][1], float (&rslt)[3][1])
    {
        rslt[2][0] = (mat1[0][0] * mat2[0][0] + mat1[0][1] * mat2[1][0] + mat1[0][2] * mat2[2][0]);
        rslt[2][1] = (mat1[1][0] * mat2[0][0] + mat1[1][1] * mat2[1][0] + mat1[1][2] * mat2[2][0]);
        rslt[2][2] = (mat1[2][0] * mat2[0][0] + mat1[2][1] * mat2[1][0] + mat1[2][2] * mat2[2][0]);
    }
    enum States
    {
        SETUP = 44,
        NORMAL = 45,
        INDEPENDENT = 46,
        ERROR = 47
    };

    class Controller
    {
    private:
        States state;
        MPU9250 mpu; // create mpu object
        TinyGPS gps; // create gps object
        bool PcSerial = false;
        int ControlInputs[5] = {0, 0, 0, 0, 0};
        RF24 radio; // using pin 7 for the CE pin, and pin 8 for the CSN pin
        uint8_t address[6] = {"1Node"};
        bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
        bool role = false;    // true = TX role, false = RX role
        // default payload value
        uint8_t payload[6] = {1, 50, 100, 150, 150};
        bool SelfControl = false;
        void InitializeRadioReciever()
        {
            SerialPrintLn("Started Radio set up");
            ChangeState(ERROR);
            this->radio = RF24(30, 31);
            // initialize the transceiver on the SPI bus
            if (!this->radio.begin())
            {
                SerialPrintLn("radio hardware is not responding!!");
                while (1)
                {
                } // hold in infinite loop
            }
            SerialPrintLn("Done with Radio set up");
            ChangeState(SETUP);
        }
        void InitializeGPS()
        {
            SerialPrintLn("Started GPS set up");
            ChangeState(ERROR);
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
            ChangeState(SETUP);
        }
        void InitializeMPU()
        {
            SerialPrintLn("Started MPU set up");
            ChangeState(ERROR);
            Wire.begin();
            this->mpu.setup(0x68);
            delay(5000);
            SerialPrintLn("Done with MPU set up");
            ChangeState(SETUP);
        }
        void InitializeSerial()
        {
            if (this->PcSerial)
            {
                Serial.begin(9600);
            }
        }
        void SerialPrintLn(char text[])
        {
            if (this->PcSerial)
            {
                Serial.println(text);
            }
        }
        bool ReadRadio()
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
        void ChangeState(State newState)
        {
            digitalWrite(this->state, 0);
            this->state = newState;
            digitalWrite(this->state, 1);
        }
        void InterpretPayload(){
           if(payload[0]>0)
           {
            ChangeState(INDEPENDENT);
            ReadDataFromSensors();
            CalculateNewControl();
           }
           else{
            ChangeState(NORMAL);
            payload[1];//Thrust
            payload[2];//Elevator
            payload[3];//Rudder
            payload[4];//Aileron Left
            -payload[4];//AileronRight            
           }
        }

    public:
        void Setup(bool serial)
        {
            ChangeState(SETUP);
            this->PcSerial = serial;
            InitializeSerial();
            InitializeMPU();
            InitializeGPS();
            delay(5000);
            InitializeRadioReciever();
        }

        void CalculateNewControl()
        {

        }

        void ReadDataFromSensors()
        {

        }
    };
}
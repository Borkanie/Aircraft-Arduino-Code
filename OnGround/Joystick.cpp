#include "Joystick.h"

namespace Controller
{

    void ChangeMode()
    {
        controller.ChangeState(Normal);
        controller.ClearRadio();
        AutonomosMode = !AutonomosMode;
        controller.SerialPrintLn("Changed Mode");
    }
    void Controller::ClearRadio()
    {
        this->radio.flush_tx();
    }
    void Controller::Setup(bool serial)
    {
        this->PcSerial = serial;
        ChangeState(Error);
        pinMode(Normal, OUTPUT);
        pinMode(Error, OUTPUT);
        pinMode(Motor, INPUT);
        pinMode(Elevator, INPUT);
        pinMode(Rudder, INPUT);
        pinMode(Aileron, INPUT);
        SetupSerial();
        SetupRadio();
        ChangeState(Normal);
    }
    void Controller::SetupRadio()
    {
        SerialPrintLn("Started Radio set up");
        ChangeState(Error);
        this->radio = RF24(8, 9);
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

        // set the TX address of the RX node into the TX pipe
        radio.openWritingPipe(address[0]); // always uses pipe 0

        radio.setRetries(0, 15);
        radio.stopListening();  // put radio in TX mode
        SerialPrintLn("Done with Radio set up");
        ChangeState(Normal);
    }
    void Controller::ChangeState(FlagPins pin)
    {
        switch (pin)
        {
        case Error:
            digitalWrite(Normal, 0);
            digitalWrite(Error, 1);
            break;
        case Normal:
            digitalWrite(Normal, 1);
            digitalWrite(Error, 0);
            break;
        default:
            break;
        }
    }
    void Controller::Read()
    {
        this->payload[0] = AutonomosMode;
        if (!AutonomosMode)
        {
            this->payload[1] = 0;
            this->payload[2] = 0;
            this->payload[3] = 0;
            this->payload[4] = 0;
        }
        else
        {
            this->payload[1] = analogRead(Motor)/4;
            this->SerialPrintLn(payload[1]);
            this->payload[2] = analogRead(Elevator)/4;
            this->SerialPrintLn(payload[2]);
            this->payload[3] = analogRead(Rudder)/4;
            this->SerialPrintLn(payload[3]);
            this->payload[4] = analogRead(Aileron)/4;
            this->SerialPrintLn(payload[4]);
        }
    }

    void Controller::Transmit()
    {
        bool report = radio.write(&payload, sizeof(int)); // transmit & save the report
        if (!report)
        {
            ChangeState(Error);
            SerialPrintLn("Message wasn't transmitted");
        }
        else
        {
            ChangeState(Normal);
            SerialPrintLn("Was transmitted");
        }
    }
    void Controller::SetupSerial()
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
    void Controller::SerialPrintLn(float text)
    {
        if (this->PcSerial)
        {
            Serial.println(text);
        }
    }

}

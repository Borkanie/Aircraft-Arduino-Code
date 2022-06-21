#include "Arduino.h"
#include "OnBoardController.h"
#include <stdint.h>

using namespace OnBoard;

uint32_t motorPin = 144;
uint32_t elevatorPin = 139;
uint32_t rudderPin = 137;
uint32_t aileronLeftPin = 136;
uint32_t aileronRightPin = 135;
Controller controller = Controller(motorPin, elevatorPin, rudderPin, aileronLeftPin, aileronRightPin);
void setup()
{ 
    controller.Setup(false);
}
void loop()
{
    controller.ReadDataFromSensors();
    controller.InterpretComand();
}
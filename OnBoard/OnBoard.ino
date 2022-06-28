#include "Arduino.h"
#include "OnBoardController.h"
#include <stdint.h>
#include "Estimator.h"

using namespace OnBoard;

uint32_t motorPin = 3;
uint32_t elevatorPin = 4;
uint32_t rudderPin = 5;
uint32_t aileronLeftPin = 6;
uint32_t aileronRightPin = 7;
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
#include "Arduino.h"
#include "OnBoardController.h"
#include <stdint.h>
#include "Estimator.h"

using namespace OnBoard;

uint32_t motorPin = 2;
uint32_t elevatorPin1 = 3;
uint32_t elevatorPin2 = 4;
uint32_t rudderPin = 5;
uint32_t aileronLeftPin = 6;
uint32_t aileronRightPin = 7;
Controller controller = Controller(motorPin, elevatorPin1,elevatorPin2, rudderPin, aileronLeftPin, aileronRightPin);
void setup()
{
  controller.Setup(true);
}
void loop()
{
  controller.ReadDataFromSensors();
  controller.InterpretComand(); 
 
}
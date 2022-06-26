#include "Arduino.h"
#include "OnBoardController.h"
#include <stdint.h>
#include "Estimator.h"

using namespace OnBoard;
unsigned long mytime;
uint32_t motorPin = 3;
uint32_t elevatorPin = 4;
uint32_t rudderPin = 5;
uint32_t aileronLeftPin = 6;
uint32_t aileronRightPin = 7;
Controller controller = Controller(motorPin, elevatorPin, rudderPin, aileronLeftPin, aileronRightPin);
void setup()
{
  controller.Setup(true);
  //  pinMode(motorPin, OUTPUT);
  //    pinMode(elevatorPin, OUTPUT);
  //    pinMode(rudderPin, OUTPUT);
  //    pinMode(aileronLeftPin, OUTPUT);
  //    pinMode(aileronRightPin, OUTPUT);
}
void loop()
{
  mytime = millis();
  controller.ReadDataFromSensors();
  controller.InterpretComand();
  if(millis() - mytime<90)
  {
    delay(100-(millis() - mytime));
  }
  //            analogWrite(motorPin, 50);        // Thrust set pwm
  //         analogWrite(elevatorPin, 100);     // Elevator set pwm
  //         analogWrite(rudderPin, 150);       // Rudder set pwm
  //         analogWrite(aileronLeftPin, 200);  // Aileron Left set pwm
  //        analogWrite(aileronRightPin,250); // Aileron Right set pwm
}
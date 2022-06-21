#include "Arduino.h"
#include "Joystick.h"
#include <stdint.h>

using namespace Controller;


void setup()
{
    controller.Setup(false);
    attachInterrupt(digitalPinToInterrupt(Mode), ChangeMode, RISING);
}
void loop()
{
    controller.Read();
    controller.Transmit();
    delay(100);
}

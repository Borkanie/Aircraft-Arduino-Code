#include "TinyGPS.h"
#include "Arduino.h"

TinyGPS gps; // create gps object
TinyGPS gps2; // create gps object
float latitude = 28.5458;
float lon = 77.1703; // create variable for latitude and longitude object
void setup()
{
  Serial.begin(9600); // connect serial
  // Serial.println("The GPS Received Signal:");
  Serial1.begin(9600); // connect gps sensor
  while(!Serial1)
  {
    delay(100);
    Serial.println("Waiting for gps");
  }

}
void loop()
{
  while (Serial1.available()>0)
  {                                 // check for gps data

    if (gps.encode(Serial1.read())) // encode gps data
    {
      gps.f_get_position(&latitude, &lon); // get latitude and longitude
      // display position
      Serial.print("Position1: ");
      Serial.print("Latitude:");
      Serial.print(latitude, 6);
      Serial.print(";");
      Serial.print("Longitude:");
      Serial.println(lon, 6);
      Serial.print(" ");
    }
  }
 
  Serial.print("Serial1:");
  Serial.print(latitude);
  Serial.print(";");
  Serial.println(lon);
  delay(1000);

}
/*
 * Rui Santos 
 * Complete Project Details https://randomnerdtutorials.com
 */


void setup(){
  Serial.begin(9600);
  Serial1.begin(9600);
}

void loop(){
  while (Serial1.available() > 0)
  Serial.write(Serial1.read());
}

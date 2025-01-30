#include <ESP32Servo.h>

#define servoPin 4

Servo myservo;  //creamos un objeto servo 
int vel = 1500;

 
void setup() { 
  myservo.attach(servoPin);  // asignamos el pin 3 al servo.
   Serial.begin(115200);

} 
 
void loop() { 
  
  if(Serial.available() >= 1)
  {
    vel = Serial.parseInt(); //Leer un entero por serial
    if(vel != 0)
    {
      myservo.writeMicroseconds(vel);


}
}

}
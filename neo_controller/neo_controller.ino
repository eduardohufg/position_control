#include <ESP32Servo.h>
#include <HardwareSerial.h>

// Pines
#define chPinA 2
#define chPinB 4
#define servoPin 5
#define PWMInput 7

// Variables globales
Servo myservo;
HardwareSerial MySerial(1);
bool init_state = false;
long motorPosition = 0;

// PID
volatile float targetPosition = 0;
float integral = 0.0005;
float proportional = 2.5;
float derivative = 0.0005;
double controlSignal = 0;
double previousTime = 0;
double previousError = 0;
double errorIntegral = 0;
double currentTime = 0;
double deltaTime = 0;
double errorValue = 0;
double edot = 0;

// PWM
int PWMValue = 0;
int motorDirection = 0;

void setup() {
  Serial.begin(115200);
  myservo.attach(servoPin);
  MySerial.begin(1000000, SERIAL_8N1, 16, 17);

  // Configurar pines
  pinMode(chPinA, INPUT_PULLUP);
  pinMode(chPinB, INPUT_PULLUP);
  pinMode(PWMInput, INPUT);

}

void loop() {
  targetpos();
  checkencoder();
  calculatePID();
  driveMotor();
}

void driveMotor() {
  // Determinar dirección
  motorDirection = (controlSignal < 0) ? -1 : (controlSignal > 0) ? 1 : 0;

  // Calcular PWM
  PWMValue = (int)fabs(controlSignal);
  PWMValue = constrain(PWMValue, 0, 255); //checar, tengo duda
  PWMValue = map(PWMValue, 0, 255, 0, 90);

  if (PWMValue <= 5 && errorValue != 0) {
    PWMValue = 5;
  }

  if (motorDirection == -1) {
    myservo.writeMicroseconds(1490 - PWMValue);
  } else if (motorDirection == 1) {
    myservo.writeMicroseconds(1501 + PWMValue);
  } else {
    myservo.writeMicroseconds(1500);
  }
}

void calculatePID() {
  currentTime = micros();
  deltaTime = (currentTime - previousTime) / 1000000.0;
  previousTime = currentTime;

  errorValue = motorPosition - targetPosition;
  edot = (errorValue - previousError) / deltaTime;
  errorIntegral += errorValue * deltaTime;
  controlSignal = (proportional * errorValue) + (derivative * edot) + (integral * errorIntegral);

  previousError = errorValue;
}

void printValues() {
  Serial.print("Position: ");
  Serial.println(motorPosition);
  Serial.print("Error: ");
  Serial.println(errorValue);
  Serial.print("PWM: ");
  Serial.println(PWMValue);
}

void targetpos() {
  if (MySerial.available() > 0) {

    String velString = MySerial.readStringUntil('\n');

    if(velString == "init_uart"){
      init_state = true;
    }
    else if(velString == "close_uart"){
      init_state = false;
    }
    else if (init_state){
    targetPosition = velString.toInt();
    }   
  }
}

void checkencoder() {
  int PWM = GetPWM(PWMInput);
  motorPosition = PWM;
}


int GetPWM(int pin)
{
  unsigned long highTime = pulseIn(pin, HIGH);

  if (highTime == 0)
    return digitalRead(pin); 

  return highTime;
}

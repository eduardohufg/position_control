#include <ESP32Servo.h>
#include <HardwareSerial.h>

// Pines
#define chPinA 2
#define chPinB 4
#define servoPin 8
#define PWMInput 7

// Variables globales
Servo myservo;
HardwareSerial MySerial(1);
bool init_state = false;
long motorPosition = 0;

// PID
volatile float targetPosition = 0;
float integral = 0.00025;
float proportional = 1.2;
float derivative = 0.00025;
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

//filter

const int win_size = 10; 
volatile float readings[win_size] = {0}; 
volatile float sum = 0; 
volatile int indexx = 0;

void setup() {
  Serial.begin(115200);
  myservo.attach(servoPin);
  MySerial.begin(1000000, SERIAL_8N1, 16, 17);

  // Configurar pines
  pinMode(chPinA, INPUT_PULLUP);
  pinMode(chPinB, INPUT_PULLUP);
  pinMode(PWMInput, INPUT);

  targetPosition = GetPWM(PWMInput);

}

void loop() {
  targetpos();
  checkencoder();
  calculatePID();
  driveMotor();
  printValues();
}

void driveMotor() {
  // Determinar dirección
  motorDirection = (controlSignal < 0) ? 1 : (controlSignal > 0) ? -1 : 0;

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
  Serial.println(deltaTime);
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
  if (Serial.available() > 0) {

    String velString = Serial.readStringUntil('\n');

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
  int rawReading = GetPWM(PWMInput);

  sum = sum - readings[indexx];
  readings[indexx] = rawReading;
  sum = sum + rawReading;
  indexx = (indexx + 1) % win_size;

  motorPosition = sum / win_size;
}


int GetPWM(int pin)
{
  unsigned long highTime = pulseIn(pin, HIGH);

  if (highTime == 0)
    return digitalRead(pin); 

  return highTime;
}

#include <ESP32Servo.h>
#include <HardwareSerial.h>

// Pines
#define chPinA 2
#define chPinB 4
#define servoPin 5
#define PWMInput 7

//timer

hw_timer_t *timer = NULL;
bool timer_flag = false;

// Variables globales
Servo myservo;
HardwareSerial MySerial(1);
bool init_state = false;
volatile long motorPosition = 0;

// PID
volatile long targetPosition = 0;
float ki = 0.00005;
float kp = 0.00001;
float kd = 0.00005;
volatile double controlSignal = 0;
double previousTime = 0;
double error_k_1 = 0;
double error_k_2 = 0;
double errorIntegral = 0;
double currentTime = 0;
const double deltaTime = 0.01;
double error_k = 0;
double edot = 0;
volatile double prev_controlSignal = 0.0;
double u_s = 0.0;
double u_max = 1;  
double u_min = -1; 

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
  MySerial.begin(1000000, SERIAL_8N1, 44, 43);

  // Configurar pines
  pinMode(chPinA, INPUT_PULLUP);
  pinMode(chPinB, INPUT_PULLUP);
  pinMode(PWMInput, INPUT);

  targetPosition = GetPWM(PWMInput);

  Timer_Init();

}

void loop() {
  if (timer_flag){
    checkencoder();
    calculatePID();
    timer_flag = false;
    
  }
  targetpos();
  driveMotor();
  printValues();
}

void driveMotor() {
  // Determinar dirección
  motorDirection = (controlSignal < 0) ? -1 : (controlSignal > 0) ? 1 : 0;

  // Calcular PWM
  PWMValue = (int)fabs(controlSignal);
  PWMValue = map(PWMValue, 0, 1, 0, 90);

  if (motorDirection == -1) {
    myservo.writeMicroseconds(1490 - PWMValue);
  } else if (motorDirection == 1) {
    myservo.writeMicroseconds(1501 + PWMValue);
  } else {
    myservo.writeMicroseconds(1500);
  }
}


void printValues() {
  Serial.print("Position: ");
  Serial.println(motorPosition);
  Serial.print("Error: ");
  Serial.println(error_k);
  Serial.print("PWMm: ");
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

void calculatePID() {

  error_k = motorPosition - targetPosition;

  controlSignal = prev_controlSignal + error_k * (kd/deltaTime + kp + ki*deltaTime) - error_k_1 * (2*kd/deltaTime + kp) + error_k_2*ki*deltaTime;

  error_k_2 = error_k_1;
  error_k_1 = error_k;

  controlSignal = saturation(controlSignal);

  prev_controlSignal = controlSignal;
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
  unsigned long highTime = pulseIn(pin, HIGH, 10);

  if (highTime == 0)
    return digitalRead(pin); 

  return highTime;
}

void ARDUINO_ISR_ATTR onTimer() {
  if(timer_flag == false)
  {
    timer_flag = true;
  }
  
}

void Timer_Init(void)
{
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 10000, true, 0);
 
}

double saturation(double u){
    return min(max(u, u_min), u_max);
}

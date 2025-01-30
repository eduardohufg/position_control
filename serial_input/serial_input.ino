#include <HardwareSerial.h>

// Pines
#define chPinA 2
#define chPinB 3
#define servoPin 8
#define PWMInput 7

//timer

hw_timer_t *timer = NULL;
volatile bool timer_flag = false;

void IRAM_ATTR onTimer() {
  if(timer_flag == false)
  {
    timer_flag = true;
  }
}

//ledc
const int PWMFreq = 50; 
const int PWMChannel = 0;
const int PWMResolution = 14;
const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) - 1);

// Variables globales
HardwareSerial MySerial(1);
bool init_state = false;
long motorPosition = 0;

// PID
volatile long targetPosition = 0;
float integral = 0.2;
float proportional = 0.2;
float derivative = 0.075;
double controlSignal = 0;
double previousTime = 0;
double previousError = 0;
double errorIntegral = 0;
double currentTime = 0;
const double deltaTime = 0.01;
long errorValue = 0;
double edot = 0;

// PWM
double PWMValue = 0;
int motorDirection = 0;

//filter

const int win_size = 4; 
volatile float readings[win_size] = {0}; 
volatile float sum = 0; 
volatile int indexx = 0;

//saturation

const int sat_control_signal = 100;
int sat_windup = 20;

void setup() {
  Serial.begin(115200);
  MySerial.begin(1000000, SERIAL_8N1, 44, 43);
  ledcSetup(PWMChannel, PWMFreq, PWMResolution);
  ledcAttachPin(servoPin, PWMChannel);

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
  //printValues();
}

void driveMotor() {
  // Determinar dirección
  motorDirection = (controlSignal < 0) ? 1 : (controlSignal > 0) ? -1 : 0;

  PWMValue = (int)fabs(controlSignal);
  PWMValue = map(PWMValue, 0, 100, 0, 100);

  if (motorDirection == -1) {
    ledcWrite(PWMChannel, (int)calculateWidh(1490 - PWMValue));
  } else if (motorDirection == 1) {
    ledcWrite(PWMChannel, (int)calculateWidh(1511 + PWMValue));
  } else {
    ledcWrite(PWMChannel, (int)calculateWidh(1500));
  }
}

void calculatePID() {
  errorValue = motorPosition - targetPosition;
  edot = (errorValue - previousError) / deltaTime;

  errorIntegral += (errorValue + previousError) * deltaTime / 2;

  errorIntegral = saturation(errorIntegral, sat_windup * -1, sat_windup);
  controlSignal = (proportional * errorValue) + (derivative * edot) + (integral * errorIntegral);

  controlSignal = saturation(controlSignal, sat_control_signal * -1, sat_control_signal);

  previousError = errorValue;
}

void printValues() {
  Serial.print("Position: ");
  Serial.println(motorPosition);
  Serial.print("Error: ");
  Serial.println(errorValue);
  Serial.print("controlSignal: ");
  Serial.println(controlSignal);
  Serial.print("PWM: ");
  Serial.println(PWMValue);
}

void targetpos() {
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');

    if (inputString == "init_uart") {
      init_state = true;
    } else if (inputString == "close_uart") {
      init_state = false;
    } else if (init_state) {
      if (inputString.indexOf(' ') == -1) {
        targetPosition = inputString.toInt();
      } else {
        // Procesar los parámetros del PID
        float newProportional, newIntegral, newDerivative;
        int newSatWindup;
        int numParsed = sscanf(inputString.c_str(), "%f %f %f %d", &newProportional, &newIntegral, &newDerivative, &newSatWindup);

        if (numParsed == 4) {
          proportional = newProportional;
          integral = newIntegral;
          derivative = newDerivative;
          sat_windup = newSatWindup;

          Serial.println("PID parameters updated:");
          Serial.print("Proportional: ");
          Serial.println(proportional);
          Serial.print("Integral: ");
          Serial.println(integral);
          Serial.print("Derivative: ");
          Serial.println(derivative);
          Serial.print("Saturation Windup: ");
          Serial.println(sat_windup);
        } else {
          Serial.println("Error: Invalid PID parameters format.");
        }
      }
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

int GetPWM(int pin) {
  unsigned long highTime = pulseIn(pin, HIGH);

  if (highTime == 0)
    return digitalRead(pin);

  return highTime;
}


void Timer_Init(void)
{
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 10000, true);
  timerAlarmEnable(timer);
}
 

double saturation(double u, double u_min, double u_max) {
  return min(max(u, u_min), u_max);
}

long calculateWidh(long time){
  return time * MAX_DUTY_CYCLE / 20000;
}

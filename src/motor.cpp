#include <Arduino.h>
#include <motor.h>
#include <PID_v1.h>

const int potPin = A0;
const int refPin = A1;
const int IN1 = PD2;
const int IN2 = PD6;
const int ENABLE = PD5;

double Kp = 0, Ki = 0, Kd = 0;
double Input, Output, Setpoint;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

const int minPWM = 50;
const int maxPWM = 255;
const float deadband = 1.0;
unsigned long scanTime = 50;
unsigned long lastTime = 0;

static bool motorIsRunning = false;
bool isMotorInitialized = false;
int readings[SMOOTHING_WINDOW] = {0};
int readIndex = 0;
long total = 0;

void initSmoothing() {
  for (int i = 0; i < SMOOTHING_WINDOW; i++) readings[i] = 0;
}

float getSmoothedInput() {
  total -= readings[readIndex];
  readings[readIndex] = analogRead(potPin);
  total += readings[readIndex];
  readIndex = (readIndex + 1) % SMOOTHING_WINDOW;
  return (total / (float)SMOOTHING_WINDOW) / 1023.0 * 180.0;
}

void MotorInit(){
    pinMode(ENABLE, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    Setpoint = 0;
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-255, 255);
    isMotorInitialized = true;
};

void MainMotorRuntime(double* Pv, double* Sp, bool* Mode){
  if(!isMotorInitialized){
    MotorInit();
  }

  if(*Mode){
    Setpoint = *Sp;
  }else{
    Setpoint = (analogRead(refPin) / 1023.0) * 180.0;
  }

  Input = getSmoothedInput();
  *Pv = Input;
  unsigned long now = millis();

  if (now - lastTime >= scanTime) {
    motorIsRunning = true;
    lastTime = now;

    if (abs(Setpoint - Input) < deadband) {
      analogWrite(ENABLE, 0);
      return;
    }

    myPID.Compute();

    int pwmValue = constrain(abs(Output), 0, 255);
    if (pwmValue < minPWM) pwmValue = minPWM;

    if (Output > 0) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    }

    analogWrite(ENABLE, pwmValue);
  }
  motorIsRunning = false;
};

void ReTuneKpid(double Kp, double Ki, double Kd){
  if(!motorIsRunning){
    myPID.SetTunings(Kp, Ki, Kd);
  }
}

void stopMotorRuntime(){
  myPID.SetMode(MANUAL);
  analogWrite(ENABLE, 0); 
  digitalWrite(IN1, LOW);  
  digitalWrite(IN2, LOW);
  Output = 0;
  initSmoothing();
  isMotorInitialized = false;
}
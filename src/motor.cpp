#include <Arduino.h>
#include <motor.h>
#include <PID_v1.h>

const int potPin = A0;
const int refPin = A1;
const int IN1 = PD2;
const int IN2 = PD6;
const int ENABLE = PD5;

double Kp = 0, Ki = 0, Kd = 0;
double Input = 0;
double Output = 0;
double Setpoint = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

const int minPWM = 0;
const int maxPWM = 255;
const float deadband = 1.0;
unsigned long scanTime = 20;
unsigned long lastTime = 0;

static bool motorIsRunning = false;
bool isMotorInitialized = false;
int readings[SMOOTHING_WINDOW] = {0};
int lastSensRead = 0;
bool firstCycle = false;
int readIndex = 0;
long total = 0;
int lastRefRead = 0;
bool kickStart = false;
static int currentPWM = 0;

void initSmoothing() {
  for (int i = 0; i < SMOOTHING_WINDOW; i++) readings[i] = 0;
}

float getSmoothedInput(int* r_limit, int* l_limit){
  total -= readings[readIndex];
  int read = analogRead(potPin);
  if(!firstCycle){
    lastSensRead = read;
    firstCycle  = true;
  }

  if(abs(lastSensRead - read) < JITTER_COMPEN){
    read = lastSensRead;
  }
  readings[readIndex] = read;
  total += readings[readIndex];
  readIndex = (readIndex + 1) % SMOOTHING_WINDOW;
  int raw = (total / (float)SMOOTHING_WINDOW);
  raw = constrain(raw, min(*r_limit, *l_limit), max(*r_limit, *l_limit));
  float mapped = (float)(raw - *l_limit) * 180.0 / (*r_limit - *l_limit);
  lastSensRead = read;
  
  // Only for Debugging
  // Serial.print(", Rlimit = ");
  // Serial.print(*r_limit);
  // Serial.print(", Llimit = ");
  // Serial.print(*l_limit);
  // Serial.print(", raw = ");
  // Serial.println(raw);

  return mapped;
}

void MotorInit(){
    pinMode(ENABLE, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-255, 255);
    isMotorInitialized = true;
};

void MainMotorRuntime(double* Pv, double* Sp, bool* Mode, int* r_limit, int* l_limit){
  if(!isMotorInitialized){
    MotorInit();
  }

  if(*Mode){
    Setpoint = *Sp;
  }else{
    int refRead = map(analogRead(refPin), 0, 1023, 0, 180); 
    if (abs(lastRefRead - refRead) > 2) {
      lastRefRead = refRead;
    }
    Setpoint = lastRefRead;
  }

  Input = getSmoothedInput(r_limit, l_limit);
  *Pv = Input;
  unsigned long now = millis();

  // Only for debugging

  // Serial.print(Setpoint);
  // Serial.print(",");
  // Serial.print(Input);
  // Serial.print(",");
  // Serial.println(Output);

  if (now - lastTime >= scanTime) {
    motorIsRunning = true;
    lastTime = now;

    myPID.Compute();

    if (abs(Setpoint - Input) < deadband) {
      analogWrite(ENABLE, 255);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      return;
    }

    int pwmValue = constrain(abs(Output), 0, 255);
    // if (pwmValue < minPWM) pwmValue = minPWM;
    if(pwmValue > currentPWM){
      currentPWM++;
    }else{
      currentPWM--;
    }

    if (Output > 0) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    }

    analogWrite(ENABLE, currentPWM);
    Serial.println(currentPWM);
  }
  motorIsRunning = false;
};

void ReTuneKpid(double Kp, double Ki, double Kd){
  if(!motorIsRunning){
    myPID.SetMode(MANUAL);
    myPID.SetTunings(Kp, Ki, Kd);
    myPID.SetMode(AUTOMATIC);
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
  motorIsRunning = false;
  total = 0;  // Sangat penting
}

void SlowMoveMotor(uint8_t direction){
  if(direction){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }else{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  if(!kickStart){
    analogWrite(ENABLE, 255);
    delay(25);
    kickStart = true;
  }
  analogWrite(ENABLE, 140);
}

int getRawSensorValue(){
  return analogRead(potPin);
}
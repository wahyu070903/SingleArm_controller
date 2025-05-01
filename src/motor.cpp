#include <Arduino.h>
#include <motor.h>

const int potPin = A0;
const int refPin = A1;
const int IN1 = PD2;
const int IN2 = PD6;
const int ENABLE = PD5;

// PID parameters
double Kp = 0, Ki = 0, Kd = 0;
double Input = 0, Output = 0, Setpoint = 0;
double integral = 0, lastError = 0;
unsigned long lastTime = 0;

const int minPWM = 0;
const int maxPWM = 255;
const float deadband = 0.5;
const unsigned long scanTime = 20;

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

float getSmoothedInput(int* r_limit, int* l_limit) {
  total -= readings[readIndex];
  int read = analogRead(potPin);
  if (!firstCycle) {
    lastSensRead = read;
    firstCycle = true;
  }

  if (abs(lastSensRead - read) < JITTER_COMPEN) {
    read = lastSensRead;
  }

  readings[readIndex] = read;
  total += readings[readIndex];
  readIndex = (readIndex + 1) % SMOOTHING_WINDOW;
  int raw = total / (float)SMOOTHING_WINDOW;
  raw = constrain(raw, min(*r_limit, *l_limit), max(*r_limit, *l_limit));
  float mapped = (float)(raw - *l_limit) * 180.0 / (*r_limit - *l_limit);
  lastSensRead = read;

  return mapped;
}

void MotorInit() {
  pinMode(ENABLE, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  isMotorInitialized = true;
  lastTime = millis();
}

double computePID(double input, double setpoint) {
  double error = setpoint - input;

  if (abs(error) < deadband) {
    integral = 0;
    return 0;
  }

  double tentativeIntegral = integral + error * (scanTime / 1000.0);
  double tentativeOutput = Kp * error + Ki * tentativeIntegral + Kd * ((error - lastError) / (scanTime / 1000.0));

  if (abs(tentativeOutput) < maxPWM) {
    integral = tentativeIntegral;
  }

  double derivative = (error - lastError) / (scanTime / 1000.0);
  Output = Kp * error + Ki * integral + Kd * derivative;

  lastError = error;
  Serial.print(integral);
  Serial.print(", ");
  Serial.println(derivative);
  return Output;
}

void MainMotorRuntime(double* Pv, double* Sp, bool* Mode, int* r_limit, int* l_limit) {
  if (!isMotorInitialized) MotorInit();

  if (*Mode) {
    Setpoint = *Sp;
  } else {
    int refRead = map(analogRead(refPin), 0, 1023, 0, 180);
    if (abs(lastRefRead - refRead) > 2) lastRefRead = refRead;
    Setpoint = lastRefRead;
  }

  Input = getSmoothedInput(r_limit, l_limit);
  *Pv = Input;

  unsigned long now = millis();
  if (now - lastTime >= scanTime) {
    double pidOutput = computePID(Input, Setpoint);

    if (abs(Setpoint - Input) < deadband) {
      analogWrite(ENABLE, 255);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      return;
    }

    int pwmValue = constrain(abs(pidOutput), 0, 255);
    if (pwmValue > currentPWM) currentPWM++;
    else currentPWM--;

    if (pidOutput > 0) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    }

    analogWrite(ENABLE, currentPWM);
    lastTime = now;
  }
}

void ReTuneKpid(double p, double i, double d) {
  Kp = p;
  Ki = i;
  Kd = d;
  integral = 0;
  lastError = 0;
}

void stopMotorRuntime() {
  analogWrite(ENABLE, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  Output = 0;
  initSmoothing();
  isMotorInitialized = false;
  motorIsRunning = false;
  total = 0;
}

void SlowMoveMotor(uint8_t direction) {
  if (direction) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  if (!kickStart) {
    analogWrite(ENABLE, 255);
    delay(25);
    kickStart = true;
  }
  analogWrite(ENABLE, 140);
}

int getRawSensorValue() {
  return analogRead(potPin);
}

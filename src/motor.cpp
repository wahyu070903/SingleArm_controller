#include <Arduino.h>
#include <motor.h>
#include <PID_v1.h>

const int potPin = A0;
const int IN1 = PD2;
const int IN2 = PD6;
const int ENABLE = PD5;

double Kp = 1.2, Ki = 0.3, Kd = 0.6;
double Input, Output, Setpoint;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

const int minPWM = 50;
const int maxPWM = 255;
const float deadband = 1.0;
unsigned long scanTime = 50;
unsigned long lastTime = 0;

void MotorInit(){
    pinMode(ENABLE, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    Setpoint = 90;
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-255, 255);
};

void MainMotorRuntime(){
    unsigned long now = millis();
  if (now - lastTime >= scanTime) {
    lastTime = now;

    int analogValue = analogRead(potPin);
    Input = map(analogValue, 0, 1023, 0, 180);

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

    Serial.print("Angle: "); Serial.print(Input);
    Serial.print(" | Setpoint: "); Serial.print(Setpoint);
    Serial.print(" | Speed: "); Serial.println(pwmValue);
  }
};
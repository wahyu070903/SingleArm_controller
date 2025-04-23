#include <Arduino.h>
#include <menus.h>
#include <motor.h>

typedef struct {
  double Kp;
  double Ki;
  double Kd;
} PIDconstant;

PIDconstant PID_constant;
double SensValue, SetPoint;
bool referenceMode = false;

void setup() {
  Serial.begin(9600);
  MenusInit();
  PID_constant.Kp = 2.4;
  PID_constant.Ki = 0.2;
  PID_constant.Kd = 0.46;
  ReTuneKpid(PID_constant.Kp, PID_constant.Ki, PID_constant.Kd);
  MotorInit();
}

void loop() {
  referenceMode = getReferenceSource();
  MainMenusRuntime(&PID_constant.Kp, &PID_constant.Ki, &PID_constant.Kd, &SensValue, &SetPoint);
  if(isOnSettingMode()){
    stopMotorRuntime();
  }
  if(isKpidUpdateComplete()){
    ReTuneKpid(PID_constant.Kp, PID_constant.Ki, PID_constant.Kd);
    KpidUpdated();
  }
  if(!isOnSettingMode()){
    MainMotorRuntime(&SensValue, &SetPoint, &referenceMode);
  }

  // Serial.print("Kp = ");
  // Serial.println(PID_constant.Kp);

  // Serial.print("Ki = ");
  // Serial.println(PID_constant.Ki);

  // Serial.print("Kd = ");
  // Serial.println(PID_constant.Kd);
}

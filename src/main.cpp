#include <Arduino.h>
#include <menus.h>
#include <motor.h>
#include <EEPROM.h>
#include <math.h>

#define LIMIT_1 A2
#define LIMIT_2 A4 

typedef struct {
  double Kp;
  double Ki;
  double Kd;
} PIDconstant;

PIDconstant PID_constant;
double SensValue = 0;
double SetPoint = 0;
bool referenceMode = false;
int left_limit = 0;
int right_limit = 0;
uint8_t firstInit_flag = 0;
uint8_t FirstInit_counter = 0;
bool system_disabled = false;

void firstInit(){
  pinMode(LIMIT_1, INPUT_PULLUP);
  pinMode(LIMIT_2, INPUT_PULLUP);

  while (digitalRead(LIMIT_1) != 0) {
    SlowMoveMotor(0);
    Serial.print("Left = ");
    Serial.println(getRawSensorValue());
  }
  stopMotorRuntime();
  delay(200);
  left_limit = getRawSensorValue();
  Serial.println(left_limit);

  delay(1500);

  while (digitalRead(LIMIT_2) != 0) {
    SlowMoveMotor(1);
    Serial.print("Right = ");
    Serial.println(getRawSensorValue());
  }
  stopMotorRuntime();
  delay(200);
  right_limit = getRawSensorValue();
  Serial.println(right_limit);
  SlowMoveMotor(0);
  delay(500);
  stopMotorRuntime();
}

void setup() {
  Serial.begin(9600);
  MenusInit();
  // EEPROM.get(0, PID_constant);
  // if(isnan(PID_constant.Kp) || isnan(PID_constant.Ki) || isnan(PID_constant.Kd)){
  //   PID_constant.Kp = 0;
  //   PID_constant.Ki = 0;
  //   PID_constant.Kd = 0;
  // }
  PID_constant.Kp = 10;
  PID_constant.Ki = 2;
  PID_constant.Kd = 0;

  SetPoint = 90.0;
  ReTuneKpid(PID_constant.Kp, PID_constant.Ki, PID_constant.Kd);
  MotorInit();
  firstInit();
}

void loop() {
  if(digitalRead(LIMIT_1) == 0 || digitalRead(LIMIT_2) == 0){
    stopMotorRuntime();
    system_disabled = true;
  }else{
    referenceMode = getReferenceSource();
    MainMenusRuntime(&PID_constant.Kp, &PID_constant.Ki, &PID_constant.Kd, &SensValue, &SetPoint);
    if(isOnSettingMode()){
      stopMotorRuntime();
    }
    if(isKpidUpdateComplete()){
      ReTuneKpid(PID_constant.Kp, PID_constant.Ki, PID_constant.Kd);
      EEPROM.put(0, PID_constant);
      KpidUpdated();
    }
    if(!isOnSettingMode()){
      MainMotorRuntime(&SensValue, &SetPoint, &referenceMode, &right_limit, &left_limit);
    }
  }

  // Serial.println(SensValue);
}

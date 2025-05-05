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
uint8_t initSteps = 3;

void firstInit(){
  pinMode(LIMIT_1, INPUT_PULLUP);
  pinMode(LIMIT_2, INPUT_PULLUP);

  for(uint8_t i = 0 ; i < initSteps; i++){
    while (digitalRead(LIMIT_1) != 0) {
      SlowMoveMotor(0);
    }
    stopMotorRuntime();
    ResetSlowAddition();
    delay(250);
    if(i >= initSteps - 1){
      right_limit = getRawSensorValue();
      delay(200);
    }else{
      SlowMoveMotor(1);
      delay(500);
    }
  }

  delay(500);

  for(uint8_t i = 0 ; i < initSteps; i++){
    while (digitalRead(LIMIT_2) != 0) {
      SlowMoveMotor(1);
    }
    stopMotorRuntime();
    ResetSlowAddition();
    delay(250);
    if(i >= initSteps - 1){
      left_limit = getRawSensorValue();
      delay(200);
    }else{
      SlowMoveMotor(0);
      delay(500);
    }
  }
  
  ResetSlowAddition();
  SlowMoveMotor(0);
  delay(500);
  stopMotorRuntime();
}

void setup() {
  Serial.begin(9600);
  MenusInit();
  EEPROM.get(0, PID_constant);
  if(isnan(PID_constant.Kp) || isnan(PID_constant.Ki) || isnan(PID_constant.Kd)){
    PID_constant.Kp = 0;
    PID_constant.Ki = 0;
    PID_constant.Kd = 0;
  }
  PID_constant.Kp = 8;
  PID_constant.Ki = 0;
  PID_constant.Kd = 0;

  SetPoint = 90.0;
  ReTuneKpid(PID_constant.Kp, PID_constant.Ki, PID_constant.Kd);
  MotorInit();
  displayInitialize();
  firstInit();
}

void loop() {
  Serial.print(right_limit);
  Serial.print(", ");
  Serial.print(left_limit);
  Serial.print(", ");
  
  if(digitalRead(LIMIT_1) == 0 || digitalRead(LIMIT_2) == 0){
    stopMotorRuntime();
    system_disabled = true;
  }else {
    referenceMode = getReferenceSource();
    MainMenusRuntime(&PID_constant.Kp, &PID_constant.Ki, &PID_constant.Kd, &SensValue, &SetPoint);
    MainSensorRuntime(&SensValue, &right_limit, &left_limit);
    if(isOnSettingMode() || !getMotorCmdStat()){
      stopMotorRuntime();
    }
    if(isKpidUpdateComplete()){
      ReTuneKpid(PID_constant.Kp, PID_constant.Ki, PID_constant.Kd);
      EEPROM.put(0, PID_constant);
      KpidUpdated();
    }
    if(!isOnSettingMode() || getMotorCmdStat()){
      MainMotorRuntime(&SetPoint, &referenceMode);
    }
  }

  // Serial.println(SensValue);
}

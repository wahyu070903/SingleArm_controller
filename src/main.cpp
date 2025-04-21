#include <Arduino.h>
#include <menus.h>
#include <motor.h>

void setup() {
  Serial.begin(9600);
  MenusInit();
  MotorInit();
}

void loop() {
  MainMenusRuntime();
  MainMotorRuntime();
}

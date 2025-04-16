#include <Arduino.h>
#include <menus.h>

void setup() {
  Serial.begin(9600);
  MenusInit();
}

void loop() {
  MainMenusRuntime();
}

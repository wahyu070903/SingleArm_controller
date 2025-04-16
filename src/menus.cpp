#include <menus.h>
#include <U8g2lib.h>
#include <Wire.h>

U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
U8X8_SSD1306_128X32_UNIVISION_HW_I2C (U8X8_PIN_NONE);

uint8_t buttonPins[] = {3, 4, 7};
const uint8_t numsOfButton = sizeof(buttonPins) / sizeof(buttonPins[1]);
bool buttonState[numsOfButton] = {0};
bool lastReading[numsOfButton] = {0};
unsigned long lastDebounceTime[numsOfButton] = {0};

uint8_t activePages = 0;
uint8_t scrollMenuActive = 0;

void MenusInit(){
    u8g2.begin();
    pinMode(BUTTON_1, INPUT);
    pinMode(BUTTON_2, INPUT);
    pinMode(BUTTON_3, INPUT);
}

void InputsWatcher(){
    for(uint8_t i = 0; i < numsOfButton; i++){
        bool reading = digitalRead(buttonPins[i]);
        if (reading != lastReading[i]) {
            lastDebounceTime[i] = millis(); 
        }
        if ((millis() - lastDebounceTime[i]) > DEBOUNCE_DELAY) {
            if (reading != buttonState[i]) {
              buttonState[i] = reading;
            }
        }
        lastReading[i] = reading;
    }

    // enter config mode
    static unsigned long lastTime = 0;
    static uint8_t isPressed = false;

    if(activePages == 0){
        if(buttonState[0] == true){
            if(isPressed == true){
                if(millis() - lastTime > 5000){
                    activePages = 1;
                }
            }else{
                isPressed = true;
                lastTime = millis();
            }
        }
    }

    if(activePages == 1){
        if(buttonState[1]){
            scrollMenuActive++;
            delay(500);
        }
        if(buttonState[2]){
            scrollMenuActive--;
            delay(500);
        }
    }
}

void MenuMainScreen(){
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 12, "P = 0.00");
    u8g2.drawStr(60, 12, "I = 0.00");
    u8g2.drawStr(0, 24, "D = 0.00");
}

void SecondScreen(){
    const char menusText[3][16] = {
        "Set PID param",
        "other menus",
        "other menus 2",
    };

    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(120, 12, "<");
    u8g2.drawStr(10, 12, menusText[scrollMenuActive]);
    if(scrollMenuActive < 3){
        u8g2.drawStr(10, 24, menusText[scrollMenuActive + 1]);
    }
}

void MainMenusRuntime(){
    u8g2.firstPage();
    do {
        switch(activePages){
            case 0:
                MenuMainScreen();
                break;
            case 1:
                SecondScreen();
                break;
        }
    } while (u8g2.nextPage());
    
    InputsWatcher();
}
#include <menus.h>
#include <Arduino.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

uint8_t buttonPins[] = {3, 4, 7};
const uint8_t numsOfButton = sizeof(buttonPins) / sizeof(buttonPins[1]);
bool buttonState[numsOfButton] = {0};
bool lastReading[numsOfButton] = {0};
bool lastButtonState[numsOfButton] = {0};
bool upEdgeRead[numsOfButton] = {0};
unsigned long lastDebounceTime[numsOfButton] = {0};
bool ReferenceSwitch = false;

uint8_t activePages = 0;
uint8_t scrollMenuActive = 0;
uint8_t pidSettingStep = 0;
bool isScreenNeedUpdate = true;
bool isKpidUpdated = false;
bool isSettingMode = false;
bool isKpidUpdateDone = false;

void MenusInit(){
    lcd.begin(16, 2);
    pinMode(BUTTON_1, INPUT);
    pinMode(BUTTON_2, INPUT);
    pinMode(BUTTON_3, INPUT);
    pinMode(A5, INPUT);
}

bool checkMax(double value){
    if(value >= MAX_CONCTANT) return false; 
    return true;
}

bool checkMin(double value){
    if(value <= MIN_CONSTANT) return false;
    return true;
}

bool isOnSettingMode(){
    return isSettingMode;
}

void InputsWatcher(double* Kp, double* Ki, double* Kd, double* Sp){
    for(uint8_t i = 0; i < numsOfButton; i++){
        bool reading = digitalRead(buttonPins[i]);
        if (reading != lastReading[i]) {
            lastDebounceTime[i] = millis(); 
        }
        if ((millis() - lastDebounceTime[i]) > DEBOUNCE_DELAY) {
            if (reading != buttonState[i]) {
                lastButtonState[i] = buttonState[i];
                buttonState[i] = reading;

                if (lastButtonState[i] == LOW && buttonState[i] == HIGH) {
                    upEdgeRead[i] = true;
                } else {
                    upEdgeRead[i] = false;
                }
            }else{
                upEdgeRead[i] = false;
            }
        }else{
            upEdgeRead[i] = false;
        }
        lastReading[i] = reading;
    }

    ReferenceSwitch = digitalRead(A5);

    // enter config mode
    static unsigned long lastTime = 0;
    static uint8_t isPressed = false;

    if(activePages == 0){
        if(buttonState[0] == true && buttonState[2] == true){
            if(isPressed == true){
                if(millis() - lastTime > 5000){
                    activePages = 1;
                    isScreenNeedUpdate = true;
                    lastTime = 0;
                    isPressed = false;
                }
            }else{
                isPressed = true;
                lastTime = millis();
            }
        }
    }

    static bool upEdge1 = false;
    static bool upEdge2 = false;
    if(activePages == 1){
        if(buttonState[1] && !upEdge1 && scrollMenuActive < 3){
            scrollMenuActive++;
            isScreenNeedUpdate = true;
            upEdge1 = true;
        }else if(!buttonState[1]){
            upEdge1 = false;
        }
        if(buttonState[2] && !upEdge2 && scrollMenuActive > 0){
            scrollMenuActive--;
            isScreenNeedUpdate = true;
            upEdge2 = true;
        }else if(!buttonState[2]){
            upEdge2 = false;
        }

        if(upEdgeRead[0]){
            activePages = activePages + (scrollMenuActive + 1);
            isScreenNeedUpdate = true;
            upEdgeRead[0] = false;
        }
    }

    if(activePages == 2){
        if(upEdgeRead[1]){
            switch(pidSettingStep){
                case 0:
                    if(checkMax(*Kp)){
                        *Kp += 0.01;
                    }
                    break;
                case 1:
                    if(checkMax(*Ki)){
                        *Ki += 0.01;
                    }
                    break;
                case 2:
                    if(checkMax(*Kd)){
                        *Kd += 0.01;
                    }
                    break;
            }
            isScreenNeedUpdate = true;
        }else if(upEdgeRead[2]){
            switch(pidSettingStep){
                case 0:
                    if(checkMin(*Kp)){
                        *Kp -= 0.01;
                    }
                    break;
                case 1:
                    if(checkMin(*Ki)){
                        *Ki -= 0.01;
                    }
                    break;
                case 2:
                    if(checkMin(*Kd)){
                        *Kd -= 0.01;
                    }
                    break;
            }
            isScreenNeedUpdate = true;
        }
        else if(upEdgeRead[0]){
            if(pidSettingStep < 2){
                pidSettingStep++;
            }else{
                isKpidUpdated = true;
                isKpidUpdateDone = true;
                activePages = 0;
                pidSettingStep = 0;
            }
            isScreenNeedUpdate = true;
            upEdgeRead[0] = false;
        }
    }

    if(activePages == 3){
        if(upEdgeRead[1]){
            if(*Sp > 0.0){
                *Sp = *Sp - 1;
                isScreenNeedUpdate = true;
            }
        }
        if(upEdgeRead[2]){
            if(*Sp < 180.0){
                *Sp = *Sp + 1;
                isScreenNeedUpdate = true;
            }
        }
        
        if(upEdgeRead[0]){
            activePages = 0;
            isScreenNeedUpdate = true;
        }
    }
}

void MenuMainScreen(double* Kp, double* Ki, double* Kd, double* Pv){

    lcd.clear();
    char buff[10];
    dtostrf(*Kp, 3, 2, buff);
    lcd.setCursor(0, 0);
    lcd.print("P=");
    lcd.setCursor(2, 0);
    lcd.print(buff);

    dtostrf(*Ki, 3, 2, buff);
    lcd.setCursor(9, 0);
    lcd.print("I=");
    lcd.setCursor(11, 0);
    lcd.print(buff);

    dtostrf(*Kd, 3, 2, buff);
    lcd.setCursor(0, 1);
    lcd.print("D=");
    lcd.setCursor(2, 1);
    lcd.print(buff);

    if(activePages == 2){
        if(pidSettingStep == 0){
            lcd.setCursor(0,0);
        }else if(pidSettingStep == 1){
            lcd.setCursor(9,0);
        }else{
            lcd.setCursor(0,1);
        }
        lcd.cursor();
        lcd.blink();
    }else{
        lcd.noCursor();
        lcd.noBlink();
    }
}

void SecondScreen(){
    const char menusText[3][12] = {
        "SetPIDparam",
        "SetTarget",
        "Back <-",
    };
    lcd.clear();
    lcd.setCursor(15, 0);
    lcd.print("<");
    lcd.setCursor(0, 0);
    lcd.print(menusText[scrollMenuActive]);
    if(scrollMenuActive < 2){
        lcd.setCursor(0, 1);
        lcd.print(menusText[scrollMenuActive + 1]);
    }
}

void SetPointScreen(double* Sp){
    char buff[10];
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SP = ");
    dtostrf(*Sp, 3, 1, buff);
    lcd.print(buff);
}
void screenBack(){
    activePages = 0;
    isScreenNeedUpdate = 0;
}
bool isKpidNeedUpdate(){
    return isKpidUpdated;
}
bool isKpidUpdateComplete(){
    return isKpidUpdateDone;
}
void KpidUpdated(){
    isKpidUpdated = false;
    isKpidUpdateDone = false;
}
bool getReferenceSource(){
    return ReferenceSwitch;
}
void MainMenusRuntime(double* Kp, double* Ki, double* Kd, double* Pv, double* Sp){
    static double lastPv = -1.0;
    Serial.println(activePages);
    if(isScreenNeedUpdate){
        switch(activePages){
            case 0:
                MenuMainScreen(Kp, Ki, Kd, Pv);
                isSettingMode = false;
                scrollMenuActive = 0;
                break;
            case 1:
                SecondScreen();
                isSettingMode = true;
                break;
            case 2:
                MenuMainScreen(Kp, Ki, Kd, Pv);
                isSettingMode = true;
                break;
            case 3:
                SetPointScreen(Sp);
                isSettingMode = true;
                break;
        }
        isScreenNeedUpdate = false;
    }

    if(activePages == 4){
        screenBack();
        if(!isScreenNeedUpdate) isScreenNeedUpdate = true;
    }

    if(activePages == 0 && abs(*Pv - lastPv) > 0.04){
        char buff[10];
        dtostrf(*Pv, 3, 1, buff);
        lcd.setCursor(9, 1);
        lcd.print("       ");
        lcd.setCursor(9, 1);
        lcd.print(buff);
        lcd.print((char)223);
        lastPv = *Pv;
    }

    InputsWatcher(Kp, Ki, Kd, Sp);
}
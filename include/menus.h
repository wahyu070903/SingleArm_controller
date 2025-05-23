#ifndef SYS_MENUS
#define SYS_MENUS

#define BUTTON_1 3
#define BUTTON_2 4
#define BUTTON_3 7
#define BUTTON_COUNT 3
#define DEBOUNCE_DELAY 50
#define MAX_CONSTANT 99.9f
#define MIN_CONSTANT 0.0f
#define LCD_RS 8
#define LCD_EN 9
#define LCD_D4 10
#define LCD_D5 11
#define LCD_D6 12
#define LCD_D7 A3

void MenusInit();
void MainMenusRuntime(double*, double*, double*, double*, double*);
bool isKpidNeedUpdate();
bool isKpidUpdateComplete();
void KpidUpdated();
bool isOnSettingMode();
bool getReferenceSource();
void displayInitialize();
bool getMotorCmdStat();
#endif
#ifndef MOTOR_H
#define MOTOR_H

#define MIN_POTVAL 0
#define MAX_POTVAL 1023
#define POT_RESISTANCE 47000
#define POT_TURN 5
#define SMOOTHING_WINDOW 8
#define JITTER_COMPEN 4

void MotorInit();
void MainSensorRuntime(double*, int*, int*);
void MainMotorRuntime(double*, bool*);
void ReTuneKpid(double, double, double);
void stopMotorRuntime();
void SlowMoveMotor(uint8_t);
void ResetSlowAddition();
int getRawSensorValue();
#endif
#ifndef MOTOR_H
#define MOTOR_H

#define MIN_POTVAL 0
#define MAX_POTVAL 1023
#define POT_RESISTANCE 47000
#define POT_TURN 5
#define SMOOTHING_WINDOW 10

void MotorInit();
void MainMotorRuntime(double*, double*, bool*);
void ReTuneKpid(double, double, double);
void stopMotorRuntime();
#endif
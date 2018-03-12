#ifndef MOTOR_CTRL
#define MOTOR_CTRL
#include "variables.h"
#include "defines.h"
//#include "TimerOne.h"

void configPinMotors(void);
void mBaseSetMicroStep(TYPE_MICRO_STEP step_type);
void mTopSetMicroStep(TYPE_MICRO_STEP step_type);
TYPE_MICRO_STEP nextStepConfig(TYPE_MICRO_STEP step_type);
uint8_t checkBaseLimit(s_motor_ctrl *motor,int16_t *c_limit);
void checkBaseBlock(void);
void limitBaseSteps(void);
#endif


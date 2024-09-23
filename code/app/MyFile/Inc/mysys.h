#ifndef __MYSYS_H
#define __MYSYS_H
#include "stm32f0xx.h"
void InitMysys(void);
void LoopMysys(void);

extern volatile uint8_t config[4][13];

#endif

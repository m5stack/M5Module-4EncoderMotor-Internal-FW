#ifndef __MYSERVOS_H
#define __MYSERVOS_H
#include "stm32f0xx_hal.h"
#define SERVO_CH1 0
#define SERVO_CH2 1

void ServoUpdate(void);
void ServoInit(void);

#endif

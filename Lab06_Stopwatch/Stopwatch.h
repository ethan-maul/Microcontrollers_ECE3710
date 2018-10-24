#ifndef __STM32L476G_DISCOVERY_Stopwatch_H
#define __STM32L476G_DISCOVERY_Stopwatch_H

#include <stdint.h>
#include "LCD.h"

void displayClock(void);
void stopwatch(void);
void init_interrupt(void);
void SysTick_Initialize(void);

#endif /* __STM32L476G_DISCOVERY_Stopwatch_H */

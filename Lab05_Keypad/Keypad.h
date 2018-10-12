#ifndef __STM32L476G_DISCOVERY_KEYPAD_H
#define __STM32L476G_DISCOVERY_KEYPAD_H

#include <stdint.h>








void colCheck(int r);
uint8_t KeyConv(int c, int r);
void display(int c, int r);
void rowCheck(void);
void keypad(void);
void wait(int);

#endif /* __STM32L476G_DISCOVERY_KEYPAD_H */

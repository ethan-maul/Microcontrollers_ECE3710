#include "stm32l476xx.h"
#include "Stopwatch.h"
#include "LCD.h"
#include <stdint.h>

#define __I volatile const //define as read only
#define __O volatile 			 //define as write only
#define __IO volatile 		 //allows both read and write

bool toggle;
uint16_t ms = 0;
uint16_t sec = 0;
uint16_t min = 0;
uint16_t t = 0;
char s[6];

void displayClock(void){
	//time counter
	if(toggle){
		ms++;
		if(ms >= 10){ //if ms reaches limit
			sec++;
			ms = 0;
			if(sec >= 60){ //if sec reaches limit
				sec = 0;
				min++;
				if(min >= 60){ //if min reaches limit, stop counting
					toggle = 0;
				}
			}
		}
	}
	
	//write time into LCD string
	s[1] = (ms % 10) + '0';
	s[2] = (sec % 10) + '0'; //get first digit
	s[3] = (sec / 10) + '0'; //get second digit
	s[4] = (min % 10) + '0';
	s[5] = (min / 10) + '0';
	
	LCD_DisplayString((uint8_t*)s);
}

void EXTI0_IRQHandler(void){ //center, pause the timer
	//check for the EXTI 0 interrupt flag
	if((EXTI->PR1 & EXTI_PR1_PIF0) == EXTI_PR1_PIF0) {
		toggle = 0;
		EXTI->PR1 |= EXTI_PR1_PIF0;
	}
}

void EXTI1_IRQHandler(void){ //left, reset the timer
	//check for the EXTI 1 interrupt flag
	if((EXTI->PR1 & EXTI_PR1_PIF1) == EXTI_PR1_PIF1) {
		ms = 0;
		sec = 0;
		min = 0;
		displayClock();
		EXTI->PR1 |= EXTI_PR1_PIF1;
	}
}

void EXTI2_IRQHandler(void){ //right, starts the timer
	//check for the EXTI 2 interrupt flag
	if((EXTI->PR1 & EXTI_PR1_PIF2) == EXTI_PR1_PIF2) {
		toggle = 1;
		EXTI->PR1 |= EXTI_PR1_PIF2;
	}	
}

void SysTick_Handler(void){//SysTick interrupt service routine
	//TimeDelay is a volatile global variable
	int a = 0;
	while(a < 59999){
		a++;
	}
	displayClock();
}

void SysTick_Initialize(void){//ticks = time interval/clock period
	//Disable SysTick IRQ and SysTick counter
	SysTick->CTRL = 0;
	
	//Set reload register, 
	SysTick->LOAD = 250 - 1;
	
	NVIC_SetPriority(SysTick_IRQn, 15);
	
	SysTick->VAL = 0;
	
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
	
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void init_interrupt(void){
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

	//configure 00: input
	GPIOA->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE2);
	
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1 | GPIO_PUPDR_PUPD2);
	GPIOA->PUPDR |= (GPIO_PUPDR_PUPD0_1 | GPIO_PUPDR_PUPD1_1 | GPIO_PUPDR_PUPD2_1);
	
	NVIC_EnableIRQ(EXTI0_IRQn); //Enable the Interrupt
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI2_IRQn);
	
	//Connect External Line to the GPI
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0; //pin 0, sets to 0:PA pin
	SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI1; //pin 1
	SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI2; //pin 2
	
	//Set EXTI priorities
	NVIC_SetPriority(EXTI0_IRQn, 7);
	NVIC_SetPriority(EXTI1_IRQn, 7);
	NVIC_SetPriority(EXTI2_IRQn, 7);

	//Interrupt Mask Register
	EXTI->IMR1 |= EXTI_IMR1_IM0;
	EXTI->IMR1 |= EXTI_IMR1_IM1;
	EXTI->IMR1 |= EXTI_IMR1_IM2;
	
	EXTI->RTSR1 |= EXTI_RTSR1_RT0;
	EXTI->RTSR1 |= EXTI_RTSR1_RT1;
	EXTI->RTSR1 |= EXTI_RTSR1_RT2;
}

void stopwatch(){
	LCD_Initialization();
	init_interrupt();
	SysTick_Initialize();
	
}

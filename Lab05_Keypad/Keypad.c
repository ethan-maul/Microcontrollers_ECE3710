#include "keypad.h"
#include "stm32l476xx.h"
#include <stdint.h>
#include <time.h>
#include "LCD.h"

//**********PSUEDOCODE***************
//debounce
//***********************************
char s[6];
int p = 0;

void Key_Init(){
	// 00: input mode, 01: output mode, 10: alternate function mode, 11: analog/reset state mode
	
	// Configure Port A Pin 1, 2, 3, 5 input, pull-up
	GPIOA->MODER &= ~(GPIO_MODER_MODE1 | GPIO_MODER_MODE2 | GPIO_MODER_MODE3 | GPIO_MODER_MODE5);
	GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR0_0);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0_1 | GPIO_PUPDR_PUPDR0_1 | GPIO_PUPDR_PUPDR0_1 | GPIO_PUPDR_PUPDR0_1);
	
	// Configure Port E Pin 10, 11, 12, 13 output, open drain
	GPIOE->MODER &= ~(GPIO_MODER_MODE10_0 | GPIO_MODER_MODE11_0 | GPIO_MODER_MODE12_0 | GPIO_MODER_MODE13_0);
	GPIOE->MODER |= (GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1 | GPIO_MODER_MODE12_1 | GPIO_MODER_MODE13_1);	
	GPIOE->OTYPER |= (GPIO_OTYPER_ODR_10 | GPIO_OTYPER_ODR_11 | GPIO_OTYPER_ODR_12 | GPIO_OTYPER_ODR_13);
}

uint8_t KeyConv(int c, int r){
	uint8_t a;
	switch(r){
		case'1':
			switch(c){
				case'1':
					a = '1';
					break;
				case'2':
					a = '2';
					break;
				case'3':
					a = '3';
					break;
				case'4':
					a = 'A';
					break;
				default:
					break;
			}
		case'2':
			switch(c){
				case'1':
					a = '4';
					break;
				case'2':
					a = '5';
					break;
				case'3':
					a = '6';
					break;
				case'4':
					a = 'B';
					break;
				default:
					break;
			}
		case'3':
			switch(c){
				case'1':
					a = '7';
					break;
				case'2':
					a = '8';
					break;
				case'3':
					a = '9';
					break;
				case'4':
					a = 'C';
					break;
				default:
					break;
			}
		case'4':
			switch(c){
				case'1':
					a = '*';
					break;
				case'2':
					a = '0';
					break;
				case'3':
					a = '#';
					break;
				case'4':
					a = 'D';
					break;
				default:
					break;
			}
	}
	return a;
}

void display(int c, int r){
	//find the ascii value
	uint8_t temp = KeyConv(c, r);
	
	//convert to a char to add to our list
	char t = (char)temp;
	
	//only time either of these should change
	s[p] = t;
	p++;
	
	//cast it and run the display
	LCD_DisplayString((uint8_t*)s);
	
	//debounce
	int pad = GPIOA->IDR;
	//don't move until all buttons are released
	while(pad != 1);
	
	return;
}

void colCheck(int r){
	//compare with 1E, or 30. If yes, exit
	int input = GPIOA->IDR;
	if(input == 30)
		return;
	int c = 0;
	//check the different columns by hex value
	switch(input)
	{
		case(0xE):
			c = 1;
			break;
		case(0x16):
			c = 2;
			break;
		case(0x1A):
			c = 3;
			break;
		case(0x1C):
			c = 4;
			break;
		default: 
			c = 0;
			break;
	}
	//if it is one of the columns, update the display
	if(c != 0)
		display(c, r);
	return;
}

void rowCheck(){	
	//no rows
	GPIOE->ODR &= ~(GPIO_ODR_OD10 | GPIO_ODR_OD11 | GPIO_ODR_OD12 | GPIO_ODR_OD13);
	//sleep needs to be 1 microsecond
	wait(5);
	colCheck(0);
	
	//row 1
	GPIOE->ODR &= ~(GPIO_ODR_OD10 | GPIO_ODR_OD11 | GPIO_ODR_OD12 | GPIO_ODR_OD13);
	GPIOE->ODR |= (GPIO_ODR_OD11 | GPIO_ODR_OD12 | GPIO_ODR_OD13);
	wait(5);
	colCheck(1);
	
	//row2
	GPIOE->ODR &= ~(GPIO_ODR_OD10 | GPIO_ODR_OD11 | GPIO_ODR_OD12 | GPIO_ODR_OD13);
	GPIOE->ODR |= (GPIO_ODR_OD10 | GPIO_ODR_OD12 | GPIO_ODR_OD13);	
	wait(5);
	colCheck(2);
	
	//row3
	GPIOE->ODR &= ~(GPIO_ODR_OD10 | GPIO_ODR_OD11 | GPIO_ODR_OD12 | GPIO_ODR_OD13);
	GPIOE->ODR |= (GPIO_ODR_OD10 | GPIO_ODR_OD11 | GPIO_ODR_OD13);	
	wait(5);
	colCheck(3);
	
	//row4
	GPIOE->ODR &= ~(GPIO_ODR_OD10 | GPIO_ODR_OD11 | GPIO_ODR_OD12 | GPIO_ODR_OD13);
	GPIOE->ODR |= (GPIO_ODR_OD10 | GPIO_ODR_OD11 | GPIO_ODR_OD12);	
	wait(5);
	colCheck(4);
}

void keypad(){
	LCD_Initialization();
	LCD_Clear();
	Key_Init();
	
	//once you hit 6, exit
	while(p < 6){
		rowCheck();
	}
	return;
}

// crude sleep function
void wait(int n){
	for(int i = 0; i < n; i++);
}

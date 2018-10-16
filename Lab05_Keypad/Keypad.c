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
	GPIOA->MODER &= ~(GPIO_MODER_MODE1 | GPIO_MODER_MODE2 | GPIO_MODER_MODE3 | GPIO_MODER_MODE5); //configure output mode
	GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR1_0 | GPIO_PUPDR_PUPDR2_0 | GPIO_PUPDR_PUPDR3_0 | GPIO_PUPDR_PUPDR5_0);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1_1 | GPIO_PUPDR_PUPDR2_1 | GPIO_PUPDR_PUPDR3_1 | GPIO_PUPDR_PUPDR5_1);
	
	// Configure Port E Pin 10, 11, 12, 13 output, open drain
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOEEN); //configure gpioe clock
	GPIOE->MODER &= ~(GPIO_MODER_MODE10 | GPIO_MODER_MODE11 | GPIO_MODER_MODE12 | GPIO_MODER_MODE13); //clear moder bits
	GPIOE->MODER |= (GPIO_MODER_MODE10_0 | GPIO_MODER_MODE11_0 | GPIO_MODER_MODE12_0 | GPIO_MODER_MODE13_0); //set to output
	GPIOE->OTYPER |= (GPIO_OTYPER_ODR_10 | GPIO_OTYPER_ODR_11 | GPIO_OTYPER_ODR_12 | GPIO_OTYPER_ODR_13);
}

uint8_t KeyConv(int c, int r){
	uint8_t a;
	switch(r){
		case(1):
			switch(c){
				case(1):
					a = '1';
					break;
				case(2):
					a = '2';
					break;
				case(3):
					a = '3';
					break;
				case(4):
					a = 'A';
					break;
				default:
					break;
			}
			break;
			
		case(2):
			switch(c){
				case(1):
					a = '4';
					break;
				case(2):
					a = '5';
					break;
				case(3):
					a = '6';
					break;
				case(4):
					a = 'B';
					break;
				default:
					break;
			}
			break;
			
		case(3):
			switch(c){
				case(1):
					a = '7';
					break;
				case(2):
					a = '8';
					break;
				case(3):
					a = '9';
					break;
				case(4):
					a = 'C';
					break;
				default:
					break;
			}
			break;
			
		case(4):
			switch(c){
				case(1):
					a = '*';
					break;
				case(2):
					a = '0';
					break;
				case(3):
					a = '#';
					break;
				case(4):
					a = 'D';
					break;
				default:
					break;
			}
			break;
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
	while(!(GPIOA->IDR & GPIO_IDR_ID1) | !(GPIOA->IDR & GPIO_IDR_ID2) | !(GPIOA->IDR & GPIO_IDR_ID3) | !(GPIOA->IDR & GPIO_IDR_ID5));
	wait(500);
	
	return;
}

void colCheck(int r){
	int c = 0;
	
	//check different colums by input values
	if(!(GPIOA->IDR & GPIO_IDR_ID1)){ //check pin 1
		c = 1;
	}
	if(!(GPIOA->IDR & GPIO_IDR_ID2)){ //check pin 2
		c = 2;
	}
	if(!(GPIOA->IDR & GPIO_IDR_ID3)){ //check pin 3
		c = 3;
	}
	if(!(GPIOA->IDR & GPIO_IDR_ID5)){ //check pin 5
		c = 4;
	}
	
	//if it is one of the columns, update the display
	if(c != 0)
		display(c, r);
	return;
}

void rowCheck(){	
	//row 1
	GPIOE->ODR &= ~(GPIO_ODR_OD10 | GPIO_ODR_OD11 | GPIO_ODR_OD12 | GPIO_ODR_OD13); //shut off output
	GPIOE->ODR |= (GPIO_ODR_OD11 | GPIO_ODR_OD12 | GPIO_ODR_OD13);  //turn on bits 11, 12, 13
	wait(500);
	colCheck(1);
	
	//row2
	GPIOE->ODR &= ~(GPIO_ODR_OD10 | GPIO_ODR_OD11 | GPIO_ODR_OD12 | GPIO_ODR_OD13);
	GPIOE->ODR |= (GPIO_ODR_OD10 | GPIO_ODR_OD12 | GPIO_ODR_OD13);	//turn on bits 10, 12, 13
	wait(500);
	colCheck(2);
	
	//row3
	GPIOE->ODR &= ~(GPIO_ODR_OD10 | GPIO_ODR_OD11 | GPIO_ODR_OD12 | GPIO_ODR_OD13);
	GPIOE->ODR |= (GPIO_ODR_OD10 | GPIO_ODR_OD11 | GPIO_ODR_OD13);	//turn on bits 10, 11, 13
	wait(500);
	colCheck(3);
	
	//row4
	GPIOE->ODR &= ~(GPIO_ODR_OD10 | GPIO_ODR_OD11 | GPIO_ODR_OD12 | GPIO_ODR_OD13);
	GPIOE->ODR |= (GPIO_ODR_OD10 | GPIO_ODR_OD11 | GPIO_ODR_OD12);	//turn on bits 10, 11, 12
	wait(500);
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

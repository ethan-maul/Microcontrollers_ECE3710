;*****************************************************************
;USU - ECE 3710 - Microcontroller Hardware and Software
;Lab03_tenBitCounter
;Authors: Tate Patterson and Ethan Maul

;Description: Create a 10 bit pull-down counter with external LED, Voltage source and bar resistor

;Details:  
;Use 220 ohm bar resistor and extra 220 ohm resistor;
;Use open GPIO pins to pull down LED voltage below minimum voltage for LED operation (about 2V)
;dont go above 5V?
;dont use C14, C15
;*****************************************************************

;**************************Constraints****************************
;STM32L4:  STM32L476VGT6 MCU = ARM Cortex-M4 + FPU + DSP, 
;           LQFP100, 1 MB of Flash, 128 KB of SRAM
;           Instruction cache = 32 lines of 4x64 bits (1KB)
;           Data cache = 8 lines of 4x64 bits (256 B)
;
; Joystick (MT-008A): 
;   Right = PA2        Up   = PA3         Center = PA0
;   Left  = PA1        Down = PA5
;
; User LEDs: 
;   LD4 Red   = PB2    LD5 Green = PE8
;   
; CS43L22 Audio DAC Stereo (I2C address 0x94):  
;   SAI1_MCK = PE2     SAI1_SD  = PE6    I2C1_SDA = PB7    Audio_RST = PE3    
;   SAI1_SCK = PE5     SAI1_FS  = PE4    I2C1_SCL = PB6                                           
;
; MP34DT01 Digital MEMS microphone 
;    Audio_CLK = PE9   Audio_DIN = PE7
;
; LSM303C eCompass (a 3D accelerometer and 3D magnetometer module): 
;   MEMS_SCK  = PD1    MAG_DRDY = PC2    XL_CS  = PE0             
;   MEMS_MOSI = PD4    MAG_CS  = PC0     XL_INT = PE1       
;                      MAG_INT = PC1 
;
; L3GD20 Gyro (three-axis digital output): 
;   MEMS_SCK  = PD1    GYRO_CS   = PD7
;   MEMS_MOSI = PD4    GYRO_INT1 = PD2
;   MEMS_MISO = PD3    GYRO_INT2 = PB8
;
; ST-Link V2 (Virtual com port, Mass Storage, Debug port): 
;   USART_TX = PD5     SWCLK = PA14      MFX_USART3_TX   MCO
;   USART_RX = PD6     SWDIO = PA13      MFX_USART3_RX   NRST
;   PB3 = 3V3_REG_ON   SWO = PB5      
;
; Quad SPI Flash Memory (128 Mbit)
;   QSPI_CS  = PE11    QSPI_D0 = PE12    QSPI_D2 = PE14
;   QSPI_CLK = PE10    QSPI_D1 = PE13    QSPI_D3 = PE15
;
; LCD (24 segments, 4 commons)
;   VLCD = PC3
;   COM0 = PA8     COM1  = PA9      COM2  = PA10    COM3  = PB9
;   SEG0 = PA7     SEG6  = PD11     SEG12 = PB5     SEG18 = PD8
;   SEG1 = PC5     SEG7  = PD13     SEG13 = PC8     SEG19 = PB14
;   SEG2 = PB1     SEG8  = PD15     SEG14 = PC6     SEG20 = PB12
;   SEG3 = PB13    SEG9  = PC7      SEG15 = PD14    SEG21 = PB0
;   SEG4 = PB15    SEG10 = PA15     SEG16 = PD12    SEG22 = PC4
;   SEG5 = PD9     SEG11 = PB4      SEG17 = PD10    SEG23 = PA6
; 
; USB OTG
;   OTG_FS_PowerSwitchOn = PC9    OTG_FS_VBUS = PC11    OTG_FS_DM = PA11  
;   OTG_FS_OverCurrent   = PC10   OTG_FS_ID   = PC12    OTG_FS_DP = PA12  
;
; PC14 = OSC32_IN      PC15 = OSC32_OUT
; PH0  = OSC_IN        PH1  = OSC_OUT 
;
; PA4  = DAC1_OUT1 (NLMFX0 WAKEUP)   PA5 = DAC1_OUT2 (Joy Down)
; PA3  = OPAMP1_VOUT (Joy Up)        PB0 = OPAMP2_VOUT (LCD SEG21)
;*****************************************************************

;**************************Psuedocode*****************************
;include constraints
;
;begin main
;   set appropriate offsets and assign registers
;	make each bit assigned to a specific GPIO port	
;
;loop 					;always starts here on reset
;	assign counter value to start at zero
;	
;counter
;	count up the value by 1
;	compare input start/stop values to see if they are true
;	BNE counter 		;branch if not equal back to the counter loop
;
;	hold loop until next value is input to start
;
;	possible button debounce loop if needed	
;	
;end
;*****************************************************************


;*******************BEGIN INITIALIZATION**************************
	INCLUDE core_cm4_constants.s		; Load Constant Definitions
	INCLUDE stm32l476xx_constants.s     ; Load register values 

	AREA    main, CODE, READONLY		;define code below, once on boar it cannot be modified
	EXPORT	__main						; make __main visible to linker
	ENTRY			
				
__main	PROC							;start main
	
	LDR r0, =RCC_BASE					
	LDR r1, [r0, #RCC_AHB2ENR]			
	ORR r1, r1, #RCC_AHB2ENR_GPIOEEN	
	STR r1, [r0, #RCC_AHB2ENR]			
	LDR r0, =GPIOE_BASE		
	LDR r6, =0x55500000	
	LDR r1, [r0, #GPIO_MODER]
	STR r6, [r0, #GPIO_MODER]
	LDR r6, =0xFC00
	LDR r1, [r0, #GPIO_OTYPER]
	ORR r1, r6
	STR r1, [r0, #GPIO_OTYPER]
	LDR r6, =0xAAAAAAAA					;figure out what this 'masking' does
	LDR r1, [r0, #GPIO_PUPDR] 			;adds a pull up, pull down reg to the input
	ORR r1, r6 							;figure out what this does
	STR r1, [r0, #GPIO_PUPDR]
	LDR r6, =0xFFFFFFFF					
	STR r6, [r0, #GPIO_ODR]			

	LDR r2, =RCC_BASE		
	LDR r3, [r2, #RCC_AHB2ENR]				
	ORR r3, r3, #RCC_AHB2ENR_GPIOBEN		
	STR r3, [r2, #RCC_AHB2ENR]			
	LDR r2, =GPIOB_BASE					
	LDR r3, [r2, #GPIO_MODER]
	LDR r6, =0x00005050
	STR r6, [r2, #GPIO_MODER]
	LDR r6, =0xCC
	LDR r3, [r2, #GPIO_OTYPER]
	ORR r3, r6
	STR r3, [r2, #GPIO_OTYPER]
	LDR r6, =0xAAAAAAAA					;figure out what this 'masking' does
	LDR r3, [r2, #GPIO_PUPDR] 			;adds a pull up, pull down reg to the input
	ORR r3, r6 							;figure out what this does
	STR r3, [r2, #GPIO_PUPDR]
	LDR r6, =0xFFFFFFFF	 				
	STR r6, [r2, #GPIO_ODR]
				
	;Joystick setup pins PA0 to PA3 & PA5
	;first value sets input mode on MODER, then feeds input values through PUPDR to input
	LDR r6, =0xFFFFF300 				;value for input mode on joystick
	LDR r4, =RCC_BASE	
	LDR r5, [r4, #RCC_AHB2ENR]				
	ORR r5, r5, #RCC_AHB2ENR_GPIOAEN		
	STR r5, [r4, #RCC_AHB2ENR]			
	LDR r4, =GPIOA_BASE					
	LDR r5, [r4, #GPIO_MODER]	
	AND r5, r6 							;changes PA0-PA3 & PA5 to input mode
	STR r5, [r4, #GPIO_MODER]
	LDR r6, =0x8AA 						;figure out what this 'masking' does
	LDR r5, [r4, #GPIO_PUPDR] 			;adds a pull up, pull down reg to the input
	ORR r5, r6 							;figure out what this does
	STR r5, [r4, #GPIO_PUPDR]
	LDR r6, =0x00000000 				;clears input register
	STR r6, [r4, #GPIO_IDR]
	
	LDR r0, =RCC_BASE					;turn HSI clk on
	LDR r1, [r0, #RCC_CR]
	ORR r1, #0x00000100
	STR r1, [r0, #RCC_CR]
	
HSIclk
	LDR r1, [r0, #RCC_CR]
	LDR r6, =0x00000563	
	CMP r1, r6							;check if HSI is ready
	BEQ HSIclk
	
	LDR r0, =GPIOE_BASE	
	
loop
	LDR r6, =0x1046A	
	mov r12, r6		 					;clk divider
DELAY 
	; center, stop
	LDR r5, [r4, #GPIO_IDR]
	LDR r6, =0x0000A001 				;value to compare joystick to check if center is pushed
	CMP r5, r6 							;compare joystick to check value
	BEQ stop 							;if equal, branch to appropriate logic
	
	; left, reset
	LDR r6, =0x0000A002
	CMP r5, r6
	BEQ reset
	
	; right, start
	LDR r6, =0x0000A004
	CMP r5, r6
	BEQ start
	
	SUBS r12, #1
	CMP r12, #0x0
	bne DELAY
	
	cmp r8, #1 ; If clock is disabled, restart the loop
	BNE loop
	ADD r9, #0x1
	LDR r6, =0x400
	CMP r9, r6
	BNE display
	LDR r9, =0x0
	
display
	;load r9 into r10, mask the appropriate bits and set them to the right point of the output GPIOs
	;bits 0 and 1 of counter, shift them and add them to r11 - to load into GPIO ports 2 and 3
	EOR r10, r9, 0xFFFFFFFF
	MOV r11, #0x0
	AND r10, #0x3
	LSL r10, #2
	ORR r11, r10
	;bits 2 and 3 of counter, shift them and add them to r11 - to load into the GPIO ports 6 and 7
	EOR r10, r9, 0xFFFFFFFF
	AND r10, #0xC
	LSL r10, #4
	ORR r11, r10
	STR r11, [r2, #GPIO_ODR]
	
	EOR r10, r9, 0xFFFFFFFF
	AND r10, #0x3F0
	LSL r10, #6
	STR r10, [r0, #GPIO_ODR]
	
	B loop
	
start ; enable the clock
	LDR r8, =0x1
	b trigger

stop ; disable the clock
	LDR r8, =0x0
	b trigger

reset ; set the counter to 0
	LDR r9, =0x0
	b display
	
trigger ;debouncer
	LDR r5, [r4, #GPIO_IDR]
	CMP r5, #0x0000A000 ;joystick press must be zero for program to leave loop
	BEQ loop
	B trigger
	
	ENDP
					
	ALIGN

	END
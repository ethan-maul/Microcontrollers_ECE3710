;*****************************************************************
;USU - ECE 3710 - Microcontroller Hardware and Software
;Lab02_JoystkLED
;Authors: Tate Patterson and Ethan Maul

;Description: Control red and green LEDs using the on-board joystick.

;Details:  
;a. Center button toggles both LEDs
;b. Left button toggles red LED
;c. Right button toggles green LED
;d. Up button turns both LEDs on
;e. Down button turns both LEDs off
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
;
;	loop
;
;		loop through input values from joystick until one is triggered
;		branch to change LED values for the appropriate joystick value
;
;		**center** - toggles LEDs
;			*toggle on both LEDs*
;			LEDs = !LEDs
;			B debounce
;
;		**left** - toggle red LED
;			*toggle on both LEDs*
;			LEDs = !LEDs
;			B debounce
;
;		**right** - turns on green
;			*toggle on both LEDs*
;			LEDs = !LEDs
;			B debounce
;
;		**up** - turns on LEDs
;			*toggle on both LEDs*
;			LEDs = !LEDs
;			B debounce
;
;		**down** - turns off LEDs
;			*toggle on both LEDs*
;			LEDs = !LEDs
;			B debounce
;
;		**debounce**
;			while joystick is not zero, stay in this loop
;			when zero, branch to loop
;
;	B loop
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
	
	;Green LED setup PE8
	LDR r0, =RCC_BASE					;Load base reset and clk control address into register 0
	LDR r1, [r0, #RCC_AHB2ENR]			;Load base offset by AHB2 peripheral clk enable reg
	ORR r1, r1, #RCC_AHB2ENR_GPIOEEN	;Logical or between r1 and AHB2 port B
	STR r1, [r0, #RCC_AHB2ENR]			;Store back into r1
	LDR r0, =GPIOE_BASE					;Load r0 with GPIOB base address
	LDR r1, [r0, #GPIO_MODER]			;Load r1 with base offset by GPIO mode address
	AND r1, r1, #(0xFFFCFFFF)			;Logical and between r1 and hex value - change MODE2 to input mode 
	ORR r1, r1, #(0x00010000)			;Logical or between r1 and hex value - change MODE2 to GP output mode
	STR r1, [r0, #GPIO_MODER]			;Store r1 into GPIO mode adress
	LDR r1, [r0, #GPIO_ODR]				;Load r1 with output data address - changes output pin
	
	;Red LED setup PB2, almost identical to above
	LDR r2, =RCC_BASE		
	LDR r3, [r2, #RCC_AHB2ENR]				
	ORR r3, r3, #RCC_AHB2ENR_GPIOBEN		
	STR r3, [r2, #RCC_AHB2ENR]			
	LDR r2, =GPIOB_BASE					
	LDR r3, [r2, #GPIO_MODER]			
	AND r3, r3, #(0xFFFFFFCF)			
	ORR r3, r3, #(0x00000010)			
	STR r3, [r2, #GPIO_MODER]			
	LDR r3, [r2, #GPIO_ODR]
				
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
;****************************************************************

;*************BEGIN JOYSTICK CHECK*******************************
loop
	LDR r5, [r4, #GPIO_IDR] 			;load the input into r5
	
;	***compare inputs***
	
	;compare and branch to center
	;all below are similar
	LDR r6, =0x0000A001 				;value to compare joystick to check if center is pushed
	CMP r5, r6 							;compare joystick to check value
	BEQ center 							;if equal, branch to appropriate logic
	
	;compare and branch to left
	LDR r6, =0x0000A002
	CMP r5, r6
	BEQ left
	
	;compare and branch to right
	LDR r6, =0x0000A004
	CMP r5, r6
	BEQ right
	
	;compare and branch to up
	LDR r6, =0x0000A008
	CMP r5, r6
	BEQ up
	
	;compare and branch to down
	LDR r6, =0x0000A020
	CMP r5, r6
	BEQ down
	B loop
;***************************************************************

;*****************BEGIN LED LOGIC*******************************
center ; toggle both
	EOR r1, r1, #(0x00000100) 			; green LED toggle
	STR r1, [r0, #GPIO_ODR]				; send to GPIO
	EOR r3, r3, #(0x00000004) 			; red LED toggle
	STR r3, [r2, #GPIO_ODR]				; send to GPIO
	B trigger							; send to debouncer
	
left ; toggle red
	EOR r3, r3, #(0x00000004) 			; red LED off if on, vice-versa
	STR r3, [r2, #GPIO_ODR]				; send to GPIO
	B trigger
	
right ; toggle green
	EOR r1, r1, #(0x00000100) 			; green LED off if on, vice-versa
	STR r1, [r0, #GPIO_ODR]				; send to GPIO
	B trigger
	
up ; both on
	LDR r1, =0x00000100 				; green LED on
	STR r1, [r0, #GPIO_ODR] 			; send to GPIO
	LDR r3, =0x00000004 				; red LED on
	STR r3, [r2, #GPIO_ODR]				; send to GPIO
	B trigger

down ; both off
	LDR r1, =0x00000000 				; green LED off
	STR r1, [r0, #GPIO_ODR] 			; send to gpio
	LDR r3, =0x00000000 				; red LED off
	STR r3, [r2, #GPIO_ODR]				; send to gpio
	B trigger
	
trigger ;debouncer
	LDR r5, [r4, #GPIO_IDR]
	CMP r5, #0x0000A000 ;joystick press must be zero for program to leave loop
	BEQ loop
	B trigger
;************************************************************************

;****************************END MAIN PROGRAM****************************

	ENDP
					
	ALIGN

	END

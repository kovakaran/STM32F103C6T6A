/*
 ******************************************************************************
 * @file           : stm32f103cx
 * @brief          : This module performs:
 * 						MACRO Global Variables
 * 						define names for CORTEX-M3 Registers
 * 						define names for registers with Register_address
 * 						Interrupt-Handler functions
 * 						define names for Register-Bits
 * 						define name for 32-bit immediate value (set/reset)
 * 						set custom functions
 ******************************************************************************
 */

#include <stdint.h>
void SysTick_Handler(void);


/******************************************************************************
* 								MACRO Global Variables
******************************************************************************/

volatile uint32_t micros = 0;  // To store elapsed microseconds
volatile uint32_t millis = 0;  // To store elapsed milliseconds

uint32_t CORE_CLOCK_FREQ = 72000000;








/******************************************************************************
 *                                                                            *                                                                   *
 * 			 			  CORTEX-M3 Registers                      *
 *                                                                            *
 *****************************************************************************/

//NVIC Registers
#define NVIC_ISER_BASE_ADDR				0xE000E100	/* NVIC Interrupt Set-Enable Register */
#define NVIC_IPR_BASE_ADDR    			0xE000E400  /* NVIC Interrupt Priority Register */

#define NVIC_ISER0   				(*((volatile uint32_t*)NVIC_ISER_ADDR))
#define NVIC_ISER1   				(*((volatile uint32_t*)NVIC_ISER_ADDR + 0x04))

#define NVIC_IPR0    				(*((volatile uint32_t*)NVIC_IPR_BASE_ADDR))
#define NVIC_IPR1    				(*((volatile uint32_t*)NVIC_IPR_BASE_ADDR + 0x04))


// Define SYS_Tick registers
#define SYS_TICK_BASE_ADDR   		0xE000E010
#define NVIC_SYS_TICK_CTRL         (*((volatile unsigned int *)(SYS_TICK_BASE_ADDR )))
#define NVIC_SYS_TICK_RELOAD       (*((volatile unsigned int *)(SYS_TICK_BASE_ADDR + 0x04)))
#define NVIC_SYS_TICK_CURRENT      (*((volatile unsigned int *)(SYS_TICK_BASE_ADDR + 0x08)))



/******************************************************************************
 *                                                                            *                                                                   *
 * 			 			STM32F103CxTx Register table.                         *
 *                                                                            *
 *****************************************************************************/

//Base Address for registers
#define RCC_BASE_ADDR       0x40021000
#define USART1_BASE_ADDR	0x40013800
#define TIM1_BASE_ADDR      0x40012C00
#define GPIOA_BASE_ADDR     0x40010800
#define GPIOB_BASE_ADDR     0x40010C00
#define GPIOC_BASE_ADDR     0x40011000
#define GPIOD_BASE_ADDR     0x40011400
#define GPIOE_BASE_ADDR     0x40011800
#define EXTI_BASE_ADDR		0x40010400
#define USART3_BASE_ADDR	0x40004800
#define USART2_BASE_ADDR	0x40004400
#define TIM5_BASE_ADDR		0x40000C00
#define TIM4_BASE_ADDR		0x40000800
#define TIM3_BASE_ADDR		0x40000400
#define TIM2_BASE_ADDR		0x40000000




//RCC registers
#define RCC_CR				(*(volatile uint32_t*)(RCC_BASE_ADDR))
#define RCC_CFGR			(*(volatile uint32_t*)(RCC_BASE_ADDR + 0x04))
#define RCC_APB2ENR			(*(volatile uint32_t*)(RCC_BASE_ADDR + 0x18))
#define RCC_APB1ENR			(*(volatile uint32_t*)(RCC_BASE_ADDR + 0x1C))

//USART1 Registers
#define USART1_SR			(*(volatile uint32_t*)(USART1_BASE_ADDR))
#define USART1_DR			(*(volatile uint32_t*)(USART1_BASE_ADDR + 0x04))
#define USART1_BRR			(*(volatile uint32_t*)(USART1_BASE_ADDR + 0x08))
#define USART1_CR1			(*(volatile uint32_t*)(USART1_BASE_ADDR + 0x0C))
#define USART1_CR2			(*(volatile uint32_t*)(USART1_BASE_ADDR + 0x10))
#define USART1_CR3			(*(volatile uint32_t*)(USART1_BASE_ADDR + 0x14))
#define USART1_GTPR			(*(volatile uint32_t*)(USART1_BASE_ADDR + 0x18))

//TIM1 Registers
#define TIM1_CR1            (*(volatile uint16_t *)(TIM1_BASE_ADDR + 0x00))
#define TIM1_CR2            (*(volatile uint16_t *)(TIM1_BASE_ADDR + 0x04))
#define TIM1_SMCR           (*(volatile uint16_t *)(TIM1_BASE_ADDR + 0x08))
#define TIM1_DIER           (*(volatile uint16_t *)(TIM1_BASE_ADDR + 0x0C))
#define TIM1_SR             (*(volatile uint16_t *)(TIM1_BASE_ADDR + 0x10))
#define TIM1_EGR            (*(volatile uint16_t *)(TIM1_BASE_ADDR + 0x14))
#define TIM1_CCMR1          (*(volatile uint16_t *)(TIM1_BASE_ADDR + 0x18))
#define TIM1_CCMR2          (*(volatile uint16_t *)(TIM1_BASE_ADDR + 0x1C))
#define TIM1_CCER           (*(volatile uint16_t *)(TIM1_BASE_ADDR + 0x20))
#define TIM1_CNT			(*(volatile uint16_t *)(TIM1_BASE_ADDR + 0x24))
#define TIM1_PSC            (*(volatile uint16_t *)(TIM1_BASE_ADDR + 0x28))
#define TIM1_ARR            (*(volatile uint16_t *)(TIM1_BASE_ADDR + 0x2C))
#define TIM1_RCR            (*(volatile uint16_t *)(TIM1_BASE_ADDR + 0x30))
#define TIM1_CCR1           (*(volatile uint16_t *)(TIM1_BASE_ADDR + 0x34))
#define TIM1_CCR2           (*(volatile uint16_t *)(TIM1_BASE_ADDR + 0x38))
#define TIM1_CCR3           (*(volatile uint16_t *)(TIM1_BASE_ADDR + 0x3C))
#define TIM1_CCR4           (*(volatile uint16_t *)(TIM1_BASE_ADDR + 0x40))
#define TIM1_BDTR           (*(volatile uint16_t *)(TIM1_BASE_ADDR + 0x44))
#define TIM1_DCR            (*(volatile uint16_t *)(TIM1_BASE_ADDR + 0x48))
#define TIM1_DMAR           (*(volatile uint16_t *)(TIM1_BASE_ADDR + 0x4C))


//GPIOA registers
#define GPIOA_CRL			(*(volatile uint32_t*)(GPIOA_BASE_ADDR + 0x00))
#define GPIOA_CRH			(*(volatile uint32_t*)(GPIOA_BASE_ADDR + 0x04))
#define GPIOA_IDR			(*(volatile uint32_t*)(GPIOA_BASE_ADDR + 0x08))
#define GPIOA_ODR			(*(volatile uint32_t*)(GPIOA_BASE_ADDR + 0x0C))
#define GPIOA_BSRR			(*(volatile uint32_t*)(GPIOA_BASE_ADDR + 0x10))
#define GPIOA_BRR			(*(volatile uint32_t*)(GPIOA_BASE_ADDR + 0x14))
#define GPIOA_LCKR			(*(volatile uint32_t*)(GPIOA_BASE_ADDR + 0x18))

//GPIOB registers
#define GPIOB_CRL			(*(volatile uint32_t*)(GPIOB_BASE_ADDR + 0x00))
#define GPIOB_CRH			(*(volatile uint32_t*)(GPIOB_BASE_ADDR + 0x04))
#define GPIOB_IDR			(*(volatile uint32_t*)(GPIOB_BASE_ADDR + 0x08))
#define GPIOB_ODR			(*(volatile uint32_t*)(GPIOB_BASE_ADDR + 0x0C))
#define GPIOB_BSRR			(*(volatile uint32_t*)(GPIOB_BASE_ADDR + 0x10))
#define GPIOB_BRR			(*(volatile uint32_t*)(GPIOB_BASE_ADDR + 0x14))
#define GPIOB_LCKR			(*(volatile uint32_t*)(GPIOB_BASE_ADDR + 0x18))

//GPIOC registers
#define GPIOC_CRL			(*(volatile uint32_t*)(GPIOC_BASE_ADDR + 0x00))
#define GPIOC_CRH			(*(volatile uint32_t*)(GPIOC_BASE_ADDR + 0x04))
#define GPIOC_IDR			(*(volatile uint32_t*)(GPIOC_BASE_ADDR + 0x08))
#define GPIOC_ODR			(*(volatile uint32_t*)(GPIOC_BASE_ADDR + 0x0C))
#define GPIOC_BSRR			(*(volatile uint32_t*)(GPIOC_BASE_ADDR + 0x10))
#define GPIOC_BRR			(*(volatile uint32_t*)(GPIOC_BASE_ADDR + 0x14))
#define GPIOC_LCKR			(*(volatile uint32_t*)(GPIOC_BASE_ADDR + 0x18))

// EXTI Registers
#define EXTI_IMR			(*((volatile uint32_t*)EXTI_BASE_ADDR))
#define EXTI_EMR			(*((volatile uint32_t*)EXTI_BASE_ADDR + 0x04))
#define EXTI_RTSR			(*((volatile uint32_t*)EXTI_BASE_ADDR + 0x08))
#define EXTI_FTSR			(*((volatile uint32_t*)EXTI_BASE_ADDR + 0x0C))
#define EXTI_SWIER			(*((volatile uint32_t*)EXTI_BASE_ADDR + 0x10))
#define EXTI_PR				(*((volatile uint32_t*)EXTI_BASE_ADDR + 0x14))


//USART3 Registers
#define USART3_SR			(*(volatile uint32_t*)(USART3_BASE_ADDR))
#define USART3_DR			(*(volatile uint32_t*)(USART3_BASE_ADDR + 0x04))
#define USART3_BRR			(*(volatile uint32_t*)(USART3_BASE_ADDR + 0x08))
#define USART3_CR1			(*(volatile uint32_t*)(USART3_BASE_ADDR + 0x0C))
#define USART3_CR2			(*(volatile uint32_t*)(USART3_BASE_ADDR + 0x10))
#define USART3_CR3			(*(volatile uint32_t*)(USART3_BASE_ADDR + 0x14))
#define USART3_GTPR			(*(volatile uint32_t*)(USART3_BASE_ADDR + 0x18))


//USART2 Registers
#define USART2_SR			(*(volatile uint32_t*)(USART2_BASE_ADDR))
#define USART2_DR			(*(volatile uint32_t*)(USART2_BASE_ADDR + 0x04))
#define USART2_BRR			(*(volatile uint32_t*)(USART2_BASE_ADDR + 0x08))
#define USART2_CR1			(*(volatile uint32_t*)(USART2_BASE_ADDR + 0x0C))
#define USART2_CR2			(*(volatile uint32_t*)(USART2_BASE_ADDR + 0x10))
#define USART2_CR3			(*(volatile uint32_t*)(USART2_BASE_ADDR + 0x14))
#define USART2_GTPR			(*(volatile uint32_t*)(USART2_BASE_ADDR + 0x18))


//TIM5 Registers
#define TIM5_CR1            (*(volatile uint16_t *)(TIM5_BASE_ADDR + 0x00))
#define TIM5_CR2            (*(volatile uint16_t *)(TIM5_BASE_ADDR + 0x04))
#define TIM5_SMCR           (*(volatile uint16_t *)(TIM5_BASE_ADDR + 0x08))
#define TIM5_DIER           (*(volatile uint16_t *)(TIM5_BASE_ADDR + 0x0C))
#define TIM5_SR             (*(volatile uint16_t *)(TIM5_BASE_ADDR + 0x10))
#define TIM5_EGR            (*(volatile uint16_t *)(TIM5_BASE_ADDR + 0x14))
#define TIM5_CCMR1          (*(volatile uint16_t *)(TIM5_BASE_ADDR + 0x18))
#define TIM5_CCMR2          (*(volatile uint16_t *)(TIM5_BASE_ADDR + 0x1C))
#define TIM5_CCER           (*(volatile uint16_t *)(TIM5_BASE_ADDR + 0x20))
#define TIM5_CNT			(*(volatile uint16_t *)(TIM5_BASE_ADDR + 0x24))
#define TIM5_PSC            (*(volatile uint16_t *)(TIM5_BASE_ADDR + 0x28))
#define TIM5_ARR            (*(volatile uint16_t *)(TIM5_BASE_ADDR + 0x2C))
#define TIM5_CCR1           (*(volatile uint16_t *)(TIM5_BASE_ADDR + 0x34))
#define TIM5_CCR2           (*(volatile uint16_t *)(TIM5_BASE_ADDR + 0x38))
#define TIM5_CCR3           (*(volatile uint16_t *)(TIM5_BASE_ADDR + 0x3C))
#define TIM5_CCR4           (*(volatile uint16_t *)(TIM5_BASE_ADDR + 0x40))
#define TIM5_DCR            (*(volatile uint16_t *)(TIM5_BASE_ADDR + 0x48))
#define TIM5_DMAR           (*(volatile uint16_t *)(TIM5_BASE_ADDR + 0x4C))



//TIM4 Registers
#define TIM4_CR1            (*(volatile uint16_t *)(TIM4_BASE_ADDR + 0x00))
#define TIM4_CR2            (*(volatile uint16_t *)(TIM4_BASE_ADDR + 0x04))
#define TIM4_SMCR           (*(volatile uint16_t *)(TIM4_BASE_ADDR + 0x08))
#define TIM4_DIER           (*(volatile uint16_t *)(TIM4_BASE_ADDR + 0x0C))
#define TIM4_SR             (*(volatile uint16_t *)(TIM4_BASE_ADDR + 0x10))
#define TIM4_EGR            (*(volatile uint16_t *)(TIM4_BASE_ADDR + 0x14))
#define TIM4_CCMR1          (*(volatile uint16_t *)(TIM4_BASE_ADDR + 0x18))
#define TIM4_CCMR2          (*(volatile uint16_t *)(TIM4_BASE_ADDR + 0x1C))
#define TIM4_CCER           (*(volatile uint16_t *)(TIM4_BASE_ADDR + 0x20))
#define TIM4_CNT			(*(volatile uint16_t *)(TIM4_BASE_ADDR + 0x24))
#define TIM4_PSC            (*(volatile uint16_t *)(TIM4_BASE_ADDR + 0x28))
#define TIM4_ARR            (*(volatile uint16_t *)(TIM4_BASE_ADDR + 0x2C))
#define TIM4_CCR1           (*(volatile uint16_t *)(TIM4_BASE_ADDR + 0x34))
#define TIM4_CCR2           (*(volatile uint16_t *)(TIM4_BASE_ADDR + 0x38))
#define TIM4_CCR3           (*(volatile uint16_t *)(TIM4_BASE_ADDR + 0x3C))
#define TIM4_CCR4           (*(volatile uint16_t *)(TIM4_BASE_ADDR + 0x40))
#define TIM4_DCR            (*(volatile uint16_t *)(TIM4_BASE_ADDR + 0x48))
#define TIM4_DMAR           (*(volatile uint16_t *)(TIM4_BASE_ADDR + 0x4C))


//TIM3 Registers
#define TIM3_CR1            (*(volatile uint16_t *)(TIM3_BASE_ADDR + 0x00))
#define TIM3_CR2            (*(volatile uint16_t *)(TIM3_BASE_ADDR + 0x04))
#define TIM3_SMCR           (*(volatile uint16_t *)(TIM3_BASE_ADDR + 0x08))
#define TIM3_DIER           (*(volatile uint16_t *)(TIM3_BASE_ADDR + 0x0C))
#define TIM3_SR             (*(volatile uint16_t *)(TIM3_BASE_ADDR + 0x10))
#define TIM3_EGR            (*(volatile uint16_t *)(TIM3_BASE_ADDR + 0x14))
#define TIM3_CCMR1          (*(volatile uint16_t *)(TIM3_BASE_ADDR + 0x18))
#define TIM3_CCMR2          (*(volatile uint16_t *)(TIM3_BASE_ADDR + 0x1C))
#define TIM3_CCER           (*(volatile uint16_t *)(TIM3_BASE_ADDR + 0x20))
#define TIM3_CNT			(*(volatile uint16_t *)(TIM3_BASE_ADDR + 0x24))
#define TIM3_PSC            (*(volatile uint16_t *)(TIM3_BASE_ADDR + 0x28))
#define TIM3_ARR            (*(volatile uint16_t *)(TIM3_BASE_ADDR + 0x2C))
#define TIM3_CCR1           (*(volatile uint16_t *)(TIM3_BASE_ADDR + 0x34))
#define TIM3_CCR2           (*(volatile uint16_t *)(TIM3_BASE_ADDR + 0x38))
#define TIM3_CCR3           (*(volatile uint16_t *)(TIM3_BASE_ADDR + 0x3C))
#define TIM3_CCR4           (*(volatile uint16_t *)(TIM3_BASE_ADDR + 0x40))
#define TIM3_DCR            (*(volatile uint16_t *)(TIM3_BASE_ADDR + 0x48))
#define TIM3_DMAR           (*(volatile uint16_t *)(TIM3_BASE_ADDR + 0x4C))


//TIM2 Registers
#define TIM2_CR1            (*(volatile uint16_t *)(TIM2_BASE_ADDR + 0x00))
#define TIM2_CR2            (*(volatile uint16_t *)(TIM2_BASE_ADDR + 0x04))
#define TIM2_SMCR           (*(volatile uint16_t *)(TIM2_BASE_ADDR + 0x08))
#define TIM2_DIER           (*(volatile uint16_t *)(TIM2_BASE_ADDR + 0x0C))
#define TIM2_SR             (*(volatile uint16_t *)(TIM2_BASE_ADDR + 0x10))
#define TIM2_EGR            (*(volatile uint16_t *)(TIM2_BASE_ADDR + 0x14))
#define TIM2_CCMR1          (*(volatile uint16_t *)(TIM2_BASE_ADDR + 0x18))
#define TIM2_CCMR2          (*(volatile uint16_t *)(TIM2_BASE_ADDR + 0x1C))
#define TIM2_CCER           (*(volatile uint16_t *)(TIM2_BASE_ADDR + 0x20))
#define TIM2_CNT			(*(volatile uint16_t *)(TIM2_BASE_ADDR + 0x24))
#define TIM2_PSC            (*(volatile uint16_t *)(TIM2_BASE_ADDR + 0x28))
#define TIM2_ARR            (*(volatile uint16_t *)(TIM2_BASE_ADDR + 0x2C))
#define TIM2_CCR1           (*(volatile uint16_t *)(TIM2_BASE_ADDR + 0x34))
#define TIM2_CCR2           (*(volatile uint16_t *)(TIM2_BASE_ADDR + 0x38))
#define TIM2_CCR3           (*(volatile uint16_t *)(TIM2_BASE_ADDR + 0x3C))
#define TIM2_CCR4           (*(volatile uint16_t *)(TIM2_BASE_ADDR + 0x40))
#define TIM2_DCR            (*(volatile uint16_t *)(TIM2_BASE_ADDR + 0x48))
#define TIM2_DMAR           (*(volatile uint16_t *)(TIM2_BASE_ADDR + 0x4C))




/******************************************************************************                                                                           *                                                                          *
 * 			 					Register Bit-Names.                           *            *
 *****************************************************************************/
#define RCC_CR_PLLRDY						0x02000000
#define RCC_CR_PLLON						0x01000000
#define RCC_CR_HSEON						0x00010000
#define RCC_CR_HSERDY						0x00020000
#define RCC_CFGR_PLLMULx2					0x00000000
#define RCC_CFGR_PLLMULx3					0x00040000
#define RCC_CFGR_PLLMULx4					0x00080000
#define RCC_CFGR_PLLMULx5					0x000C0000
#define RCC_CFGR_PLLMULx6					0x00100000
#define RCC_CFGR_PLLMULx7					0x00140000
#define RCC_CFGR_PLLMULx8					0x00180000
#define RCC_CFGR_PLLMULx9					0x001C0000
#define RCC_CFGR_PLLMULx10					0x00200000
#define RCC_CFGR_PLLXTPRE					0x00020000
#define RCC_CFGR_PLLSRC_HSE					0x00010000
#define RCC_CFGR_PPRE1_0					0X00000000
#define RCC_CFGR_PPRE1_2					0x00000400
#define RCC_CFGR_PPRE1_4					0x00000500
#define RCC_CFGR_PPRE1_8					0x00000600
#define RCC_CFGR_PPRE1_16					0x00000700
#define RCC_CFGR_SWS_PLL					0x00000008
#define RCC_CFGR_SW_HSI						0x00000000
#define RCC_CFGR_SW_HSE						0x00000001
#define RCC_CFGR_SW_PLL						0x00000002


#define TIMx_CR1_ARPE						0x0080
#define TIMx_CR1_DIR_UP						0x0000
#define TIMx_CR1_DIR_DOWN					0x0010
#define TIMx_CR1_URS_SET					0x0004
#define TIMx_CR1_URS_RESET					0x0000
#define TIMx_CR1_UDIS_ENABLE				0x0000
#define TIMx_CR1_UDIS_DISABLE				0x0002
#define TIMx_CR1_CEN						0x0001
#define TIMx_CCMR1_OC1M_PWM_MODE_1			0x0060
#define TIMx_CCMR1_OC1M_PWM_MODE_2			0x0070
#define TIMx_CCMR1_OC1PE					0x0008
#define TIMx_CCER_CC1E						0x0001
#define TIMx_BDTR_MOE						0x8000


/******************************************************************************                                                                           *                                                                          *
 * 			 			Interrupt-Handler Functions                           *
 *****************************************************************************/

// SysTick_Handler function will be called every microsecond
void SysTick_Handler(void) {
    micros++;
}






/******************************************************************************
* 							PORT-BIT Set/Reset names.
******************************************************************************/

//Set names

#define SET_BIT_0 		0x00000001
#define SET_BIT_1 		0x00000002
#define SET_BIT_2 		0x00000004
#define SET_BIT_3 		0x00000008

#define SET_BIT_4 		0x00000010
#define SET_BIT_5 		0x00000020
#define SET_BIT_6 		0x00000040
#define SET_BIT_7 		0x00000080

#define SET_BIT_8 		0x00000100
#define SET_BIT_9 		0x00000200
#define SET_BIT_10 		0x00000400
#define SET_BIT_11 		0x00000800

#define SET_BIT_12		0x00001000
#define SET_BIT_13 		0x00002000
#define SET_BIT_14 		0x00004000
#define SET_BIT_15 		0x00008000

#define SET_BIT_ALL		0XFFFFFFFF

//Clear names

#define CLEAR_BIT_0 		~0x00000001
#define CLEAR_BIT_1 		~0x00000002
#define CLEAR_BIT_2 		~0x00000004
#define CLEAR_BIT_3 		~0x00000008

#define CLEAR_BIT_4 		~0x00000010
#define CLEAR_BIT_5 		~0x00000020
#define CLEAR_BIT_6 		~0x00000040
#define CLEAR_BIT_7 		~0x00000080

#define CLEAR_BIT_8 		~0x00000100
#define CLEAR_BIT_9 		~0x00000200
#define CLEAR_BIT_10 		~0x00000400
#define CLEAR_BIT_11 		~0x00000800

#define CLEAR_BIT_12		~0x00001000
#define CLEAR_BIT_13 		~0x00002000
#define CLEAR_BIT_14 		~0x00004000
#define CLEAR_BIT_15 		~0x00008000

#define CLEAR_BIT_ALL		 0x00000000







/******************************************************************************
* 								Custom Functions
******************************************************************************/

void System_Clock_Init(void);
void SysTick_Init(void);
void Tim1_Pwm_Init(void);
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);


void System_Clock_Init(void)
{
	RCC_CFGR |= RCC_CFGR_PPRE1_2;								// APB1 Sets to 36MHz
	RCC_CR |= RCC_CR_HSEON;										// Enable HSE (High-Speed External) clock
	while(!(RCC_CR & RCC_CR_HSERDY));							// Wait until HSE is ready

	RCC_CFGR |=	RCC_CFGR_PLLSRC_HSE;							// HSE is PLL input clock source
	RCC_CFGR |= RCC_CFGR_PLLMULx8;								// PLL multiplication factor (9)
	RCC_CR |= RCC_CR_PLLON;										// Enable PLL
	while(!(RCC_CR & RCC_CR_PLLRDY));							// Wait until PLL is ready


	RCC_CFGR |= RCC_CFGR_SW_PLL;								// Set PLL as the system clock source
	while(!(RCC_CFGR & RCC_CFGR_SWS_PLL));						// Wait until PLL becomes the system clock source

}



//SysTick not initialization , has to be debug?
// Initialize the System Tick Timer
void SysTick_Init(void)
{
    NVIC_SYS_TICK_RELOAD = ( 64U - 1U );  						// Set reload value for a 1us tick (adjust based on your clock frequency)
    NVIC_SYS_TICK_CURRENT = 0U; 								// Clear current value
    NVIC_SYS_TICK_CTRL = 0x07;  								// Enable SysTick, use core clock, and enable interrupts
}







// Simple delay function using the SysTick timer for milliseconds
void delay_ms(uint32_t ms)
{
	uint32_t start = micros/1000;
	while( ( (micros / 1000)- start ) < ms);
}




// Simple delay function using the SysTick timer for microseconds
void delay_us(uint32_t us) {
    uint32_t start = micros;
    while ((micros - start) < us);
}



#include "stm32f103cx.h"

void Gpio_Init(void);
void Tim1_Pwm_Init(void);
void delay(uint32_t sec);


int main(void)
{
	System_Clock_Init();		// 64 MHz Clock set as SystemClock
	Gpio_Init();
	Tim1_Pwm_Init();

	while(1)
	{
		for(uint16_t i = 0U; i < 20000U; i++)			// Voltage rise from 0 to Max
		{
			TIM1_CCR1 = i;
			delay(2);
		}
		for(uint16_t i = 20000U ; i > 0U; i--)			// Voltage drop from Max to 0
		{
			TIM1_CCR1 = i;
			delay(2);
		}

	}

}

void Gpio_Init(void)
{
	RCC_APB2ENR |= 0x00000005;    		// Enable GPIOA CLOCK
	GPIOA_CRH &= 0xFFFFFFF0;
	GPIOA_CRH |= 0x0000000A;			// Enables GPIOA8 mode as Alternative Function Push/Pull
}


void Tim1_Pwm_Init(void)
{
	RCC_APB2ENR |=	SET_BIT_11;					// Enable clock for TIM1

	TIM1_PSC = ( 64U - 1U );					// TIM1 Pre-scaler for a 1 MHz clock (adjust based on your clock frequency)
	TIM1_ARR = 20000U -1U ;						// Configure TIM1 period for a PWM frequency of 50 Hz

	TIM1_CR1 |=  0x0080;						// Enable Auto-Reload PreLoad
	TIM1_CCMR1 = 0x6060 ;						// PWM mode 1 and Enable PreLoad
	TIM1_CCER |= 0x0011;						// Enable output for CCR1
	TIM1_BDTR |= 0x8000;						// Enable TIM1 main output
	TIM1_CR1 |= 0x0001;							// Enable TIM1 counter

}



void delay(uint32_t sec)
{
		sec = sec * 64; 						 // Adjust for your actual clock frequency
	    for(int i = 0; i < sec; i++);
}


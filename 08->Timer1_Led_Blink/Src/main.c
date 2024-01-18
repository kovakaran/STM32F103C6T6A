#include "stm32f103cx.h"

void Gpio_Init(void);
void Tim1_Init(void);
void Delay_Ms(uint16_t ms);

int main(void)
{
	System_Clock_Init();		//To enable External clock
	Gpio_Init();
	Tim1_Init();

	while(1)
	{
		GPIOA_ODR = 0x00000100;
		Delay_Ms(1000);
		GPIOA_ODR = 0x00000000;
		Delay_Ms(1000);
	}
}


void Gpio_Init(void)
{

	RCC_APB2ENR |= 0x00000005;    		// Enable GPIOA CLOCK
	GPIOA_CRH &= 0xFFFFFFF0;			// GPIOA8 set as output PP
	GPIOA_CRH |= 0x00000002;
}


void Tim1_Init(void)
{
	RCC_APB2ENR |=	SET_BIT_11;						// Enable clock for TIM1
	TIM1_PSC = ( 64000U - 1U );						// TIM1 Pre-scaler for a 1 KHz clock (adjust based on your clock frequency)
	TIM1_ARR = 0xFFFF;								// 16-bit ARR is set to maximum
	TIM1_CR1 |=  TIMx_CR1_ARPE;						// Enable Auto-Reload PreLoad
	TIM1_CR1 |= TIMx_CR1_CEN;						// Enable TIM1 counter
}

void Delay_Ms(uint16_t ms)
{
	    uint16_t startValue = TIM1_CNT;
	    while ((uint16_t)(TIM1_CNT - startValue) < ms);
}

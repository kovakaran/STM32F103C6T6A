#include "stm32f103cx.h"

void RCC_APB2_Init(void);
void EXTI_Init(void);
void EXTI0_IRQHandler(void);


int main(void)
{
	RCC_APB2_Init();
	EXTI_Init();
}

void RCC_APB2_Init(void)
{
    RCC_APB2ENR |= 0x00000005;		// Enable GPIO-A Clock

    GPIOA_CRL &= 0xffffff00;		// Set A0 for internal pull-down, A1 for output
    GPIOA_CRL |= 0x00000028;

    GPIOA_ODR = 0x00000002;			// Set Output-high
}


void EXTI_Init(void)
{

	// Configure EXTI Line0
	EXTI_IMR = 0x00000001;			//Un-mask A0 for External Interrupt
	EXTI_RTSR = 0x00000001;			//Set Rising-edge as Interrupt

	// Set the interrupt priority
	NVIC_IPR0 &= ~0xFF;  			// Clear priority bits
    NVIC_IPR0 |= (0 << 4);  		// Set priority to the lowest value (highest priority)

	// Enable the interrupt
	NVIC_ISER0 |= (1 << 6);  		// EXTI0_IRQn is at position 6 in ISER register

}


void EXTI0_IRQHandler(void)
{
	if(EXTI_PR == 0x00000001)
	{
		EXTI_PR = 0x00000001;			//Clear Pending register
		GPIOA_ODR = ~GPIOA_ODR;
	}
}

#include <stdio.h>
#include "stm32f103cx.h"

void Gpio_Init(void);
void EXTI_Init(void);
void EXTI0_IRQHandler(void);
void Uart1_Init(void);
void Send_Char(char c);
void Send_String(char* str);

int main(void)
{
   System_Clock_Init();				// 64 MHz Clock Initialized
   Gpio_Init();
   EXTI_Init();
   Uart1_Init();

   while(1)
   {
	   if (USART1_SR & 0x0020)			// Wait until receiver data register is not empty
	   {
	       	uint8_t rx = USART1_DR;
	        if (rx == '1')
	        {
	        	GPIOA_ODR = ~GPIOA_ODR;
	        }
	   }
   }
}


void Gpio_Init(void)
{
	RCC_APB2ENR |= SET_BIT_0;			// AFIO Clock enable
	RCC_APB2ENR |= SET_BIT_2;			// GPIOA Clock enable

	GPIOA_CRH &= 0xFFFFF000;			// A9(TX) -> Set AFIO P/P , A10(RX) -> set INPUT PU/PD, A8(LED) -> Set as output PP
	GPIOA_CRH |= 0x000008A2;

	GPIOA_CRL &= 0xfffffff0;			// A0 -> internal pull-down
	GPIOA_CRL |= 0x00000008;
}


void EXTI_Init(void)
{

	// Configure EXTI Line0
	EXTI_IMR = 0x00000001;		//Un-mask A0 for External Interrupt
	EXTI_RTSR = 0x00000001;		//Set Rising-edge as Interrupt

	// Set the interrupt priority
	NVIC_IPR0 &= ~0xFF;  // Clear priority bits
    NVIC_IPR0 |= (0 << 4);  // Set priority to the lowest value (highest priority)

	// Enable the interrupt
	NVIC_ISER0 |= (1 << 6);  // EXTI0_IRQn is at position 6 in ISER register

}

void EXTI0_IRQHandler(void)
{

	if(EXTI_PR == 0x00000001)
	{
		EXTI_PR = 0x00000001;	//Clear Pending register
			Send_Char('1');
	}

}


void Uart1_Init(void)
{
	RCC_APB2ENR |= SET_BIT_14;				// USART1 Clock enable
	USART1_BRR = 0xd055;					// Set 1200 as Baud-rate (Clock - 64Mhz) -> Adjust based on your Clock
	USART1_CR1 |= ( SET_BIT_3 ) | ( SET_BIT_2 ) | ( SET_BIT_13 ); 		//Enable TX, RX, UART
}

void Send_Char(char c) {

    while (!(USART1_SR & 0x0080));				// Wait until transmit data register is empty
    USART1_DR = c;		    					// Send character
}



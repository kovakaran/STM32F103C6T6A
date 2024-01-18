#include "stm32f103cx.h"

void Gpio_Init(void);
void Uart1_Init(void);
void Send_Char(char c);
void Send_String(char* str);

int main(void)
{
   System_Clock_Init();				// 64 MHz Clock Initialized
   Gpio_Init();
   Uart1_Init();
   while(1)
   {
	   if (USART1_SR & 0x0020)			// Wait until receiver data register is not empty
	   {
	       	uint8_t rx = USART1_DR;
	        if (rx == '1')
	        {
	        	GPIOA_ODR = SET_BIT_8;
	        	Send_String("LED_ON\r\n");
	        }
	        else if(rx == '0')
	        {
	        	GPIOA_ODR = CLEAR_BIT_8;
	        	Send_String("LED_OFF\r\n");
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


}


void Uart1_Init(void)
{
	RCC_APB2ENR |= SET_BIT_14;				// USART1 Clock enable
	USART1_BRR = 0xd055;					// Set 1200 as Baud-rate (Clock - 64Mhz) -> Adjust based on your Clock
	USART1_CR1 |= ( SET_BIT_3 ) | ( SET_BIT_2 ) | ( SET_BIT_13 ); 		//Enable TX, RX, UART
	Send_String("UART-1 Communication Started\r\n");			//Send String on Port-A9
	Send_String("Press 1-ON_LED, 0->OFF_LED\r\n");
}

void Send_Char(char c) {

    while (!(USART1_SR & 0x0080));				// Wait until transmit data register is empty
    USART1_DR = c;		    					// Send character
}


void Send_String(char* str)
{
	while (*str != '\0') {
	        Send_Char(*str);
	        str++;
	    }
}

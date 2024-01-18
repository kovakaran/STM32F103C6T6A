#include <stdint.h>
#include "stm32f103cx.h"


void GPIO_Init(void);
void Stepper_Motor_Start(void);



int main(void)
{
   GPIO_Init();
   Stepper_Motor_Start();
}

void GPIO_Init(void)
{

	RCC_APB2ENR = SET_BIT_2;							//Initialize GPIOA Clock
	GPIOA_CRL &= 0xFFFF0000;GPIOA_CRL |= 0x00002222;	//Set->GPIOA0-A3 as Output-Push/Pull

}

void Stepper_Motor_Start(void)
{

	while(1)
	{
		GPIOA_ODR = SET_BIT_0;
		delay_ms(10);
		GPIOA_ODR = SET_BIT_1;
		delay_ms(10);
		GPIOA_ODR = SET_BIT_2;
		delay_ms(10);
		GPIOA_ODR = SET_BIT_3;
		delay_ms(10);
	}
}

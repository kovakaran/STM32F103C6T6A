#include "stm32f103cx.h"

void Gpio_Init(void);
void Trigger(void);
void Measure(void);
void Tim1_Init(void);
void LCD_Init(void); // Function to initialize the LCD
void LCD_SendCommand(uint8_t command); // Function to send a command to the LCD
void LCD_SendData(uint8_t data); // Function to send data to the LCD
void LCD_PrintString(char* str); // Function to print a string on the LCD



int main(void)
{
    System_Clock_Init();
    SysTick_Init();
    Gpio_Init();
    LCD_Init(); // Initialize LCD
    LCD_SendCommand(0x80);   // Move cursor to the beginning of the first line
    LCD_PrintString("~~~~~Line 1~~~~~");



    while(1)
    {
    Measure();
    delay_ms(1000);
    }
}


void Gpio_Init(void)
{
RCC_APB2ENR |= SET_BIT_2; // GPIO-A Clock enable
GPIOA_CRH &= 0xFFFFFF00; // A8->Set as Output, A9->Set as input floating
GPIOA_CRH |= 0x00000042;
}



void Tim1_Init(void)
{
	RCC_APB2ENR |=	SET_BIT_11;						// Enable clock for TIM1
	TIM1_PSC = ( 640U - 1U );						// TIM1 Pre-scaler for a 100 KHz clock (adjust based on your clock frequency)
	TIM1_ARR = 0xFFFF;								// 16-bit ARR is set to maximum
	TIM1_CR1 |=  TIMx_CR1_ARPE;						// Enable Auto-Reload PreLoad
	TIM1_CR1 |= TIMx_CR1_CEN;						// Enable TIM1 counter
}


void LCD_Init(void)
{
    delay_us(20000); // Wait for LCD to power up

    RCC_APB2ENR |= (SET_BIT_2 | SET_BIT_4);   // Enable GPIOA and GPIOC clock
    GPIOA_CRL = 0x22222222;   //Set->GPIOA0-A7 as Output-Push/Pull
    GPIOC_CRH &= 0x00FFFFFF;GPIOC_CRH |= 0x22000000; //Set->GPIOC14-C15 as Output-Push/Pull

    // Initialize LCD
    LCD_SendCommand(0x38);   // 8-bit data, 2-line display, 5x8 font
    LCD_SendCommand(0x0C);   // Display on, cursor off
    LCD_SendCommand(0x01);   // Clear display
    delay_us(2000);
    LCD_SendCommand(0x06);   // Entry mode set: Increment cursor
}


void LCD_SendCommand(uint8_t command)
{
    GPIOC_ODR &= CLEAR_BIT_14; // RS = 0 (Command mode)
    GPIOA_ODR = (uint16_t)(command);   // Send command to data pins
    GPIOC_ODR |= SET_BIT_15; // Enable LCD
    delay_us(10);
    GPIOC_ODR &= ~SET_BIT_15;   // Disable LCD
}


void LCD_SendData(uint8_t data)
{
    GPIOC_ODR |= SET_BIT_14;   // RS = 1 (Data mode)
    GPIOA_ODR = (uint16_t)(data);   // Send data to data pins
    GPIOC_ODR |= SET_BIT_15; // Enable LCD
    delay_us(10);
    GPIOC_ODR &= ~SET_BIT_15;   // Disable LCD
}


void LCD_PrintString(char* str)
{
    while (*str) {
        LCD_SendData(*str++);
    }
}


void Trigger(void)
{
GPIOA_ODR |= SET_BIT_8;   // Set bit-8
delay_us(100);
GPIOA_ODR &= CLEAR_BIT_8; // Clear bit-8
}

void Measure(void)
{
	Trigger();

	while (!(GPIOA_IDR & 0x00000200));  // Wait for rising edge on (PA9)
	uint32_t initial_time = TIM1_CNT;
	while (GPIOA_IDR & 0x00000200); // Wait for falling edge on (PA9)
	uint32_t time_taken =TIM1_CNT - initial_time;

	uint16_t distance = (33000 * time_taken) / 200000; //divide by 1MHz to get Speed-of-Sound in microsecond

	LCD_SendCommand(0xC0); // Move cursor to the beginning of the second line
	LCD_SendData( (distance)/100 + 48 );
    LCD_SendData( (distance/10)%10 + 48 );
    LCD_SendData( (distance%10) + 48 );
}

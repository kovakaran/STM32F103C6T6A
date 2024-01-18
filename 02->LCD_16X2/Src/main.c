#include "stm32f103cx.h"

void DELAY_us(uint32_t us);				// Function to provide a delay in microseconds
void LCD_Init(void);					// Function to initialize the LCD
void LCD_SendCommand(uint8_t command);	// Function to send a command to the LCD
void LCD_SendData(uint8_t data);		// Function to send data to the LCD
void LCD_PrintString(char* str);		// Function to print a string on the LCD


void DELAY_us(uint32_t us) {
    us *= 8;  							// Adjust for your clock frequency
    while (us--) {}
}



int main(void)
{
    LCD_Init();								// Initialize LCD
    LCD_SendCommand(0x80);  				// Move cursor to the beginning of the first line
    LCD_PrintString("~~~~~Line 1~~~~~");
    LCD_SendCommand(0xC0); 					// Move cursor to the beginning of the second line
    LCD_PrintString("~~~~~Line 2~~~~~");

    while (1);
}



void LCD_Init(void)
{
	DELAY_us(20000);									// Wait for LCD to power up

    RCC_APB2ENR |= (SET_BIT_2 | SET_BIT_4);  			// Enable GPIOA and GPIOC clock
    GPIOA_CRL = 0x22222222;  							//Set->GPIOA0-A7 as Output-Push/Pull
    GPIOC_CRH &= 0x00FFFFFF;GPIOC_CRH |= 0x22000000;	//Set->GPIOC14-C15 as Output-Push/Pull

    // Initialize LCD
    LCD_SendCommand(0x38);  							// 8-bit data, 2-line display, 5x8 font
    LCD_SendCommand(0x0C);  							// Display on, cursor off
    LCD_SendCommand(0x01);  							// Clear display
    DELAY_us(2000);
    LCD_SendCommand(0x06);  							// Entry mode set: Increment cursor
}


void LCD_SendCommand(uint8_t command)
{
    GPIOC_ODR &= CLEAR_BIT_14; 			 // RS = 0 (Command mode)
    GPIOA_ODR = (uint16_t)(command);  	 // Send command to data pins
    GPIOC_ODR |= SET_BIT_15; 			 // Enable LCD
    DELAY_us(10);
    GPIOC_ODR &= ~SET_BIT_15;  			 // Disable LCD
}


void LCD_SendData(uint8_t data)
{
    GPIOC_ODR |= SET_BIT_14;  			 // RS = 1 (Data mode)
    GPIOA_ODR = (uint16_t)(data);  		 // Send data to data pins
    GPIOC_ODR |= SET_BIT_15;			 // Enable LCD
    DELAY_us(10);
    GPIOC_ODR &= ~SET_BIT_15;  			 // Disable LCD
}


void LCD_PrintString(char* str)
{
    while (*str) {
        LCD_SendData(*str++);
    }
}

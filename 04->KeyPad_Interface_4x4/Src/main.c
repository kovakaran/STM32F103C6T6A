#include "stm32f103cx.h"

void GPIO_Init(void);
void KEYPAD_Read(void);

void LCD_Init(void); // Function to initialize the LCD
void LCD_SendCommand(uint8_t command); // Function to send a command to the LCD
void LCD_SendData(uint8_t data); // Function to send data to the LCD
void LCD_PrintString(char* str); // Function to print a string on the LCD

int main(void)
{
		GPIO_Init();
		LCD_Init(); // Initialize LCD
		LCD_SendCommand(0x80);   // Move cursor to the beginning of the first line
		LCD_PrintString("~~~~~Line 1~~~~~");
		while(1){
		KEYPAD_Read();
		}
}



void GPIO_Init(void)
{
	RCC_APB2ENR |= ( SET_BIT_2 | SET_BIT_3 );			// Enable Clock for GPIO-A, GPIO-B

	GPIOA_CRH &= 0xFFFF0000;							// A8-A11 -> Set as Output PP
	GPIOA_CRH |= 0x00002222;

	GPIOB_CRL &= 0xFFFF0000;							// B0-B3 -> Set as Input Pull-Down
	GPIOB_CRL |= 0x00008888;
}



void LCD_Init(void)
{
    delay_us(20000); 								 // Wait for LCD to power up

    RCC_APB2ENR |= (SET_BIT_2 | SET_BIT_4);   		 // Enable GPIOA and GPIOC clock
    GPIOA_CRL = 0x22222222;  						 //Set->GPIOA0-A7 as Output-Push/Pull
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



void KEYPAD_Read(void)
{
	uint8_t keypad_matrix[4][4] = {
	        						{'1', '2', '3', 'A'},
									{'4', '5', '6', 'B'},
									{'7', '8', '9', 'C'},
	        						{'*', '0', '#', 'D'}
	    							};

	for(int row = 0; row < 4; row++)
	{
		GPIOA_ODR = (1 << (row + 8));		// Moves to A8-bit(A8-A11 are rows)
		for(int col = 0; col < 4; col++)
		{
			if( (GPIOB_IDR & (1 << col)) )
			{
				LCD_SendCommand(0xC0); // Move cursor to the beginning of the second line
				 LCD_SendData( keypad_matrix[row][col] );
			}
		}

		GPIOA_ODR = 0x0000;					// Reset Rows

	}
}

/*
 * ST7920_SERIAL.c
 *
 *  Created on: 07-Jun-2019
 *      Author: poe
 */

#include "stm32f1xx_hal.h"
#include "ST7920_SERIAL.h"
#include "delay.h"
#include "font.h"


/* setup below is as follows
 * A5 ---------> SCLK (EN)
 * A6 ---------> CS (RS)
 * A7 ---------> SID (RW)
 * B0 ---------> RST (RST)
 *
 */

#define SCLK_PIN GPIO_PIN_5
#define SCLK_PORT GPIOA
#define CS_PIN GPIO_PIN_6
#define CS_PORT GPIOA
#define SID_PIN GPIO_PIN_7
#define SID_PORT GPIOA
#define RST_PIN GPIO_PIN_0
#define RST_PORT GPIOB


uint8_t startRow, startCol, endRow, endCol; // coordinates of the dirty rectangle
uint8_t numRows = 64;
uint8_t numCols = 128;
uint8_t Graphic_Check = 0;
uint8_t image[(128 * 64)/8];
extern SPI_HandleTypeDef hspi1;

// A replacement for SPI_TRANSMIT

void SendByteSPI(uint8_t byte)
{
	for(int i=0;i<8;i++)
	{
		if((byte<<i)&0x80)
			{
				HAL_GPIO_WritePin(SID_PORT, SID_PIN, GPIO_PIN_SET);  // SID=1  OR MOSI
			}

		else HAL_GPIO_WritePin(SID_PORT, SID_PIN, GPIO_PIN_RESET);  // SID=0

		HAL_GPIO_WritePin(SCLK_PORT, SCLK_PIN, GPIO_PIN_RESET);  // SCLK =0  OR SCK

		HAL_GPIO_WritePin(SCLK_PORT, SCLK_PIN, GPIO_PIN_SET);  // SCLK=1

	}
}




void ST7920_SendCmd (uint8_t cmd)
{
	uint16_t data;
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);  // PUll the CS high
	
	data=0xf8+(0<<1);
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&data,1,HAL_MAX_DELAY);
	//SendByteSPI(0xf8+(0<<1));  // send the SYNC + RS(0)
	
	data=cmd&0xf0;
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&data,1,HAL_MAX_DELAY);
	//SendByteSPI(cmd&0xf0);  // send the higher nibble first
	
	data=(cmd<<4)&0xf0;
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&data,1,HAL_MAX_DELAY);
	//SendByteSPI((cmd<<4)&0xf0);  // send the lower nibble
	delay_us(50);

	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);  // PUll the CS LOW

}

void ST7920_SendData (uint8_t data)
{

	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);  // PUll the CS high
	
	uint8_t sync=0xf8+(1<<1);						//SPI.transfer(0xFA);
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&sync,1,HAL_MAX_DELAY);
	//SendByteSPI(0xf8+(1<<1));  // send the SYNC + RS(1)
	
	uint8_t data_temp=data&0xf0;
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&data_temp,1,HAL_MAX_DELAY);
	//SendByteSPI(data&0xf0);  // send the higher nibble first
	
	data_temp=(data<<4)&0xf0;
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&data_temp,1,HAL_MAX_DELAY);
	//SendByteSPI((data<<4)&0xf0);  // send the lower nibble
	delay_us(50);
	
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);  // PUll the CS LOW
}

void ST7920_SendString(int row, int col, char* string)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0x90;
            break;
        case 2:
            col |= 0x88;
            break;
        case 3:
            col |= 0x98;
            break;
        default:
            col |= 0x80;
            break;
    }

    ST7920_SendCmd(col);

    while (*string)
    	{
    		ST7920_SendData(*string++);
    	}
}



// switch to graphic mode or normal mode::: enable = 1 -> graphic mode enable = 0 -> normal mode

void ST7920_GraphicMode (int enable)   // 1-enable, 0-disable
{
	if (enable == 1)
	{
		ST7920_SendCmd(0x30);  // 8 bit mode
		HAL_Delay (1);
		ST7920_SendCmd(0x34);  // switch to Extended instructions
		HAL_Delay (1);
		ST7920_SendCmd(0x36);  // enable graphics
		HAL_Delay (1);
		Graphic_Check = 1;  // update the variable
	}

	else if (enable == 0)
	{
		ST7920_SendCmd(0x30);  // 8 bit mode
		HAL_Delay (1);
		Graphic_Check = 0;  // update the variable
	}
}

void ST7920_GraphicsDisplayString(unsigned char line, unsigned char col, char* str, unsigned char flip)
{
	// Output text to the LCD in graphics mode using a 5x7 font
	// Each char occupies 6 cols, 8 lines, including of space to next char
	// Flip = 1 ==> Flips all bits of the world ==> Highlights text
	unsigned char row;
	unsigned char colInd;
	unsigned char shouldContinue = 1;
	unsigned char count = 0;

	while(*str && shouldContinue)
	{
		unsigned char letterA = *str++;
		if (letterA == 0)
		{
			letterA = 32;
			shouldContinue = 0;
		}

		// if string length is odd, last letter does not come in pair, append space			
		unsigned char letterB = *str++;
		if (letterB == 0) 
		{
			letterB = 32; // odd number of characters in a string, replace NULL with space		
			shouldContinue = 0;
		}

		unsigned char indA = letterA < 0x52 ? letterA - 0x20 : letterA - 0x52;
		unsigned char indB = letterB < 0x52 ? letterB - 0x20 : letterB - 0x52;
		unsigned char colListA[5];
		unsigned char colListB[5];
	
	    if(letterA < 0x52){
			colListA[4] = Alpha1[(indA*5)];
			colListA[3] = Alpha1[(indA*5)+1];
			colListA[2] = Alpha1[(indA*5)+2];
			colListA[1] = Alpha1[(indA*5)+3];
			colListA[0] = Alpha1[(indA*5)+4];
		}
		else
		{
			colListA[4] = Alpha2[(indA*5)];
			colListA[3] = Alpha2[(indA*5)+1];
			colListA[2] = Alpha2[(indA*5)+2];
			colListA[1] = Alpha2[(indA*5)+3];
			colListA[0] = Alpha2[(indA*5)+4];
		}
	
	    if(letterB < 0x52){
			colListB[4] = Alpha1[(indB*5)];
			colListB[3] = Alpha1[(indB*5)+1];
			colListB[2] = Alpha1[(indB*5)+2];
			colListB[1] = Alpha1[(indB*5)+3];
			colListB[0] = Alpha1[(indB*5)+4];
		}
		else
		{
			colListB[4] = Alpha2[(indB*5)];
			colListB[3] = Alpha2[(indB*5)+1];
			colListB[2] = Alpha2[(indB*5)+2];
			colListB[1] = Alpha2[(indB*5)+3];
			colListB[0] = Alpha2[(indB*5)+4];
		}
		
		for (row=0;row<8;row++)
		{
			if (line < 4)	// first half
			{
				ST7920_SendCmd(0x80 | (line * 8 + row));
				//ST7920_SendCmd(0x80 | count);
				ST7920_SendCmd(0x80 | (col + count));
			}
			else
			{
				ST7920_SendCmd(0x80 | ( (line-4) * 8 + row));
				ST7920_SendCmd(0x88 | (col +count));
			}
					
			unsigned char dataA = 0x00;
			for (colInd=0;colInd<5;colInd++)
			{
				if (colListA[colInd] & (1 << row))
				{
					dataA = dataA | (1 << (colInd+3));
				}
			}

			unsigned char dataB = 0x00;
			for (colInd=0;colInd<5;colInd++)
			{
				if (colListB[colInd] & (1 << row))
				{
					dataB = dataB | (1 << (colInd+3));
				}
			}
			
			if (flip == 0)
			{
				ST7920_SendData(dataA);
				ST7920_SendData(dataB);
			}
			else
			{
				ST7920_SendData(~dataA);
				ST7920_SendData(~dataB);
			}
		}

		count++;
	}
}

void ST7920_GraphicsDisplayString5x7(unsigned char line, unsigned char col, char* str, unsigned char flip)
{
	// Output text to the LCD in graphics mode using a 5x7 font
	// Each char occupies 5 cols, 8 lines + including of space to next char
	// Como se accede de a 8 columnas, armo grupos de 4 letras (5 col + 1 espacio)
	// 6 * 4 ==> 24 ==> 24 / 8 ==> 3  Aprovecho los espacios al máximo
	// Flip = 1 ==> Flips all bits of the world ==> Highlights text
	unsigned char row;
	unsigned char colInd;
	unsigned char shouldContinue = 1;
	unsigned char count = 0;

	while(*str && shouldContinue)
	{
		unsigned char letter1 = *str++;
		if (letter1 == 0)
		{
			letter1 = 32;
			shouldContinue = 0;
		}

		// if string length is odd, last letter does not come in pair, append space			
		unsigned char letter2 = *str++;
		if (letter2 == 0) 
		{
			letter2 = 32; // odd number of characters in a string, replace NULL with space		
			shouldContinue = 0;
		}
		
		unsigned char letter3 = *str++;
		if (letter3 == 0) 
		{
			letter3 = 32; // odd number of characters in a string, replace NULL with space		
			shouldContinue = 0;
		}

		unsigned char letter4 = *str++;
		if (letter4 == 0) 
		{
			letter4 = 32; // odd number of characters in a string, replace NULL with space		
			shouldContinue = 0;
		}
		
		unsigned char letter5 = *str++;
		if (letter5 == 0) 
		{
			letter5 = 32; // odd number of characters in a string, replace NULL with space		
			shouldContinue = 0;
		}
		
		unsigned char letter6 = *str++;
		if (letter6 == 0) 
		{
			letter6 = 32; // odd number of characters in a string, replace NULL with space		
			shouldContinue = 0;
		}
		
		unsigned char letter7 = *str++;
		if (letter7 == 0) 
		{
			letter7 = 32; // odd number of characters in a string, replace NULL with space		
			shouldContinue = 0;
		}
		
		unsigned char letter8 = *str++;
		if (letter8 == 0) 
		{
			letter8 = 32; // odd number of characters in a string, replace NULL with space		
			shouldContinue = 0;
		}
		
		unsigned char ind1 = letter1 < 0x52 ? letter1 - 0x20 : letter1 - 0x52;
		unsigned char ind2 = letter2 < 0x52 ? letter2 - 0x20 : letter2 - 0x52;
		unsigned char ind3 = letter3 < 0x52 ? letter3 - 0x20 : letter3 - 0x52;
		unsigned char ind4 = letter4 < 0x52 ? letter4 - 0x20 : letter4 - 0x52;
		unsigned char ind5 = letter5 < 0x52 ? letter5 - 0x20 : letter5 - 0x52;
		unsigned char ind6 = letter6 < 0x52 ? letter6 - 0x20 : letter6 - 0x52;
		unsigned char ind7 = letter7 < 0x52 ? letter7 - 0x20 : letter7 - 0x52;
		unsigned char ind8 = letter8 < 0x52 ? letter8 - 0x20 : letter8 - 0x52;
		unsigned char colList1[8];
		unsigned char colList2[8];
		unsigned char colList3[8];
		unsigned char colList4[8];
		unsigned char colList5[8];
		unsigned char colList6[8];
	
	  if(letter1 < 0x52)
		{
			colList1[7] = Alpha1[(ind1*5)];
			colList1[6] = Alpha1[(ind1*5)+1];
			colList1[5] = Alpha1[(ind1*5)+2];
			colList1[4] = Alpha1[(ind1*5)+3];
			colList1[3] = Alpha1[(ind1*5)+4];
		}
		else
		{
			colList1[7] = Alpha2[(ind1*5)];
			colList1[6] = Alpha2[(ind1*5)+1];
			colList1[5] = Alpha2[(ind1*5)+2];
			colList1[4] = Alpha2[(ind1*5)+3];
			colList1[3] = Alpha2[(ind1*5)+4];
		}
		
		colList1[2] = 0x00; // Fill space between letters
		
	  if(letter2 < 0x52)
		{
			colList1[1] = Alpha1[(ind2*5)];
			colList1[0] = Alpha1[(ind2*5)+1];
			colList2[7] = Alpha1[(ind2*5)+2];
			colList2[6] = Alpha1[(ind2*5)+3];
			colList2[5] = Alpha1[(ind2*5)+4];
		}
		else
		{
			colList1[1] = Alpha2[(ind2*5)];
			colList1[0] = Alpha2[(ind2*5)+1];
			colList2[7] = Alpha2[(ind2*5)+2];
			colList2[6] = Alpha2[(ind2*5)+3];
			colList2[5] = Alpha2[(ind2*5)+4];
		}
		
		colList2[4] = 0x00; // Fill space between letters
		
		if(letter3 < 0x52)
		{
			colList2[3] = Alpha1[(ind3*5)];
			colList2[2] = Alpha1[(ind3*5)+1];
			colList2[1] = Alpha1[(ind3*5)+2];
			colList2[0] = Alpha1[(ind3*5)+3];
			colList3[7] = Alpha1[(ind3*5)+4];
		}
		else
		{
			colList2[3] = Alpha2[(ind3*5)];
			colList2[2] = Alpha2[(ind3*5)+1];
			colList2[1] = Alpha2[(ind3*5)+2];
			colList2[0] = Alpha2[(ind3*5)+3];
			colList3[7] = Alpha2[(ind3*5)+4];
		}
		
		colList3[6] = 0x00; // Fill space between letters

		if(letter4 < 0x52)
		{
			colList3[5] = Alpha1[(ind4*5)];
			colList3[4] = Alpha1[(ind4*5)+1];
			colList3[3] = Alpha1[(ind4*5)+2];
			colList3[2] = Alpha1[(ind4*5)+3];
			colList3[1] = Alpha1[(ind4*5)+4];
		}
		else
		{
			colList3[5] = Alpha2[(ind4*5)];
			colList3[4] = Alpha2[(ind4*5)+1];
			colList3[3] = Alpha2[(ind4*5)+2];
			colList3[2] = Alpha2[(ind4*5)+3];
			colList3[1] = Alpha2[(ind4*5)+4];
		}
		
		colList3[0] = 0x00; // Fill space between letters
		
		if(letter5 < 0x52)
		{
			colList4[7] = Alpha1[(ind5*5)];
			colList4[6] = Alpha1[(ind5*5)+1];
			colList4[5] = Alpha1[(ind5*5)+2];
			colList4[4] = Alpha1[(ind5*5)+3];
			colList4[3] = Alpha1[(ind5*5)+4];
		}
		else
		{
			colList4[7] = Alpha2[(ind5*5)];
			colList4[6] = Alpha2[(ind5*5)+1];
			colList4[5] = Alpha2[(ind5*5)+2];
			colList4[4] = Alpha2[(ind5*5)+3];
			colList4[3] = Alpha2[(ind5*5)+4];
		}
		
		colList4[2] = 0x00; // Fill space between letters
		
	  if(letter6 < 0x52)
		{
			colList4[1] = Alpha1[(ind6*5)];
			colList4[0] = Alpha1[(ind6*5)+1];
			colList5[7] = Alpha1[(ind6*5)+2];
			colList5[6] = Alpha1[(ind6*5)+3];
			colList5[5] = Alpha1[(ind6*5)+4];
		}
		else
		{
			colList4[1] = Alpha2[(ind6*5)];
			colList4[0] = Alpha2[(ind6*5)+1];
			colList5[7] = Alpha2[(ind6*5)+2];
			colList5[6] = Alpha2[(ind6*5)+3];
			colList5[5] = Alpha2[(ind6*5)+4];
		}
		
		colList5[4] = 0x00; // Fill space between letters
		
		if(letter7 < 0x52)
		{
			colList5[3] = Alpha1[(ind7*5)];
			colList5[2] = Alpha1[(ind7*5)+1];
			colList5[1] = Alpha1[(ind7*5)+2];
			colList5[0] = Alpha1[(ind7*5)+3];
			colList6[7] = Alpha1[(ind7*5)+4];
		}
		else
		{
			colList5[3] = Alpha2[(ind7*5)];
			colList5[2] = Alpha2[(ind7*5)+1];
			colList5[1] = Alpha2[(ind7*5)+2];
			colList5[0] = Alpha2[(ind7*5)+3];
			colList6[7] = Alpha2[(ind7*5)+4];
		}
		
		colList6[6] = 0x00; // Fill space between letters

		if(letter8 < 0x52)
		{
			colList6[5] = Alpha1[(ind8*5)];
			colList6[4] = Alpha1[(ind8*5)+1];
			colList6[3] = Alpha1[(ind8*5)+2];
			colList6[2] = Alpha1[(ind8*5)+3];
			colList6[1] = Alpha1[(ind8*5)+4];
		}
		else
		{
			colList6[5] = Alpha2[(ind8*5)];
			colList6[4] = Alpha2[(ind8*5)+1];
			colList6[3] = Alpha2[(ind8*5)+2];
			colList6[2] = Alpha2[(ind8*5)+3];
			colList6[1] = Alpha2[(ind8*5)+4];
		}
		
		colList6[0] = 0x00; // Fill space between letters

		for (row=0;row<8;row++)
		{
			if (line < 4)	// first half
			{
				ST7920_SendCmd(0x80 | (line * 8 + row));
				ST7920_SendCmd(0x80 | (col + count));
			}
			else
			{
				ST7920_SendCmd(0x80 | ( (line-4) * 8 + row));
				ST7920_SendCmd(0x88 | (col + count));
			}
					
			unsigned char data1 = 0x00;
			for (colInd=0;colInd<8;colInd++)
			{
				if (colList1[colInd] & (1 << row))
				{
					data1 = data1 | (1 << (colInd));
				}
			}

			unsigned char data2 = 0x00;
			for (colInd=0;colInd<8;colInd++)
			{
				if (colList2[colInd] & (1 << row))
				{
					data2 = data2 | (1 << (colInd));
				}
			}
			
			unsigned char data3 = 0x00;
			for (colInd=0;colInd<8;colInd++)
			{
				if (colList3[colInd] & (1 << row))
				{
					data3 = data3 | (1 << (colInd));
				}
			}
			
			unsigned char data4 = 0x00;
			for (colInd=0;colInd<8;colInd++)
			{
				if (colList4[colInd] & (1 << row))
				{
					data4 = data4 | (1 << (colInd));
				}
			}
			
			unsigned char data5 = 0x00;
			for (colInd=0;colInd<8;colInd++)
			{
				if (colList5[colInd] & (1 << row))
				{
					data5 = data5 | (1 << (colInd));
				}
			}
			
			unsigned char data6 = 0x00;
			for (colInd=0;colInd<8;colInd++)
			{
				if (colList6[colInd] & (1 << row))
				{
					data6 = data6 | (1 << (colInd));
				}
			}
			
			if (flip == 0)
			{
				if ((count > 3) && ((letter2 == 0x20) | (letter3 == 0x20) | (letter4 == 0x20) | (letter5 == 0x20)
					| (letter6 == 0x20) | (letter7 == 0x20) | (letter8 == 0x20)))	// Evito un bug de corrimiento en la memoria
				{
					data4 = 0x00;
					data5 = 0x00;
					data6 = 0x00;
					ST7920_SendData(data1);
					ST7920_SendData(data2);
					ST7920_SendData(data3);
					ST7920_SendData(data4);
					ST7920_SendData(data5);
					ST7920_SendData(data6);
				}
				else if ((((letter2 | letter3) == 0x20) | ((letter3 | letter4) == 0x20) | 
					((letter4 | letter5) == 0x20) | ((letter5 | letter6) == 0x20) | ((letter6 | letter7) == 0x20)))
				{
					data4 = 0x00;
					data5 = 0x00;
					data6 = 0x00;
					ST7920_SendData(data1);
					ST7920_SendData(data2);
					ST7920_SendData(data3);
					ST7920_SendData(data4);
					ST7920_SendData(data5);
					ST7920_SendData(data6);
				}
				else
				{
					ST7920_SendData(data1);
					ST7920_SendData(data2);
					ST7920_SendData(data3);
					ST7920_SendData(data4);
					ST7920_SendData(data5);
					ST7920_SendData(data6);
				}
			}
			else
			{
				if ((count > 3) && ((letter2 == 0x20) | (letter3 == 0x20) | (letter4 == 0x20) | (letter5 == 0x20)
					| (letter6 == 0x20) | (letter7 == 0x20) | (letter8 == 0x20)))	// Evito un bug de corrimiento en la memoria
				{
					data4 = 0x00;
					data5 = 0x00;
					data6 = 0x00;
					ST7920_SendData(~data1);
					ST7920_SendData(~data2);
					ST7920_SendData(~data3);
					ST7920_SendData(~data4);
					ST7920_SendData(~data5);
					ST7920_SendData(~data6);
				}
				else if ((((letter2 | letter3) == 0x20) | ((letter3 | letter4) == 0x20) | 
					((letter4 | letter5) == 0x20) | ((letter5 | letter6) == 0x20) | ((letter6 | letter7) == 0x20)))
				{
					data4 = 0x00;
					data5 = 0x00;
					data6 = 0x00;
					ST7920_SendData(~data1);
					ST7920_SendData(~data2);
					ST7920_SendData(~data3);
					ST7920_SendData(data4);
					ST7920_SendData(data5);
					ST7920_SendData(data6);
				}
				else
				{
					ST7920_SendData(~data1);
					ST7920_SendData(~data2);
					ST7920_SendData(~data3);
					ST7920_SendData(~data4);
					ST7920_SendData(~data5);
					ST7920_SendData(~data6);
				}
			}
		}
		count = count + 3; // Avanzo para escribir los siguientes ocho caracteres
	}
}

void ST7920_HighlightMenuItem(unsigned char idx, int fill) {
  //Fill must be 0 or 1
	idx &= 0x03; // 4 rows only

  unsigned char y = idx * 16;
  unsigned char x_addr = 0x80;
	
  // adjust cooridinates and address
  if (y >= 32) {
    y -= 32;
    x_addr = 0x88;
  }

  for (unsigned char x = 0; x < 8; x++) {
    ST7920_SendCmd(0x80 | y);
    ST7920_SendCmd(x_addr | x);
    fill ? ST7920_SendData(0xFF) : ST7920_SendData(0x00);
    fill ? ST7920_SendData(0xFF) : ST7920_SendData(0x00);

    ST7920_SendCmd(0x80 | y + 15);
    ST7920_SendCmd(x_addr | x);
    fill ? ST7920_SendData(0xFF) : ST7920_SendData(0x00);
    fill ? ST7920_SendData(0xFF) : ST7920_SendData(0x00);
  }

  for (unsigned char y1 = y + 1; y1 < y + 15; y1++) {
    ST7920_SendCmd(0x80 | y1);
    ST7920_SendCmd(x_addr);
    fill ? ST7920_SendData(0x80) : ST7920_SendData(0x00);
    ST7920_SendData(0x00);

    ST7920_SendCmd(0x80 | y1);
    ST7920_SendCmd(x_addr + 7);
    ST7920_SendData(0x00);
    fill ? ST7920_SendData(0x01) : ST7920_SendData(0x00);
  }
}

void ST7920_DrawBitmap(const unsigned char* graphic)
{
	uint8_t x, y;
	for(y = 0; y < 64; y++)
	{
		if(y < 32)
		{
			for(x = 0; x < 8; x++)							// Draws top half of the screen.
			{												// In extended instruction mode, vertical and horizontal coordinates must be specified before sending data in.
				ST7920_SendCmd(0x80 | y);				// Vertical coordinate of the screen is specified first. (0-31)
				ST7920_SendCmd(0x80 | x);				// Then horizontal coordinate of the screen is specified. (0-8)
				ST7920_SendData(graphic[2*x + 16*y]);		// Data to the upper byte is sent to the coordinate.
				ST7920_SendData(graphic[2*x+1 + 16*y]);	// Data to the lower byte is sent to the coordinate.
			}
		}
		else
		{
			for(x = 0; x < 8; x++)							// Draws bottom half of the screen.
			{												// Actions performed as same as the upper half screen.
				ST7920_SendCmd(0x80 | (y-32));			// Vertical coordinate must be scaled back to 0-31 as it is dealing with another half of the screen.
				ST7920_SendCmd(0x88 | x);
				ST7920_SendData(graphic[2*x + 16*y]);
				ST7920_SendData(graphic[2*x+1 + 16*y]);
			}
		}

	}
}


// Update the display with the selected graphics
void ST7920_Update(void)
{
	ST7920_DrawBitmap(image);
}



void ST7920_Clear(void)
{
	if (Graphic_Check == 1)  // if the graphic mode is set
	{
		uint8_t x, y;
		for(y = 0; y < 64; y++)
		{
			if(y < 32)
			{
				ST7920_SendCmd(0x80 | y);
				ST7920_SendCmd(0x80);
			}
			else
			{
				ST7920_SendCmd(0x80 | (y-32));
				ST7920_SendCmd(0x88);
			}
			for(x = 0; x < 8; x++)
			{
				ST7920_SendData(0);
				ST7920_SendData(0);
			}
		}
	}

	else
	{
		ST7920_SendCmd(0x01);   // clear the display using command
		HAL_Delay(2); // delay >1.6 ms
	}
}


void ST7920_Init (void)
{
	
	HAL_GPIO_WritePin(RST_PORT, RST_PIN, GPIO_PIN_RESET);  // RESET=0
	HAL_Delay(10);   // wait for 10ms
	HAL_GPIO_WritePin(RST_PORT, RST_PIN, GPIO_PIN_SET);  // RESET=1
	
	HAL_Delay(50);   //wait for >40 ms


	ST7920_SendCmd(0x30);  // 8bit mode												LCD_BASIC       0x30
	//delay_us(110);  //  >100us delay
	HAL_Delay(10);
	
	ST7920_SendCmd(0x30);  // 8bit mode 											LCD_BASIC       0x30
	//delay_us(40);  // >37us delay
	HAL_Delay(10);
	
	ST7920_SendCmd(0x08);  // D=0, C=0, B=0										LCD_DISPLAYOFF  0x08
	//delay_us(110);  // >100us delay
	HAL_Delay(10);
	
	ST7920_SendCmd(0x01);  // clear screen										LCD_CLS         0x01
	HAL_Delay(12);  // >10 ms delay


	ST7920_SendCmd(0x06);  // cursor increment right no shift	LCD_ADDRINC     0x06
	HAL_Delay(1);  // 1ms delay

	ST7920_SendCmd(0x0C);  // D=1, C=0, B=0										LCD_DISPLAYON   0x0C
    HAL_Delay(1);  // 1ms delay

	ST7920_SendCmd(0x02);  // return to home									LCD_HOME        0x02
	HAL_Delay(1);  // 1ms delay

}



// set Pixel

void SetPixel(uint8_t x, uint8_t y)
{
  if (y < numRows && x < numCols)
  {
    uint8_t *p = image + ((y * (numCols/8)) + (x/8));
    *p |= 0x80u >> (x%8);

    *image = *p;

    // Change the dirty rectangle to account for a pixel being dirty (we assume it was changed)
    if (startRow > y) { startRow = y; }
    if (endRow <= y)  { endRow = y + 1; }
    if (startCol > x) { startCol = x; }
    if (endCol <= x)  { endCol = x + 1; }


  }

}

/* draw a line
 * start point (X0, Y0)
 * end point (X1, Y1)
 */
void DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
  int dx = (x1 >= x0) ? x1 - x0 : x0 - x1;
  int dy = (y1 >= y0) ? y1 - y0 : y0 - y1;
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  for (;;)
  {
    SetPixel(x0, y0);
    if (x0 == x1 && y0 == y1) break;
    int e2 = err + err;
    if (e2 > -dy)
    {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx)
    {
      err += dx;
      y0 += sy;
    }
  }
}




/* Draw rectangle
 * start point (x,y)
 * w -> width
 * h -> height
 */
void DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
	/* Check input parameters */
	if (
		x >= numCols ||
		y >= numRows
	) {
		/* Return error */
		return;
	}

	/* Check width and height */
	if ((x + w) >= numCols) {
		w = numCols - x;
	}
	if ((y + h) >= numRows) {
		h = numRows - y;
	}

	/* Draw 4 lines */
	DrawLine(x, y, x + w, y);         /* Top line */
	DrawLine(x, y + h, x + w, y + h); /* Bottom line */
	DrawLine(x, y, x, y + h);         /* Left line */
	DrawLine(x + w, y, x + w, y + h); /* Right line */
}




/* Draw filled rectangle
 * Start point (x,y)
 * w -> width
 * h -> height
 */
void DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
	uint8_t i;

	/* Check input parameters */
	if (
		x >= numCols ||
		y >= numRows
	) {
		/* Return error */
		return;
	}

	/* Check width and height */
	if ((x + w) >= numCols) {
		w = numCols - x;
	}
	if ((y + h) >= numRows) {
		h = numRows - y;
	}

	/* Draw lines */
	for (i = 0; i <= h; i++) {
		/* Draw lines */
		DrawLine(x, y + i, x + w, y + i);
	}
}




/* draw circle
 * centre (x0,y0)
 * radius = radius
 */
void DrawCircle(uint8_t x0, uint8_t y0, uint8_t radius)
{
  int f = 1 - (int)radius;
  int ddF_x = 1;

  int ddF_y = -2 * (int)radius;
  int x = 0;

  SetPixel(x0, y0 + radius);
  SetPixel(x0, y0 - radius);
  SetPixel(x0 + radius, y0);
  SetPixel(x0 - radius, y0);

  int y = radius;
  while(x < y)
  {
    if(f >= 0)
    {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
    SetPixel(x0 + x, y0 + y);
    SetPixel(x0 - x, y0 + y);
    SetPixel(x0 + x, y0 - y);
    SetPixel(x0 - x, y0 - y);
    SetPixel(x0 + y, y0 + x);
    SetPixel(x0 - y, y0 + x);
    SetPixel(x0 + y, y0 - x);
    SetPixel(x0 - y, y0 - x);
  }
}


// Draw Filled Circle

void DrawFilledCircle(int16_t x0, int16_t y0, int16_t r)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    SetPixel(x0, y0 + r);
    SetPixel(x0, y0 - r);
    SetPixel(x0 + r, y0);
    SetPixel(x0 - r, y0);
    DrawLine(x0 - r, y0, x0 + r, y0);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        DrawLine(x0 - x, y0 + y, x0 + x, y0 + y);
        DrawLine(x0 + x, y0 - y, x0 - x, y0 - y);

        DrawLine(x0 + y, y0 + x, x0 - y, y0 + x);
        DrawLine(x0 + y, y0 - x, x0 - y, y0 - x);
    }
}



// Draw Traingle with coordimates (x1, y1), (x2, y2), (x3, y3)
void DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3)
{
	/* Draw lines */
	DrawLine(x1, y1, x2, y2);
	DrawLine(x2, y2, x3, y3);
	DrawLine(x3, y3, x1, y1);
}



// Draw Filled Traingle with coordimates (x1, y1), (x2, y2), (x3, y3)
void DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3)
{
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
	yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
	curpixel = 0;

#define ABS(x)   ((x) > 0 ? (x) : -(x))

	deltax = ABS(x2 - x1);
	deltay = ABS(y2 - y1);
	x = x1;
	y = y1;

	if (x2 >= x1) {
		xinc1 = 1;
		xinc2 = 1;
	} else {
		xinc1 = -1;
		xinc2 = -1;
	}

	if (y2 >= y1) {
		yinc1 = 1;
		yinc2 = 1;
	} else {
		yinc1 = -1;
		yinc2 = -1;
	}

	if (deltax >= deltay){
		xinc1 = 0;
		yinc2 = 0;
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax;
	} else {
		xinc2 = 0;
		yinc1 = 0;
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay;
	}

	for (curpixel = 0; curpixel <= numpixels; curpixel++)
	{
		DrawLine(x, y, x3, y3);

		num += numadd;
		if (num >= den) {
			num -= den;
			x += xinc1;
			y += yinc1;
		}
		x += xinc2;
		y += yinc2;
	}
}

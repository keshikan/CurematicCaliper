/*
 *******************************************************************************
 *  [ssd1306.c]
 *  This module is making display pattern for SSD1306 OLED module (using I2C mode) .
 *
 *  This program is under the terms of the GPLv3.
 *  https://www.gnu.org/licenses/gpl-3.0.html
 *
 *  Copyright(c) 2019 Keshikan (www.keshikan.net)
 *******************************************************************************
 */

#include <ssd1306.h>
#include <stdio.h>
#include <stdlib.h>

#include "stm32l4xx_hal.h"


//porting
extern I2C_HandleTypeDef hi2c1;

uint8_t disp_memory[DISP_MEMORY_SIZE][2];
bool disp_transmitted=true;


//in 128x32,following setting should be needed.
//0xA0, // set remap  to //A0 or A1
//0xC0, // set com scan direction//C0 or C8
//0xDA, 0x02, // set com pin conf
//or
//0xA1, // set remap  to //A0 or A1
//0xC8, // set com scan direction//C0 or C8
//0xDA, 0x22, // set com pin conf
uint8_t disp_settings[] = {
		0x00,	//set command byte (following data is all for command.)
		0xA8, 0x3F,// set multiplex ratio
		0xD3, 0x00 , //set offset to 0
		0x40, // set start line to 0
		0xA0, // set remap  to //A0 or A1
		0x2E, //deactivate scroll
		0xC0, // set com scan direction//C0 or C8
		0xDA, 0x02, // set com pin conf
		0x81, 0x55, // set contrast
		0xA4, //set display on
		0xA6, // set normal
		0xD5, 0x80, // set freq
		0x8D, 0x14,//Enable charge pump
//		0x20, 0x00, // set display mode to horizonal addressing mode.
//		0x21, 0x00, 0x7F,//Set Column Address
//		0x22, 0x00, ((VRAM_Y_SIZE >> 3)-1),//Set Page Address 0x03: 128*32
		0xAF
};

void displayError()
{
	while(1);
}

void displayInit()
{

	//[a page structure]
	//disp memory is
	//0x80(command), 0x02,
	//0x80(command), 0x10,
	//0x80(command), 0xBn,
	//0xC0(data), dat
	//...


	//Control command
	for (uint32_t page=0; page<(VRAM_Y_SIZE >> 3); page++) {
		for (uint32_t j=0; j<3; j++) {
			disp_memory[page * 131 + j][0] = 0x80;//Continuation bit is 1, D/C is C (following byte is command.)
		}

		for (uint32_t j=3; j<131; j++) {//segment 0-127
			disp_memory[page * 131 + j][0] = 0xC0;//Continuation bit is 1, D/C is Data (following byte is data.)
		}
	}

	//Data byte
	for (uint32_t page=0; page<(VRAM_Y_SIZE >> 3); page++) {

		disp_memory[page * 131   ][1] = 0x02;//command: set lower column
		disp_memory[page * 131 +1][1] = 0x10;//command: set higher column
		disp_memory[page * 131 +2][1] = 0xB0 + page;//command: set page start address(page0-7)

		for (uint32_t j = 3; j < 131; j++) {
			disp_memory[page * 131 + j][1] = 0x00;
		}
	}

    disp_transmitted = true;


    //I2C version
  if(HAL_OK != HAL_I2C_Master_Transmit(&hi2c1, 0x78, disp_settings, sizeof(disp_settings), 1000) )
	  {
	  	  displayError();
	  }

  if(HAL_OK != HAL_I2C_Master_Transmit_IT(&hi2c1, 0x78, (uint8_t *)disp_memory, DISP_MEMORY_SIZE*2) )
	  {
	  	  displayError();
	  }

}



//x=0-127, y=0-64
//col:00-7F => black, 80-FF => white
void displayDrawDot(uint8_t x, uint8_t y, uint8_t col)
{

	uint8_t page, seg, dat;

	page = y >> 3;
	seg = x;
	dat = (0x01 << (y%8) );

	disp_memory[page * 131 + (seg+3)][1] = (disp_memory[page * 131 + (seg+3)][1] & ~dat ) + dat * (col >> 7);

	//Above code is equal to below code
	//
	//	if(col <= 0x7F){
	//		disp_memory[page * 131 + (column+3)][1] |= dat;
	//	}else{
	//		disp_memory[page * 131 + (column+3)][1] &= ~dat;
	//	}

}

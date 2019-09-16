/*
 *******************************************************************************
 *  [ssd1306.h]
 *  This module is making display pattern for SH1103 OLED module (using I2C mode) .
 *
 *  This program is under the terms of the GPLv3.
 *  https://www.gnu.org/licenses/gpl-3.0.html
 *
 *  Copyright(c) 2019 Keshikan (www.keshikan.net)
 *******************************************************************************
 */

#ifndef SSD1306_H_
#define SSD1306_H_

#include <stdbool.h>
#include <stdint.h>

//display memory
#define VRAM_X_SIZE (128)
#define VRAM_Y_SIZE (32)
#define DISP_MEMORY_SIZE ((VRAM_Y_SIZE >> 3) * 131)

//table
extern uint8_t disp_settings[];
extern uint8_t disp_memory[DISP_MEMORY_SIZE][2];
extern bool disp_transmitted;

//public func
extern void displayInit();
extern void displayDrawDot(uint8_t x, uint8_t y, uint8_t col);


#endif /* SSD1306_H_ */


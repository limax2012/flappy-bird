/*
 * oled.h
 *
 *  Created on: Jun 4, 2025
 *      Author: Max
 */

#ifndef INC_OLED_H_
#define INC_OLED_H_

#include "stm32f1xx_hal.h"

#define SSD1306_I2C_ADDR 0x3C
#define OLED_HEIGHT 128
#define OLED_WIDTH 64

void fb_clear(void);
void fb_fill_rectangle(int x, int y, int w, int h);
void fb_draw_floor(void);
void oled_init(I2C_HandleTypeDef *hi2c);
void oled_flush_fb(I2C_HandleTypeDef *hi2c);

#endif /* INC_OLED_H_ */

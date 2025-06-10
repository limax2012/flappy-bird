/*
 * oled.h
 *
 *  Created on: Jun 4, 2025
 *      Author: Max
 */

#ifndef INC_OLED_H_
#define INC_OLED_H_

#include <stdbool.h>

#include "stm32f1xx_hal.h"

#define SSD1306_I2C_ADDR 0x3C
#define OLED_H 128
#define OLED_W 64

void oled_init(I2C_HandleTypeDef *hi2c);
void fb_clear(void);
void fb_fill_rect(int x, int y, int w, int h, bool filled);
void fb_draw_floor(void);
void fb_draw_score(int score);
void oled_flush_fb(I2C_HandleTypeDef *hi2c);

#endif /* INC_OLED_H_ */

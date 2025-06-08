/*
 * oled.c
 *
 *  Created on: Jun 4, 2025
 *      Author: Max
 */

#include <string.h>
#include "oled.h"

static uint8_t fb[OLED_HEIGHT][OLED_WIDTH];

void oled_init(I2C_HandleTypeDef *hi2c) {
  uint8_t cmds[] = { 0x00,    // Control byte for commands
      0xAE,                   // Display OFF
      0xD5, 0x80,             // Set display clock divide ratio/osc frequency
      0xA8, 0x3F,             // Set multiplex ratio (height - 1)
      0xD3, 0x00,             // Set display offset
      0x40,                   // Set start line to 0
      0x8D, 0x14,             // Enable charge pump
      0x20, 0x00,             // Set memory addressing mode to horizontal
      0xA1,                   // Set segment re-map (mirror horizontally)
      0xC8,                   // Set COM output scan direction (remapped mode)
      0xDA, 0x12,             // Set COM pins hardware configuration
      0x81, 0x7F,             // Set contrast control
      0xD9, 0xF1,             // Set pre-charge period
      0xDB, 0x40,             // Set VCOMH deselect level
      0xA4,                   // Resume display from RAM content
      0xA6,                   // Set normal display (not inverted)
      0x00, 0xB0, 0x00, 0x10, // Set addressing start to page 0 col 0
      0xAF                    // Display ON
      };

  HAL_I2C_Master_Transmit(hi2c, SSD1306_I2C_ADDR << 1, cmds, sizeof(cmds), 100);
  HAL_Delay(100);

  uint8_t set_start_addr[] = { 0x00, 0xB0, 0x00, 0x10 };
  HAL_I2C_Master_Transmit(hi2c, SSD1306_I2C_ADDR << 1, set_start_addr,
      sizeof(set_start_addr), 100);
}

void fb_clear(void) {
  memset(fb, 0, sizeof(fb));
}

void fb_fill_rectangle(int x, int y, int w, int h) {
  for (int fb_y = y; fb_y < y + h; fb_y++) {
    for (int fb_x = x; fb_x < x + w; fb_x++) {
      if (fb_y >= 0 && fb_y < OLED_HEIGHT && fb_x >= 0 && fb_x < OLED_WIDTH) {
        fb[fb_y][fb_x] = 1;
      }
    }
  }
}

void fb_draw_floor(void) {
  fb_fill_rectangle(0, OLED_HEIGHT - 1, OLED_WIDTH, 1);
}

void oled_flush_fb(I2C_HandleTypeDef *hi2c) {
  // uint32_t start = HAL_GetTick();

  uint8_t flush_data[1 + (OLED_WIDTH / 8) * OLED_HEIGHT] = { 0 };
  flush_data[0] = 0x40;
  for (int fb_y = 0; fb_y < OLED_HEIGHT; fb_y++) {
    for (int fb_x = 0; fb_x < OLED_WIDTH; fb_x++) {
      int mirrored_fb_x = OLED_WIDTH - 1 - fb_x;
      int oled_byte_index = 1 + ((mirrored_fb_x / 8) * OLED_HEIGHT) + fb_y;
      flush_data[oled_byte_index] |= fb[fb_y][fb_x] << (mirrored_fb_x % 8);
    }
  }

  // uint32_t before_i2c = HAL_GetTick() - start;

  HAL_I2C_Master_Transmit(hi2c, SSD1306_I2C_ADDR << 1, flush_data,
      sizeof(flush_data), 100);

  // uint32_t after_i2c = HAL_GetTick() - start;
  // uint32_t i2c_time = after_i2c - before_i2c;
}

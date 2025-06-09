/*
 * bird.c
 *
 *  Created on: Jun 4, 2025
 *      Author: Max
 */

#include "bird.h"
#include "oled.h"

#define MAX_POS_Y OLED_HEIGHT - BIRD_H

static float pos_y = BIRD_START_POS_Y;
static float vel_y = BIRD_JUMP_VEL_Y;

float bird_get_y(void) {
  return pos_y;
}

void bird_update(void) {
  vel_y += BIRD_ACCEL_Y;
  pos_y += vel_y;
  if (pos_y > MAX_POS_Y) {
    pos_y = MAX_POS_Y;
  }
}

void bird_reset_pos(void) {
  pos_y = BIRD_START_POS_Y;
}

void bird_reset_vel(void) {
  vel_y = BIRD_JUMP_VEL_Y;
}

void bird_draw(void) {
  fb_fill_rectangle(BIRD_POS_X, (int) pos_y, BIRD_W, BIRD_H);
}

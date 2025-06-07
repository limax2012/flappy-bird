/*
 * bird.h
 *
 *  Created on: Jun 4, 2025
 *      Author: Max
 */

#ifndef INC_BIRD_H_
#define INC_BIRD_H_

#define BIRD_W 4
#define BIRD_H 4
#define BIRD_POS_X 12
#define BIRD_START_POS_Y 64.0f
#define BIRD_JUMP_VEL_Y -1.3f
#define BIRD_ACCEL_Y 0.04f

void bird_update_pos(void);
void bird_reset_vel(void);
void bird_draw(void);

#endif /* INC_BIRD_H_ */

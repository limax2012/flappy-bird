/*
 * pipe_queue.h
 *
 *  Created on: May 30, 2025
 *      Author: Max
 */

#ifndef SRC_PIPE_QUEUE_H_
#define SRC_PIPE_QUEUE_H_

#include <stdbool.h>

#define PIPE_W 6
#define PIPE_SEPARATION 16
#define PIPE_GAP_SIZE 40
#define PIPE_SPEED 0.15f
#define PIPE_MIN_Y 28
#define PIPE_MAX_Y 68

void pq_init(void);
void pq_enqueue();
void pq_update(void);
void pq_clear(void);
void pq_draw(void);
bool pq_collision(int bird_x, float bird_y, int bird_w, int bird_h);
bool pq_scored(int bird_x);

#endif /* SRC_PIPE_QUEUE_H_ */

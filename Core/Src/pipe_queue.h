/*
 * pipe.h
 *
 *  Created on: May 30, 2025
 *      Author: Max
 */

#ifndef SRC_PIPE_QUEUE_H_
#define SRC_PIPE_QUEUE_H_

#define SCREEN_WIDTH 64
#define PIPE_WIDTH 6
#define PIPE_SEPARATION 16
#define PIPE_OPENING_SIZE 42
#define PIPE_SPEED 0.08f
#define PIPE_HIGHEST_Y 28
#define PIPE_LOWEST_Y 68
#define PIPE_PERSIST_OFFSCREEN_PIXELS PIPE_WIDTH

typedef void (*PipeRenderFunc)(float prev_x, float x, float gap_y);

void pq_init(void);
void pq_enqueue();
void pq_update(void);
void pq_clear(void);
void oled_render_pq(PipeRenderFunc render);

#endif /* SRC_PIPE_QUEUE_H_ */

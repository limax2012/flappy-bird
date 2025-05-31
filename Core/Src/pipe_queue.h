/*
 * pipe.h
 *
 *  Created on: May 30, 2025
 *      Author: Max
 */

#ifndef SRC_PIPE_QUEUE_H_
#define SRC_PIPE_QUEUE_H_

typedef void (*PipeRenderFunc)(float x, float gap_y);

void pq_init(void);
void pq_enqueue(float gap_y);
void pq_update(void);
void pq_render_all(PipeRenderFunc render);
void pq_clear(void);

#endif /* SRC_PIPE_QUEUE_H_ */

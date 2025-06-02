/*
 * pipe.c
 *
 *  Created on: May 30, 2025
 *      Author: Max
 */

#include <stdlib.h>
#include <time.h>
#include "pipe_queue.h"

typedef struct Pipe {
  int last_rendered_x;
  float x;
  float gap_y;
  struct Pipe *next;
} Pipe;

typedef struct {
  Pipe *front;
  Pipe *rear;
} PipeQueue;

static PipeQueue pq;

void pq_init(void) {
  srand((unsigned int) time(NULL));

  pq.front = NULL;
  pq.rear = NULL;
}

void pq_enqueue() {
  Pipe *new_pipe = malloc(sizeof(Pipe));
  if (!new_pipe)
    return;

  float gap_y = ((float) rand() / RAND_MAX)
      * (PIPE_LOWEST_Y - PIPE_HIGHEST_Y)+ PIPE_HIGHEST_Y;

  new_pipe->last_rendered_x = -100;
  new_pipe->x = (float) -PIPE_WIDTH;
  new_pipe->gap_y = gap_y;
  new_pipe->next = NULL;

  if (pq.rear) {
    pq.rear->next = new_pipe;
  } else {
    pq.front = new_pipe;
  }

  pq.rear = new_pipe;
}

static void pq_dequeue(void) {
  if (!pq.front)
    return;

  Pipe *tmp = pq.front;
  pq.front = pq.front->next;
  if (!pq.front) {
    pq.rear = NULL;
  }

  free(tmp);
}

void pq_update(void) {
  Pipe *current_pipe = pq.front;

  while (current_pipe) {
    current_pipe->x += PIPE_SPEED;
    current_pipe = current_pipe->next;
  }

  if (pq.rear && pq.rear->x > PIPE_WIDTH + PIPE_SEPARATION) {
    pq_enqueue();
  }

  while (pq.front
      && (pq.front->x > SCREEN_WIDTH + PIPE_PERSIST_OFFSCREEN_PIXELS)) {
    pq_dequeue();
  }
}

void pq_clear(void) {
  while (pq.front) {
    pq_dequeue();
  }
}

void oled_render_pq(PipeRenderFunc render) {
  Pipe *current_pipe = pq.front;

  while (current_pipe) {
    render(current_pipe->last_rendered_x, current_pipe->x, current_pipe->gap_y);
    current_pipe->last_rendered_x = current_pipe->x;
    current_pipe = current_pipe->next;
  }
}

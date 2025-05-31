/*
 * pipe.c
 *
 *  Created on: May 30, 2025
 *      Author: Max
 */

#include <stdlib.h>
#include "pipe_queue.h"

#define SCREEN_WIDTH 128

static const int PIPE_WIDTH = 6;
static const float PIPE_SPEED = 0.1f;

typedef struct Pipe {
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
  pq.front = NULL;
  pq.rear = NULL;
}

void pq_enqueue(float gap_y) {
  Pipe *new_pipe = malloc(sizeof(Pipe));
  if (!new_pipe)
    return;

  new_pipe->x = SCREEN_WIDTH;
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
    current_pipe->x -= PIPE_SPEED;
    current_pipe = current_pipe->next;
  }

  while (pq.front && (pq.front->x + PIPE_WIDTH < 0)) {
    pq_dequeue();
  }
}

void pq_render_all(PipeRenderFunc render) {
  Pipe *current_pipe = pq.front;

  while (current_pipe) {
    render(current_pipe->x, current_pipe->gap_y);
    current_pipe = current_pipe->next;
  }
}

void pq_clear(void) {
  while (pq.front) {
    pq_dequeue();
  }
}

/*
 * pipe_queue.c
 *
 *  Created on: May 30, 2025
 *      Author: Max
 */

#include <stdbool.h>
#include <stdlib.h>
#include <time.h>

#include "oled.h"
#include "pipe_queue.h"

typedef struct Pipe {
  float x;
  float gap_top_y;
  bool scored;
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
  if (!new_pipe) {
    return;
  }

  float gap_top_y = PIPE_MIN_Y
      + (float) rand() / RAND_MAX * (PIPE_MAX_Y - PIPE_MIN_Y);

  new_pipe->x = (float) OLED_W;
  new_pipe->gap_top_y = gap_top_y;
  new_pipe->scored = false;
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

  if (pq.rear && (OLED_W - pq.rear->x > PIPE_W + PIPE_SEPARATION)) {
    pq_enqueue();
  }

  while (pq.front && (pq.front->x < -PIPE_W)) {
    pq_dequeue();
  }
}

void pq_clear(void) {
  while (pq.front) {
    pq_dequeue();
  }
}

void pq_draw() {
  Pipe *current_pipe = pq.front;

  while (current_pipe) {
    int x = (int) current_pipe->x;
    int gap_top_y = (int) current_pipe->gap_top_y;
    int gap_bottom_y = gap_top_y + PIPE_GAP_SIZE;

    fb_fill_rect(x, 0, PIPE_W, gap_top_y, true);
    fb_fill_rect(x, gap_bottom_y, PIPE_W, OLED_H - gap_bottom_y, true);

    current_pipe = current_pipe->next;
  }
}

bool pq_collision(int bird_x, float bird_y, int bird_w, int bird_h) {
  Pipe *current_pipe = pq.front;

  while (current_pipe) {
    float pipe_x = current_pipe->x;
    if ((bird_x + bird_w > pipe_x) && (bird_x < pipe_x + PIPE_W)) {
      float pipe_top = current_pipe->gap_top_y;
      if ((bird_y < pipe_top) || (bird_y + bird_h > pipe_top + PIPE_GAP_SIZE)) {
        return true;
      }
      break;
    }

    current_pipe = current_pipe->next;
  }

  return false;
}

bool pq_scored(int bird_x) {
  Pipe *current_pipe = pq.front;

  while (current_pipe) {
    if (!current_pipe->scored && (current_pipe->x < bird_x)) {
      current_pipe->scored = true;
      return true;
    }

    current_pipe = current_pipe->next;
  }

  return false;
}

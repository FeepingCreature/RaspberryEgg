#ifndef RASPBERRYEGG_RINGBUFFER_H
#define RASPBERRYEGG_RINGBUFFER_H

#include <stdbool.h>
#include <stdlib.h>
#include <stdatomic.h>

#include "util.h"

struct task
{
  bool quit; // exit when this task is found
  coordinate from, to;
  float dt;
};

struct task_ring_buffer
{
  size_t reading; // index of read pointer
  size_t writing; // index of write pointer

  size_t length; // immutable
  struct task *data;
};

struct task_ring_buffer *ringbuffer_init(size_t length);

void ringbuffer_queue(struct task_ring_buffer *buffer, struct task task);

bool ringbuffer_peek(struct task_ring_buffer *buffer);

struct task ringbuffer_take(struct task_ring_buffer *buffer);

#endif

#ifndef RASPBERRYEGG_RINGBUFFER_H
#define RASPBERRYEGG_RINGBUFFER_H

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

struct task_ring_buffer *ringbuffer_init(size_t length)
{
  struct task_ring_buffer *res = malloc(sizeof(struct task_ring_buffer));
  *res = (struct task_ring_buffer) {
    .reading = 0,
    .writing = 0,
    .length = length,
    .data = malloc(sizeof(struct task) * length)
  };
  return res;
}

void ringbuffer_queue(struct task_ring_buffer *buffer, struct task task)
{
  while ((buffer->writing + 1) % buffer->length == buffer->reading % buffer->length)
  {
    nap(10);
  }
  buffer->data[buffer->writing] = task;
  atomic_thread_fence(memory_order_release);
  buffer->writing = (buffer->writing + 1) % buffer->length;
}

bool ringbuffer_peek(struct task_ring_buffer *buffer)
{
  atomic_thread_fence(memory_order_acquire);
  return buffer->reading != buffer->writing;
}

struct task ringbuffer_take(struct task_ring_buffer *buffer)
{
  atomic_thread_fence(memory_order_acquire);
  struct task task = buffer->data[buffer->reading];
  atomic_thread_fence(memory_order_release);
  buffer->reading = (buffer->reading + 1) % buffer->length;
  return task;
}

#endif

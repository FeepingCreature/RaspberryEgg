#include "ringbuffer.h"

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

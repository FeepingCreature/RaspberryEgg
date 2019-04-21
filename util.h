#ifndef RASPBERRYEGG_UTIL_H
#define RASPBERRYEGG_UTIL_H

// for clock_gettime
#define _GNU_SOURCE

#include <stdio.h>
#include <time.h>

typedef struct
{
  int step;
  float substep;
} unit;

typedef struct
{
  unit egg;
  float egg_speed;
  unit pen;
  float pen_speed;
  float servo;
} coordinate;

unit unit_rebalance(unit unit);

unit unit_add(unit unit, float f);

coordinate coord_advance(coordinate from, float egg, float pen, float servo);

float unitf(unit unit);

float unit_diff_f(unit from, unit to);

static double secs()
{
  struct timespec time;

  clock_gettime(CLOCK_MONOTONIC, &time);
  return time.tv_sec * 1.0 + time.tv_nsec / 1000000000.0;
}

static inline int min(int a, int b)
{
  return (a < b) ? a : b;
}

static inline void nap(int ms)
{
  struct timespec spec = { .tv_sec = 0, .tv_nsec = ms * 1000 * 1000 };
  nanosleep(&spec, NULL);
}

static inline void burn_cpu()
{
  printf("cpu burn in...\n");
  double start = secs();
  while (secs() - start < 1);
  printf("cpu burn in OK\n");
}

static inline void wait_for_return(char *msg)
{
  char *lineptr = NULL;
  size_t linelen = 0;
  printf(msg);
  getline(&lineptr, &linelen, stdin);
}

#endif

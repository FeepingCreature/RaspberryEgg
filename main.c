#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include "config.h"
#include "pi.h"
#include "ringbuffer.h"
#include "util.h"

struct pwm_config
{
  int length_pow2;
  float factor;
};

struct stepper_config
{
  int out1, out2; // in1, in2 (winding 1)
  int out3, out4; // in3, in4 (winding 2)
};

struct servo_config
{
  int out;
  float low, high;
  float pwm_low, pwm_high;
  float pwm_length; // in s
};

struct eggbot_config
{
  double cycles_per_s;
  struct pwm_config pwm_config;
  struct stepper_config egg_config;
  struct stepper_config pen_config;
  struct servo_config servo_config;
  bool dry_run; // don't move the servo
};

static double move_accel(double length, double dt, double start_speed)
{
  // d(t) = 1/2 a t^2 + v0 t
  // d(dt) = length = 1/2 a dt^2 + v0 dt
  // a = 2 length / dt^2 - 2 v0 / dt
  return 2 * (length / (dt * dt) - start_speed / dt);
}

static float end_speed(float length, float dt, float start_speed)
{
  return move_accel(length, dt, start_speed) * dt + start_speed;
}

static float blend(float t, float low, float high)
{
  return low + (high - low) * t;
}

static float servo_factor(struct servo_config servo_config, float f)
{
  return blend(
    blend(
      1.0f - f,
      servo_config.low, servo_config.high
    ),
    servo_config.pwm_low, servo_config.pwm_high
  );
}

static pthread_t worker_id;
static bool worker_abort = false;
static bool worker_aborted = false;

#ifdef LOG_SERVO_TIMINGS
static int servolog;
#endif

static uint64_t global_cycle_counter = 0;

static void step(struct eggbot_config *config, coordinate from, coordinate to, float dt)
{
  if (dt < 0)
  {
    fprintf(stderr, "Time travel detected!\n");
    abort();
  }

  float distance_egg = unit_diff_f(from.egg, to.egg);
  double egg_accel = move_accel(distance_egg, dt, from.egg_speed);
  // transform into unit "dist_unit / dt^2"
  double egg_accel_unit = egg_accel * dt * dt;
  // transform into unit "dist_unit / dt"
  double egg_v0_unit = from.egg_speed * dt;

  float distance_pen = unit_diff_f(from.pen, to.pen);
  double pen_accel = move_accel(distance_pen, dt, from.pen_speed);
  // transform into unit "dist_unit / dt^2"
  double pen_accel_unit = pen_accel * dt * dt;
  // transform into unit "dist_unit / dt"
  double pen_v0_unit = from.pen_speed * dt;

  int cycles = (int) (dt * config->cycles_per_s);
  /*printf(
    "dt %f, degg %f, dpen %f - %i cycles\n",
    dt, distance_egg, distance_pen, cycles
  );*/

  volatile uint32_t *set_reg = gpio_port + (GPIO_SET_OFFSET / sizeof(uint32_t));
  volatile uint32_t *clr_reg = gpio_port + (GPIO_CLR_OFFSET / sizeof(uint32_t));

  const float TWOPI = M_PI * 2.0;

  uint32_t last_bits = 0;

  for (int i = 0; i < cycles && !worker_abort; /* i is incremented by the k loop below */)
  {
    if (i % 1024 == 0)
    {
      atomic_thread_fence(memory_order_acquire);
    }
    double t = (double) i / (double) cycles; // unit "distance"
    double t_global = secs(); // unit s
    double egg_angle = from.egg.substep + (egg_accel_unit / 2.0) * t * t + egg_v0_unit * t;
    double pen_angle = from.pen.substep + (pen_accel_unit / 2.0) * t * t + pen_v0_unit * t;
    // printf("%f; angle = %f vs %f; %f and v0 %f\n", t, angle - from.egg.substep, distance_egg, accel_unit, v0_unit);
    float egg_angle_substep = TWOPI * (egg_angle - floor(egg_angle));
    float pen_angle_substep = TWOPI * (pen_angle - floor(pen_angle));

    float egg_sin = sinf(egg_angle_substep), egg_cos = cosf(egg_angle_substep);
    float pen_sin = sinf(pen_angle_substep), pen_cos = cosf(pen_angle_substep);

    const float exp = 1.0;

    float egg_winding1 = copysignf(powf(fabsf(egg_sin), exp), egg_sin);
    float egg_winding2 = copysignf(powf(fabsf(egg_cos), exp), egg_cos);

    float pen_winding1 = copysignf(powf(fabsf(pen_sin), exp), pen_sin);
    float pen_winding2 = copysignf(powf(fabsf(pen_cos), exp), pen_cos);

    float servo_f = servo_factor(config->servo_config, blend(t, from.servo, to.servo));
    float pwm_length = config->servo_config.pwm_length;
    float servo_t = ((t_global / pwm_length) - floor(t_global / pwm_length)) * pwm_length; // in s
    float servo_lowf = servo_f * pwm_length - servo_t; // in s
    float servo_highf = pwm_length - servo_t; // in s
    int servo_to_low = (int)(servo_lowf * config->cycles_per_s);
    int servo_to_high = (int)(servo_highf * config->cycles_per_s);

    bool egg_out1 = egg_winding1 > 0, egg_out2 = egg_winding1 < 0;
    bool egg_out3 = egg_winding2 > 0, egg_out4 = egg_winding2 < 0;

    bool pen_out1 = pen_winding1 > 0, pen_out2 = pen_winding1 < 0;
    bool pen_out3 = pen_winding2 > 0, pen_out4 = pen_winding2 < 0;

    struct pwm_config pwm_config = config->pwm_config;
    struct stepper_config egg_config = config->egg_config;
    struct stepper_config pen_config = config->pen_config;
    struct servo_config servo_config = config->servo_config;

    int pwm_limit_egg_winding1 = (int) (pwm_config.factor * fabsf(egg_winding1) * pwm_config.length_pow2);
    int pwm_limit_egg_winding2 = (int) (pwm_config.factor * fabsf(egg_winding2) * pwm_config.length_pow2);

    int pwm_limit_pen_winding1 = (int) (pwm_config.factor * fabsf(pen_winding1) * pwm_config.length_pow2);
    int pwm_limit_pen_winding2 = (int) (pwm_config.factor * fabsf(pen_winding2) * pwm_config.length_pow2);

    uint32_t bits_egg_winding1 = (egg_out1 << egg_config.out1) | (egg_out2 << egg_config.out2);
    uint32_t bits_egg_winding2 = (egg_out3 << egg_config.out3) | (egg_out4 << egg_config.out4);
    uint32_t bits_pen_winding1 = (pen_out1 << pen_config.out1) | (pen_out2 << pen_config.out2);
    uint32_t bits_pen_winding2 = (pen_out3 << pen_config.out3) | (pen_out4 << pen_config.out4);
    uint32_t bit_servo = (1 << servo_config.out);

    int pwm_len = min(config->pwm_config.length_pow2, cycles - i);
    for (int k = 0; k < pwm_len; k++)
    {
      uint32_t pwm_egg_winding1 = (k < pwm_limit_egg_winding1) ? bits_egg_winding1 : 0;
      uint32_t pwm_egg_winding2 = (k < pwm_limit_egg_winding2) ? bits_egg_winding2 : 0;
      uint32_t pwm_pen_winding1 = (k < pwm_limit_pen_winding1) ? bits_pen_winding1 : 0;
      uint32_t pwm_pen_winding2 = (k < pwm_limit_pen_winding2) ? bits_pen_winding2 : 0;
      uint32_t servo_bits = ((k < servo_to_low) ? bit_servo : 0) | ((k >= servo_to_high) ? bit_servo : 0);
      uint32_t pwm_bits = pwm_egg_winding1 | pwm_egg_winding2 | pwm_pen_winding1 | pwm_pen_winding2;
      uint32_t bits = pwm_bits | servo_bits;

#ifdef LOG_SERVO_TIMINGS
      if ((bits & bit_servo) != (last_bits & bit_servo)) dprintf(servolog, "%f\t%i\n", secs(), !!(bits & bit_servo));
#endif

      uint32_t set =  bits & ~last_bits;
      uint32_t clr = ~bits &  last_bits;

      *clr_reg = clr;
      *set_reg = set;
      last_bits = bits;
    }
    i += pwm_len;
  }
  global_cycle_counter += cycles;
}

static void clear_all(int signum)
{
  printf("clear all...\n");
  if (worker_id != 0 && worker_id != pthread_self())
  {
    printf(" halting worker thread\n");
    // cancel worker thread to stop it from writing bits
    worker_abort = true;
    while (!worker_aborted) { nap(1); }
  }

  volatile uint32_t *clr_reg = gpio_port + (GPIO_CLR_OFFSET / sizeof(uint32_t));

  *clr_reg = gpio_reset_mask;

  printf("clear all OK\n");
  signal(signum, SIG_DFL);
  raise(signum);
}

static void setup_guards()
{
  signal(SIGINT, &clear_all);
  signal(SIGTERM, &clear_all);
  signal(SIGKILL, &clear_all);
  signal(SIGABRT, &clear_all);
}

static void initialize_gpios(struct eggbot_config *config)
{
  initialize_gpio_for_output(config->servo_config.out);
  initialize_gpio_for_output(config->egg_config.out1);
  initialize_gpio_for_output(config->egg_config.out2);
  initialize_gpio_for_output(config->egg_config.out3);
  initialize_gpio_for_output(config->egg_config.out4);
  initialize_gpio_for_output(config->pen_config.out1);
  initialize_gpio_for_output(config->pen_config.out2);
  initialize_gpio_for_output(config->pen_config.out3);
  initialize_gpio_for_output(config->pen_config.out4);
}

struct worker
{
  struct eggbot_config config;
  struct task_ring_buffer *queue;
};

static void coord_bound(coordinate *coordp)
{
  float penf = unitf(coordp->pen);
  int low = -5, high = 5;
  if (penf < low)
  {
    fprintf(stderr, "warn: attempt to set pen position out of bounds: %f\n", penf);
    coordp->pen = (unit) { .step = low };
  }
  if (penf > high)
  {
    fprintf(stderr, "warn: attempt to set pen position out of bounds: %f\n", penf);
    coordp->pen = (unit) { .step = high };
  }
}

static void queue_task(struct task_ring_buffer *buffer, coordinate from, coordinate to, float dt)
{
  coord_bound(&from);
  coord_bound(&to);

  ringbuffer_queue(buffer, (struct task) { .quit = false, .from = from, .to = to, .dt = dt });
}

static void queue_quit(struct task_ring_buffer *buffer)
{
  ringbuffer_queue(buffer, (struct task) { .quit = true });
}

// set speed in `nextp` to match the acceleration computed by `step()` so that next continues smoothly.
static void set_finishing_speed(coordinate from, coordinate *nextp, float dt)
{
  float distance_egg = unit_diff_f(from.egg, nextp->egg);
  nextp->egg_speed = end_speed(distance_egg, dt, from.egg_speed);

  float distance_pen = unit_diff_f(from.pen, nextp->pen);
  nextp->pen_speed = end_speed(distance_pen, dt, from.pen_speed);
}

static void stepper_advance(struct worker *worker, coordinate *coordp, float dt, float egg, float pen, float servo)
{
  coordinate next = coord_advance(*coordp, egg, pen, servo);

  queue_task(worker->queue, *coordp, next, dt);
  set_finishing_speed(*coordp, &next, dt);
  *coordp = next;
}

static void *worker_task(void *data)
{
  struct worker *worker = (struct worker*) data;
  cpu_set_t cpuset;

  CPU_ZERO(&cpuset);
  CPU_SET(3, &cpuset);
  int res = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
  if (res != 0)
  {
    fprintf(stderr, "pthread_setaffinity_np() failed: %i, %i\n", res, errno);
    abort();
  }

  coordinate last = {{ 0 }};
  while (!worker_abort)
  {
    if (ringbuffer_peek(worker->queue))
    {
      struct task task = ringbuffer_take(worker->queue);

      if (task.quit) break;

      step(&worker->config, task.from, task.to, task.dt);
      last = task.to;
    }
    else
    {
      fprintf(stderr, "warn: ring buffer underrun, idling\n");
      while (!ringbuffer_peek(worker->queue) && !worker_abort)
      {
        coordinate next = last;
        step(&worker->config, last, next, 0.1f);
      }
    }
  }
  worker_aborted = true;
  return NULL;
}

static pthread_t start_worker(struct worker *worker)
{
  pthread_t worker_thread;
  int res = pthread_create(&worker_thread, NULL, worker_task, worker);

  if (res != 0)
  {
    fprintf(stderr, "pthread_create() failed: %i, %i\n", res, errno);
    abort();
  }
  return worker_thread;
}

// set speed at `*fromp` so as to move towards `to` with instant acceleration
static void set_instant_speed(coordinate *fromp, coordinate to, float dt)
{
  float eggmove = unit_diff_f(fromp->egg, to.egg);
  float penmove = unit_diff_f(fromp->pen, to.pen);

  fromp->egg_speed = eggmove / dt;
  fromp->pen_speed = penmove / dt;
}

static void process_eggcode_file(struct worker *worker, coordinate *pos, const char *filename)
{
  const float speedscale = 1.0;
  FILE *cmd_file = fopen(filename, "r");
  char *line_ptr = NULL;
  size_t line_len0 = 0;
  while (true)
  {
    ssize_t res = getline(&line_ptr, &line_len0, cmd_file);
    if (res == -1)
    {
      fclose(cmd_file);
      fprintf(stderr, "cannot read line, file processing complete.\n");
      break;
    }
    size_t line_len = line_len0 - 1;

    if (strncmp(line_ptr, "SP,", min(3, line_len)) == 0)
    {
      int dt_ms;
      int penstate; // 0 = down, 1 = up
      res = sscanf(line_ptr, "SP,%i,%i", &penstate, &dt_ms);
      if (res != 2)
      {
        fprintf(stderr, "invalid SP command %s\n", line_ptr);
        abort();
      }
      pos->egg_speed = 0;
      pos->pen_speed = 0;
      stepper_advance(worker, pos, dt_ms * speedscale / 1000.0f, 0.0, 0.0, penstate);
    }
    else if (strncmp(line_ptr, "SM,", min(3, line_len)) == 0)
    {
      int dt_ms;
      int degg;
      int dpen;
      res = sscanf(line_ptr, "SM,%i,%i,%i", &dt_ms, &dpen, &degg);
      if (res != 3)
      {
        fprintf(stderr, "invalid SM command %s\n", line_ptr);
        abort();
      }
      float eggmove = degg / 64.0f;
      float penmove = dpen / 90.0f;
      float dt = dt_ms * speedscale / 1000.0f;
      coordinate next = coord_advance(*pos, eggmove, penmove, pos->servo);
      set_instant_speed(pos, next, dt);
      stepper_advance(worker, pos, dt, eggmove, penmove, pos->servo);
    }
    else
    {
      printf("# unknown command %s\n", line_ptr);
    }
  }
}

int main(int argc, const char **argv)
{
#ifdef LOG_SERVO_TIMINGS
  servolog = creat("/tmp/servolog.txt", 0666);
#endif

  gpio_port = mmap_bcm_register(GPIO_REGISTER_BASE);
  setup_guards();

  burn_cpu();

  struct servo_config servo_config = {
    .out = SERVO_PIN,
    .low = SERVO_LOW,
    .high = SERVO_HIGH,
    .pwm_length = SERVO_PWM_LENGTH,
    .pwm_low = SERVO_PWM_LOW,
    .pwm_high = SERVO_PWM_HIGH,
  };

  struct stepper_config egg_config = {
    .out1 = STEPPER_EGG_PIN1,
    .out2 = STEPPER_EGG_PIN2,
    .out3 = STEPPER_EGG_PIN3,
    .out4 = STEPPER_EGG_PIN4,
  };

  struct stepper_config pen_config = {
    .out1 = STEPPER_PEN_PIN1,
    .out2 = STEPPER_PEN_PIN2,
    .out3 = STEPPER_PEN_PIN3,
    .out4 = STEPPER_PEN_PIN4,
  };

  struct eggbot_config calibrate_config = {
    .cycles_per_s = CALIBRATION_CYCLES, // and run for "1s"
    .dry_run = true,
    .pwm_config = {
      .length_pow2 = 2048,
      .factor = 1.0 / 512.0,
    },
    .egg_config = egg_config,
    .pen_config = pen_config,
    .servo_config = servo_config,
  };

  initialize_gpios(&calibrate_config);

  coordinate calibrateFrom = {{ 0 }};
  coordinate calibrateTo = {{ 0 }};
  double start = secs();
  printf("calibrate stepper loop...\n");
  step(&calibrate_config, calibrateFrom, calibrateTo, 1.0);
  double end = secs();
  printf("calibrate stepper loop OK\n");
  printf("%f seconds for %i stepper control cycles\n", end - start, CALIBRATION_CYCLES);
  double cycles_per_s = CALIBRATION_CYCLES / (end - start);
  double us_per_cycle = ((end - start) * 1000000.0) / CALIBRATION_CYCLES;
  int cycles_per_pwm = next_pow2((int)(US_PER_PWM / us_per_cycle));
  printf(
    "%f us/cycle; %f cycles/s, %i cycles/pwm\n",
    us_per_cycle, cycles_per_s, cycles_per_pwm
  );

  struct eggbot_config config = {
    .cycles_per_s = cycles_per_s,
    .pwm_config = {
      .length_pow2 = cycles_per_pwm,
      .factor = BASE_PWM_FACTOR,
      // .boost = 16, // TODO per-coil pwm
    },
    .egg_config = egg_config,
    .pen_config = pen_config,
    .servo_config = servo_config
  };

  struct worker worker_thread = {
    .config = config,
    .queue = ringbuffer_init(16),
  };

  printf("start worker\n");
  pthread_t worker = start_worker(&worker_thread);
  worker_id = worker; // so the signal handler can cancel it

  coordinate origin = {{ 0 }};
  // origin.pen = unit_add(origin.pen, 0.5);
  origin.servo = 1.0; // up

  coordinate coord = origin;
  // raise servo if it's low
  stepper_advance(&worker_thread, &coord, 0.5, 0.0, 0.0, 1.0);

  // stepper_advance(&worker_thread, &coord, 2.0, 0.0, -4.0, 1.0);

  for (int i = 1; i < argc; i++)
  {
    printf("next: '%s'\n", argv[i]);
    wait_for_return("Please insert the next pen and press return to continue.");
    printf("printing...\n");
    process_eggcode_file(&worker_thread, &coord, argv[i]);
    printf("printing OK\n");
    // better use eggbot exporter "always home" feature for this.
    /*printf("homing.\n");
    {
      // approx distance
      float dist = fabsf(unitf(coord.egg)) + fabsf(unitf(coord.pen));
      float dt = dist / 5.0;

      set_instant_speed(&coord, origin, dt);
      queue_task(worker_thread.queue, coord, origin, dt);
      coord = origin;
    }*/
  }

  queue_quit(worker_thread.queue);
  pthread_join(worker, NULL);
  worker_id = 0;
  clear_all(0);
  return 0;
}

#ifndef RASPBERRYPI_PI_H
#define RASPBERRYPI_PI_H

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>

// thanks rpi-gpio-dma-demo

#ifndef PI_VERSION
#  define PI_VERSION 2
#endif

#define PAGE_SIZE 4096

#define BCM2708_PI1_PERI_BASE  0x20000000
#define BCM2709_PI2_PERI_BASE  0x3F000000

#if PI_VERSION == 2
#  define PERI_BASE BCM2709_PI2_PERI_BASE
#else
#  define PERI_BASE BCM2708_PI1_PERI_BASE
#endif

// ---- GPIO specific defines
#define GPIO_REGISTER_BASE 0x200000
#define GPIO_SET_OFFSET 0x1C
#define GPIO_CLR_OFFSET 0x28
#define PHYSICAL_GPIO_BUS (0x7E000000 + GPIO_REGISTER_BASE)

extern volatile uint32_t *gpio_port;
extern uint32_t gpio_reset_mask;

void *mmap_bcm_register(off_t register_offset);

void initialize_gpio_for_output(int bit);

#endif

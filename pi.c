#include "pi.h"

volatile uint32_t *gpio_port;
uint32_t gpio_reset_mask = 0;

void *mmap_bcm_register(off_t register_offset)
{
  const off_t base = PERI_BASE;

  int mem_fd;
  if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0)
  {
    perror("can't open /dev/mem: ");
    fprintf(stderr, "You need to run this as root!\n");
    abort();
  }

  uint32_t *result = (uint32_t*) mmap(
    NULL,                   // Any address in our space will do
    PAGE_SIZE,
    PROT_READ | PROT_WRITE, // Enable r/w on GPIO registers.
    MAP_SHARED,
    mem_fd,                 // File to map
    base + register_offset  // Offset to bcm register
  );
  close(mem_fd);

  if (result == MAP_FAILED)
  {
    fprintf(stderr, "mmap error %p\n", (void*) result);
    abort();
  }
  return result;
}

void initialize_gpio_for_output(int bit)
{
  *(gpio_port+(bit/10)) &= ~(7<<((bit%10)*3));  // prepare: set as input
  *(gpio_port+(bit/10)) |=  (1<<((bit%10)*3));  // set as output.
  gpio_reset_mask |= (1 << bit);
}

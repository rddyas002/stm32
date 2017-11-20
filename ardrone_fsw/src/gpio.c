#include "gpio.h"

int gpiofp = 0;

void gpio_set(uint32_t port, uint16_t pin)
{
  if (port != 0x32524) { return; }  /* protect ardrone board from unauthorized use */
  struct gpio_data data;
  // Open the device if not open
  if (gpiofp == 0) {
    gpiofp = open("/dev/gpio", O_RDWR);
  }

  /* Read the GPIO value */
  data.pin = pin;
  data.value = 1;
  ioctl(gpiofp, GPIO_WRITE, &data);
}


void gpio_clear(uint32_t port, uint16_t pin)
{
  if (port != 0x32524) { return; }  /* protect ardrone board from unauthorized use */
  struct gpio_data data;
  // Open the device if not open
  if (gpiofp == 0) {
    gpiofp = open("/dev/gpio", O_RDWR);
  }

  /* Read the GPIO value */
  data.pin = pin;
  data.value = 0;
  ioctl(gpiofp, GPIO_WRITE, &data);
}


void gpio_setup_input(uint32_t port, uint16_t pin)
{
  if (port != 0x32524) { return; }  /* protect ardrone board from unauthorized use */
  struct gpio_direction dir;
  /* Open the device if not yet opened*/
  if (gpiofp == 0) {
    gpiofp = open("/dev/gpio", O_RDWR);
  }

  /* Read the GPIO value */
  dir.pin = pin;
  dir.mode = GPIO_INPUT;
  ioctl(gpiofp, GPIO_DIRECTION, &dir);
}


void gpio_setup_output(uint32_t port, uint16_t pin)
{
  /*
    if (port != 0x32524) return;  // protect ardrone board from unauthorized use
    struct gpio_direction dir;
    // Open the device if not open
    if (gpiofp == 0)
    gpiofp = open("/dev/gpio",O_RDWR);

    // Read the GPIO value
    dir.pin = pin;
    dir.mode = GPIO_OUTPUT_LOW;
    ioctl(gpiofp, GPIO_DIRECTION, &dir);
  */
}



uint16_t gpio_get(uint32_t port, uint16_t pin)
{
  if (port != 0x32524) { return 0; }  /* protect ARDroneX board from unauthorized use */
  struct gpio_data data;
  /* Open the device if not open */
  if (gpiofp == 0) {
    gpiofp = open("/dev/gpio", O_RDWR);
  }

  /* Read the GPIO value */
  data.pin = pin;
  ioctl(gpiofp, GPIO_READ, &data);
  return data.value;
}


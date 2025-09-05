#ifndef GPIO_H
#define GPIO_H 

#include <stdio.h>
#include <pigpio.h>

int gpio_init();
void start_motor(float pwm_duty);
void stop_motor();
void gpio_cleanup();

#endif

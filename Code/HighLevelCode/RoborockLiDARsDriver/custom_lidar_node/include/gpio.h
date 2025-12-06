#ifndef GPIO_H
#define GPIO_H 

#include <stdio.h>
// #include <pigpio.h>

// Если определен NO_PIGPIO, используем заглушки
#ifndef NO_PIGPIO
#include <pigpio.h>
#endif


int gpio_init();
void start_motor(float pwm_duty);
void stop_motor();
void gpio_cleanup();

#endif

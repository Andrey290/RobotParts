#include "lds01_driver/gpio.h"
#include <pigpiod_if2.h>
#include <stdio.h>

static int pi = -1;

int gpio_init(void) {
    pi = pigpio_start(NULL, NULL); // Подключение к локальному демону
    if (pi < 0) {
        fprintf(stderr, "Failed to connect to pigpio daemon\n");
        return -1;
    }
    return 0;
}

void start_motor(float pwm_duty) {
    if (pi < 0) return;
    
    const int motor_pin = 18;
    const unsigned int range = 1000;
    
    if (pwm_duty < 0.0f) pwm_duty = 0.0f;
    if (pwm_duty > 1.0f) pwm_duty = 1.0f;

    set_PWM_frequency(pi, motor_pin, 8000);
    set_PWM_range(pi, motor_pin, range);
    unsigned int duty = (unsigned int)(pwm_duty * (float)range);
    set_PWM_dutycycle(pi, motor_pin, duty);
}

void stop_motor(void) {
    if (pi < 0) return;
    set_PWM_dutycycle(pi, 18, 0);
}

void gpio_cleanup(void) {
    if (pi >= 0) {
        pigpio_stop(pi);
        pi = -1;
    }
}

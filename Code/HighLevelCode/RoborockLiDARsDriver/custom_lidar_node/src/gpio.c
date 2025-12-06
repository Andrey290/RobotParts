// src/gpio.c
#include "../include/gpio.h"
#include <stdio.h>
#include <stdlib.h>
#include <pigpiod_if2.h>

// handle pi для pigpiod_if2
static int pi = -1;

int gpio_init() {
    // подключаемся к локальному демону pigpiod (addr=NULL, port=NULL)
    pi = pigpio_start(NULL, NULL);
    if (pi < 0) {
        fprintf(stderr, "gpio_init: pigpio_start failed (pi=%d)\n", pi);
        return -1;
    }
    return 0;
}

void start_motor(float pwm_duty) {
    const unsigned motor_pin = 18;
    const unsigned freq = 8000;   // частота PWM по спецификации лидара
    const unsigned range = 1000;  // диапазон, удобный для duty в процентах: duty = range * fraction

    if (pi < 0) {
        fprintf(stderr, "start_motor: pigpio not initialised\n");
        return;
    }

    // назначаем пин как OUTPUT
    set_mode(pi, motor_pin, PI_OUTPUT);

    // задаём частоту и диапазон, затем duty
    set_PWM_frequency(pi, motor_pin, freq);
    set_PWM_range(pi, motor_pin, range);

    unsigned duty = (unsigned)(pwm_duty * (float)range); // pwm_duty 0.0..1.0
    set_PWM_dutycycle(pi, motor_pin, duty);
}

void stop_motor() {
    if (pi >= 0) {
        set_PWM_dutycycle(pi, 18, 0);
    }
}

void gpio_cleanup() {
    if (pi >= 0) {
        pigpio_stop(pi);
        pi = -1;
    }
}


#include "../inc/gpio.h"

int gpio_init() {
    if (gpioInitialise() < 0) {
        fprintf(stderr, "Failed to initialize pigpio\n");
        return -1;
    }
    return 0;
}

void start_motor(float pwm_duty) {
    const int motor_pin = 18;
    const unsigned int range = 1000; // finer granularity; matches python tests
    // clamp duty into [0..1]
    if (pwm_duty < 0.0f) pwm_duty = 0.0f;
    if (pwm_duty > 1.0f) pwm_duty = 1.0f;

    gpioSetPWMfrequency(motor_pin, 8000); // frequency by lidar spec
    gpioSetPWMrange(motor_pin, range);
    unsigned int duty = (unsigned int)(pwm_duty * (float)range);
    gpioPWM(motor_pin, duty);
}

void stop_motor() {
    const int motor_pin = 18;
    gpioPWM(motor_pin, 0);
}

void gpio_cleanup() {
    gpioTerminate();
}


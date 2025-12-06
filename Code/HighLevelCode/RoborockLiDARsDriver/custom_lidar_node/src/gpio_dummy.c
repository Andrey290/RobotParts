#include <stdio.h>

int gpio_init() { 
    printf("[GPIO DUMMY] Initialized\n"); 
    return 0; 
}

void start_motor(float pwm_duty) { 
    printf("[GPIO DUMMY] Motor started with duty: %.2f\n", pwm_duty); 
}

void stop_motor() { 
    printf("[GPIO DUMMY] Motor stopped\n"); 
}

void gpio_cleanup() { 
    printf("[GPIO DUMMY] Cleanup\n"); 
}
#ifndef LDS01_DRIVER_GPIO_H
#define LDS01_DRIVER_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

int gpio_init(void);
void start_motor(float pwm_duty);
void stop_motor(void);
void gpio_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif // LDS01_DRIVER_GPIO_H

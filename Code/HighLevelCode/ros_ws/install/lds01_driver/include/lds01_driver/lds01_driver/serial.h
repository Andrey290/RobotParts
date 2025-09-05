#ifndef LDS01_DRIVER_SERIAL_H
#define LDS01_DRIVER_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <termios.h>
#include <stdint.h>
#include <stddef.h> 

int open_serial(const char *device, speed_t baud_rate);
int read_serial(int fd, uint8_t *buffer, size_t size);
int close_serial(int fd);

#ifdef __cplusplus
}
#endif

#endif

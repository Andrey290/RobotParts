#ifndef SERIAL_H
#define SERIAL_H

#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

int open_serial(const char *device, speed_t baud_rate);
int close_serial(int fd);

#endif

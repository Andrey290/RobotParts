#include "lds01_driver/serial.h"
#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>

int open_serial(const char *device, speed_t baud_rate) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("open_serial: unable to open device");
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("open_serial: tcgetattr");
        close(fd);
        return -1;
    }

    // raw mode
    cfmakeraw(&tty);
    cfsetospeed(&tty, baud_rate);
    cfsetispeed(&tty, baud_rate);

    // set baud rates
    if (cfsetispeed(&tty, B115200) != 0 || cfsetospeed(&tty, B115200) != 0) {
        perror("open_serial: cfsetispeed/cfsetospeed");
        close(fd);
        return -1;
    }

    // 8N1, no parity, one stop bit
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    // disable hardware flow control
    tty.c_cflag &= ~CRTSCTS;

    // disable software flow control
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    // enable receiver, local mode
    tty.c_cflag |= (CLOCAL | CREAD);

    // non-canonical, no echo, no signals
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // output processing off
    tty.c_oflag &= ~OPOST;

    // Set read timeout: VMIN = 0, VTIME = 1 => read() returns
    // as soon as any data is available, or after 0.1s timeout.
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1; // tenths of seconds

    // flush and apply
    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("open_serial: tcsetattr");
        close(fd);
        return -1;
    }

    // ensure blocking mode (so poll + read interplay is predictable)
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) flags = 0;
    flags &= ~O_NONBLOCK;
    fcntl(fd, F_SETFL, flags);

    return fd;
}

int read_serial(int fd, uint8_t *buffer, size_t size) {
    ssize_t bytes_read = read(fd, buffer, size);
    if (bytes_read < 0) {
        perror("read_serial error");
        return -1;
    }
    return bytes_read;
}

int close_serial(int fd) {
    if (fd >= 0) {
        return close(fd);
    }
    return -1;
}


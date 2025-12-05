#include "../include/serial.h"

#include <termios.h>
#include <sys/ioctl.h>

int open_serial(const char *device, speed_t baud_rate) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("open_serial: unable to open device");
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    // input flags - clear parity checking, strip, etc.
    tty.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | ISTRIP | IXON | IXOFF | IXANY);
    // output flags
    tty.c_oflag &= ~OPOST;
    // control flags - 8N1, no parity
    tty.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    tty.c_cflag |= CS8 | CLOCAL | CREAD;
    // local flags - raw input
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    // Set baud rate
    cfsetispeed(&tty, baud_rate);
    cfsetospeed(&tty, baud_rate);

    // read() will return as soon as at least 1 byte is available, or after timeout
    tty.c_cc[VMIN]  = 0;    // return immediately with what is available
    tty.c_cc[VTIME] = 1;    // timeout in deciseconds (0.1s)

    // apply
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    // flush input/output
    tcflush(fd, TCIOFLUSH);

    return fd;
}

int close_serial(int fd) {
    return close(fd);
}


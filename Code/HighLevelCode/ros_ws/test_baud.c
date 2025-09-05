#include <stdio.h>
#include <termios.h>

int main() {
    speed_t speeds[] = {B9600, B19200, B38400, B57600, B115200, B230400};
    const char *names[] = {"9600", "19200", "38400", "57600", "115200", "230400"};
    
    for (int i = 0; i < 6; i++) {
        struct termios tty;
        printf("Testing %s: ", names[i]);
        if (cfsetispeed(&tty, speeds[i]) == 0) {
            printf("OK\n");
        } else {
            printf("FAIL\n");
        }
    }
    return 0;
}

// LIBRARIES
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <poll.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

// HEADERS
#include "parser.h"
#include "gpio.h"
#include "serial.h"

// GLOBALS
volatile sig_atomic_t running = 1;
lidar_parser_t parser;

// PROTOTYPES
void siginit_handler(int sig);

int main() {
    // Ctrl+C ending
    signal(SIGINT, siginit_handler);
    signal(SIGTERM, siginit_handler);

    //////////
    // UART //
    //////////
    int uart_fd = open_serial("/dev/ttyAMA0", B115200); // ../inc/serial.h->open_serial
    if (uart_fd < 0) {
        perror("Failed to open serial port\n");
        return 1;
    }
    
    /////////
    // PWM //
    /////////
    if (gpio_init() < 0) { // ../inc/gpio.h->gpio_init()
        fprintf(stderr, "GPIO initialization failed\n");
        close(uart_fd);
        return 1;
    }
    start_motor(0.85); // ../inc/gpio.h->start_motor(float)
    
    //////////////////////////
    // Data reception cycle //
    //////////////////////////
    struct pollfd fds[1];
    fds[0].fd = uart_fd;
    fds[0].events = POLLIN;
    
    lidar_parser_init(&parser);

    unsigned char buffer[256];
    
    while (running) {
        int ready = poll(fds, 1, -1); // timeout = -1 (never ends)
	if (ready < 0) {
		if (errno == EINTR) {
			continue;
		}
		perror("poll");
		break;
	}
	if (ready == 0) {
		continue;
	}
	
	if (fds[0].revents & POLLIN) {
		int bytes_read = read(uart_fd, buffer, sizeof(buffer));
        	if (bytes_read > 0) {
		    feed_data(&parser , buffer, bytes_read);  // ../inc/parser.h->feed_data
        	} else if (bytes_read < 0) {
                    perror("Read error");
		    break;    
		}
    	}

	if (fds[0].revents & (POLLERR | POLLHUP)) {
            fprintf(stderr, "Connection error with lidar (POLLERR/POLLHUP)\n");
            break;
	}
    }
    
    // Корректное завершение
    stop_motor(); // ../inc/gpio.h->stoop_motor()
    gpio_cleanup(); // ../inc/gpio.h->gpio_cleanup()
    close(uart_fd);
    printf("Program terminated\n");
    return 0;
}

void siginit_handler(int sig) {
	running = 0;
}


#ifndef PARSER_H
#define PARSER_H

#define PACKET_LEN 22
#define START_BYTE 0xFA
#define BUF_SIZE 8192

#include <stddef.h>  
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>   
#include <string.h>  

typedef struct {
    uint8_t buf[BUF_SIZE];
    size_t buf_len;
    float scan[360];  
    int packets;
    int bad_packets;

} lidar_parser_t;

void lidar_parser_init(lidar_parser_t *parser);
void feed_data(lidar_parser_t *parser, const uint8_t *data, size_t len);
void parse_packet(lidar_parser_t *parser, const uint8_t *pkt);
void on_scan_ready(lidar_parser_t *parser, float rpm);

#endif

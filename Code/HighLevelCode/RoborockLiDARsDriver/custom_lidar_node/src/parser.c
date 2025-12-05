#include "../include/parser.h"

void lidar_parser_init(lidar_parser_t *parser) {
    memset(parser, 0, sizeof(*parser));
    for (int i = 0; i < 360; i++) {
        parser->scan[i] = NAN;
    }
}

void feed_data(lidar_parser_t* parser, const uint8_t* data, size_t len) {
    // Проверка на переполнение буфера
    if (parser->buf_len + len > BUF_SIZE) {
        parser->buf_len = 0;  // Сброс буфера при переполнении
        return;
    }
    
    // Добавление новых данных в буфер
    memcpy(parser->buf + parser->buf_len, data, len);
    parser->buf_len += len;
    
    // Обработка буфера
    while (parser->buf_len >= PACKET_LEN) {
        // Поиск стартового байта
        int start_idx = -1;
        for (int i = 0; i <= parser->buf_len - PACKET_LEN; i++) {
            if (parser->buf[i] == START_BYTE) {
                start_idx = i;
                break;
            }
        }
        
        // Стартовый байт не найден
        if (start_idx == -1) {
            parser->buf_len = 0;  // Очистка буфера
            return;
        }
        
        // Удаление мусора до стартового байта
        if (start_idx > 0) {
            memmove(parser->buf, parser->buf + start_idx, parser->buf_len - start_idx);
            parser->buf_len -= start_idx;
        }
        
        // Проверка на достаточность данных
        if (parser->buf_len < PACKET_LEN) {
            return;
        }
        
        // Извлечение и обработка пакета
        parse_packet(parser, parser->buf);
        
        // Удаление обработанного пакета из буфера
        memmove(parser->buf, parser->buf + PACKET_LEN, parser->buf_len - PACKET_LEN);
        parser->buf_len -= PACKET_LEN;
    }
}

void parse_packet(lidar_parser_t* parser, const uint8_t* pkt) {
    // Проверка валидности пакета
    if (pkt[0] != START_BYTE) {
        parser->bad_packets++;
        return;
    }

    uint8_t idx = pkt[1];
    if (idx < 0xA0 || idx > 0xF9) {
        parser->bad_packets++;
        return;
    }

    parser->packets++;

    // Извлечение скорости вращения
    uint16_t speed_raw = pkt[2] | (pkt[3] << 8);
    uint8_t packet_num = idx - 0xA0;  // 0..89

    // Обработка измерений
    for (int i = 0; i < 4; i++) {
        int offset = 4 + i * 4;
        uint8_t b0 = pkt[offset];
        uint8_t b1 = pkt[offset + 1];
        uint8_t b2 = pkt[offset + 2];
        uint8_t b3 = pkt[offset + 3];

        // Проверка валидности измерения
        bool invalid = (b1 & 0x80) != 0;
        uint16_t dist_mm = b0 | ((b1 & 0x3F) << 8);
        uint16_t intensity = b2 | (b3 << 8);

        // Вычисление угла
        int angle = packet_num * 4 + i;

        // Сохранение результата
        if (invalid || dist_mm == 0) {
            parser->scan[angle] = NAN;
        } else {
            parser->scan[angle] = dist_mm / 1000.0f;  // Перевод в метры
        }
    }

    // After full circle send data to topic.
    if (packet_num == 89) {
	if (parser->cb) {
            parser->cb(parser->scan, speed_raw / 64.0f, parser->cb_user);
        }

        // Сброс данных скана
        for (int i = 0; i < 360; i++) {
            parser->scan[i] = NAN;
        }
    }
}


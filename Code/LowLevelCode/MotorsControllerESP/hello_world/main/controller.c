// hello_world_main.c  — исправленный вариант

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

#define IN1 GPIO_NUM_32
#define IN2 GPIO_NUM_33
#define IN3 GPIO_NUM_25
#define IN4 GPIO_NUM_26

#define ENA1 GPIO_NUM_5   // MCPWM0A
#define ENB1 GPIO_NUM_18  // MCPWM0B

#define IN5 GPIO_NUM_27
#define IN6 GPIO_NUM_14
#define IN7 GPIO_NUM_12
#define IN8 GPIO_NUM_13

#define ENA2 GPIO_NUM_19  // MCPWM1A
#define ENB2 GPIO_NUM_21  // MCPWM1B

#define UART_RX_PIN  16
#define UART_TX_PIN  17
#define UART_NUM     UART_NUM_1
#define BUF_SIZE     1024
#define RD_BUF_SIZE  (BUF_SIZE)
#define QUEUE_SIZE   16     // очередь сообщений от uart -> pid

// Структуры
typedef struct {
    uint32_t timestamp_ms; // millis()
    float rpmA, rpmB, rpmC, rpmD;
} rpm_msg_t;

typedef struct {
    float Kp, Ki, Kd;
    float integrator;
    float prev_error;
    float prev_measurement;
    float d_filtered;
    float out; // последнее значение управляющего (0..100)
    float out_min, out_max;
    float integrator_min, integrator_max;
    float deriv_tau; // фильтр времени для D (seconds)
} pid_t;

// Глобали
static const char *TAG = "main";
static QueueHandle_t uart_queue = NULL;   // очередь событий UART (устанавливается в uart_driver_install)
static QueueHandle_t rpm_queue  = NULL;   // очередь распарсенных rpm сообщений для pid_task

// --- прототипы ---
void setup_gpio(void);
void setup_mcpwm(void);
void set_motor(float duty_percent, gpio_num_t dir1, gpio_num_t dir2, uint8_t direction);
void apply_control(int motor_index, float u_percent); // u может быть +/-; индекс 0..3
void motor_task(void *arg);

void pid_init(pid_t *p, float Kp, float Ki, float Kd, float out_min, float out_max);
float pid_update(pid_t *p, float setpoint, float measurement, float Ts);
void pid_task(void *arg);

void uart_init_event(void);
void uart_event_task(void *pvParameters);

// ===================================================================================
// app_main
// ===================================================================================
void app_main(void)
{
    ESP_LOGI(TAG, "Starting app");

    setup_gpio();
    setup_mcpwm();

    // очередь для rpm сообщений (parser -> pid)
    rpm_queue = xQueueCreate(QUEUE_SIZE, sizeof(rpm_msg_t));
    if (rpm_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create rpm_queue");
        return;
    }

    // init UART (создаст uart_queue внутренне)
    uart_init_event();

    // задачи
    xTaskCreate(motor_task, "motor_task", 3072, NULL, 4, NULL);
    xTaskCreate(uart_event_task, "uart_rx_task", 4096, NULL, 8, NULL);
    xTaskCreate(pid_task, "pid_task", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "Tasks created");
}

// ===================================================================================
// GPIO / MCPWM
// ===================================================================================
void setup_gpio(void)
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL<<IN1) | (1ULL<<IN2) | (1ULL<<IN3) | (1ULL<<IN4)
                      | (1ULL<<IN5) | (1ULL<<IN6) | (1ULL<<IN7) | (1ULL<<IN8),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

void setup_mcpwm(void)
{
    // Привязка пинов MCPWM
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, ENA1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, ENB1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, ENA2);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, ENB2);

    mcpwm_config_t pwm_config = {
        .frequency = 1000,    // 1 kHz
        .cmpr_a = 0,
        .cmpr_b = 0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0
    };

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
}

// Установить dir пины + duty (duty 0..100)
void set_motor(float duty_percent, gpio_num_t dir1, gpio_num_t dir2, uint8_t direction)
{
    // direction: 0 = forward (dir1=1, dir2=0), 1 = reverse (dir1=0, dir2=1)
    gpio_set_level(dir1, direction ? 0 : 1);
    gpio_set_level(dir2, direction ? 1 : 0);

    if (duty_percent < 0.0f) duty_percent = 0.0f;
    if (duty_percent > 100.0f) duty_percent = 100.0f;

    // Заметь: mcpwm_set_duty принимает float
    // Нужно указывать unit/timer/operator корректно при вызове (см. apply_control)
    // Здесь простой wrapper не знает таймеров — apply_control вызовет mcpwm_set_duty напрямую.
}

// Apply control: индекс мотора 0..3 (A,B,C,D), u_percent может быть отрицательным
// преобразует sign->dir и вызывает mcpwm_set_duty с соответствующим оператором.
void apply_control(int motor_index, float u_percent)
{
    uint8_t direction = (u_percent >= 0.0f) ? 0 : 1; // 0 forward, 1 reverse
    float duty = fabsf(u_percent);
    duty = fminf(fmaxf(duty, 0.0f), 100.0f);

    switch (motor_index) {
        case 0: // A -> MCPWM_TIMER_0, OPR_A, pins IN1/IN2
            gpio_set_level(IN1, direction ? 0 : 1);
            gpio_set_level(IN2, direction ? 1 : 0);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
            break;
        case 1: // B -> TIMER_0 OPR_B, IN3/IN4
            gpio_set_level(IN3, direction ? 0 : 1);
            gpio_set_level(IN4, direction ? 1 : 0);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty);
            break;
        case 2: // C -> TIMER_1 OPR_A, IN5/IN6
            gpio_set_level(IN5, direction ? 0 : 1);
            gpio_set_level(IN6, direction ? 1 : 0);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty);
            break;
        case 3: // D -> TIMER_1 OPR_B, IN7/IN8
            gpio_set_level(IN7, direction ? 0 : 1);
            gpio_set_level(IN8, direction ? 1 : 0);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, duty);
            break;
        default:
            break;
    }
}

// ===================================================================================
// простая тестовая задача моторов (open-loop)
// ===================================================================================
void motor_task(void *arg)
{
    while (1) {
        // Тестовые значения (просто чтобы было видно поведение)
        apply_control(0, 60.0f);  // motor A  +60%
        apply_control(1, 60.0f); // motor B  +60%
        apply_control(2, 60.0f);  // motor C
        apply_control(3, 60.0f); // motor D

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// ===================================================================================
// UART init + event driven reader
// ===================================================================================
void uart_init_event(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Устанавливаем драйвер с очередью событий uart_queue
    // uart_driver_install запишет дескриптор очереди в uart_queue
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, RD_BUF_SIZE * 2, 0, 10, &uart_queue, 0));
}

// Event-driven UART reader: собирает строки, парсит и отправляет rpm_msg_t в rpm_queue
void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t *dtmp = (uint8_t*) malloc(BUF_SIZE);
    if (!dtmp) {
        ESP_LOGE(TAG, "uart buffer malloc failed");
        vTaskDelete(NULL);
        return;
    }
    size_t idx = 0;

    for (;;) {
        // ждём событие
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                int rxlen = uart_read_bytes(UART_NUM, dtmp + idx, event.size, pdMS_TO_TICKS(100));
                if (rxlen > 0) {
                    // пройдём по новым байтам и соберём строки
                    size_t end = idx + rxlen;
                    for (size_t i = idx; i < end; ++i) {
                        uint8_t ch = dtmp[i];
                        if (ch == '\r') {
                            // игнорируем
                            continue;
                        } else if (ch == '\n') {
                            // строка закончилась — терминируем и парсим
                            dtmp[i] = '\0';
                            // Собрали строку в dtmp[0..i-1] (если idx==0) или с начала буфера.
                            // Чтобы упростить — копируем текущую строку в отдельный буфер
                            char line[256];
                            size_t linelen = strlen((char*)dtmp);
                            if (linelen >= sizeof(line)) linelen = sizeof(line)-1;
                            memcpy(line, dtmp, linelen);
                            line[linelen] = '\0';
                            // Лог
                            ESP_LOGI(TAG, "Line: %s", line);

                            // Парсинг: ожидаем "A:%.2f,B:%.2f,C:%.2f,D:%.2f"
                            rpm_msg_t msg;
                            msg.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                            float a,b,c,d;
                            int parsed = sscanf(line, "A:%f,B:%f,C:%f,D:%f", &a, &b, &c, &d);
                            if (parsed == 4 && isfinite(a) && isfinite(b) && isfinite(c) && isfinite(d)) {
                                msg.rpmA = a;
                                msg.rpmB = b;
                                msg.rpmC = c;
                                msg.rpmD = d;
                                // Отправляем в очередь (без блокировки). Если очередь полна — отбрасываем старое.
                                if (xQueueSend(rpm_queue, &msg, 0) != pdTRUE) {
                                    ESP_LOGW(TAG, "rpm_queue full, dropping message");
                                }
                            } else {
                                ESP_LOGW(TAG, "Bad format (parsed=%d): %s", parsed, line);
                            }

                            // очистим буфер для следующей строки
                            idx = 0;
                            // очистим dtmp (не обязательно, но безопасно)
                            memset(dtmp, 0, BUF_SIZE);
                            // продолжим
                        } else {
                            // обычный символ — сдвигаем в начало буфера, если надо
                            if (i != idx) {
                                // This branch won't be hit because we fill sequentially, keep simple.
                            }
                            // добавлять символ в буфер (если есть место)
                            if (idx < BUF_SIZE - 1) {
                                dtmp[idx++] = ch;
                            } else {
                                // переполнение строки
                                ESP_LOGW(TAG, "UART line too long, resetting buffer");
                                idx = 0;
                            }
                        }
                    }
                }
            } else if (event.type == UART_FIFO_OVF) {
                ESP_LOGW(TAG, "UART FIFO overflow");
                uart_flush_input(UART_NUM);
                xQueueReset(uart_queue);
            } else if (event.type == UART_BUFFER_FULL) {
                ESP_LOGW(TAG, "UART Buffer full");
                uart_flush_input(UART_NUM);
                xQueueReset(uart_queue);
            } else {
                ESP_LOGW(TAG, "uart event type: %d", event.type);
            }
        }
    }

    // unreachable but tidy
    free(dtmp);
    vTaskDelete(NULL);
}

// ===================================================================================
// PID implementation
// ===================================================================================
void pid_init(pid_t *p, float Kp, float Ki, float Kd, float out_min, float out_max)
{
    p->Kp = Kp; p->Ki = Ki; p->Kd = Kd;
    p->integrator = 0.0f;
    p->prev_error = 0.0f;
    p->prev_measurement = 0.0f;
    p->d_filtered = 0.0f;
    p->out = 0.0f;
    p->out_min = out_min; p->out_max = out_max;
    p->integrator_min = -1000.0f; p->integrator_max = 1000.0f;
    p->deriv_tau = 0.02f; // 20 ms default
}

float pid_update(pid_t *p, float setpoint, float measurement, float Ts)
{
    float error = setpoint - measurement;

    float P = p->Kp * error;
    p->integrator += p->Ki * error * Ts;
    // clamp integrator
    
    if (p->integrator > p->integrator_max) p->integrator = p->integrator_max;
    if (p->integrator < p->integrator_min) p->integrator = p->integrator_min;
    float I = p->integrator;

    // Derivative on measurement (reduces derivative kick)
    float d_meas = (measurement - p->prev_measurement) / (Ts > 0 ? Ts : 1e-6f);
    float D_raw = - p->Kd * d_meas;
    float alpha = p->deriv_tau / (p->deriv_tau + Ts);
    p->d_filtered = alpha * p->d_filtered + (1.0f - alpha) * D_raw;
    float D = p->d_filtered;
    
    float u = P + I + D;

    // clamp final output
    if (u > p->out_max) u = p->out_max;
    if (u < p->out_min) u = p->out_min;

    // slew limiter (пример)
    
    float max_delta = 50.0f * Ts; // % per second -> scaled by Ts; подберите экспериментально
    if ((u - p->out) > max_delta) u = p->out + max_delta;
    if ((u - p->out) < -max_delta) u = p->out - max_delta;
    
    p->prev_error = error;
    p->prev_measurement = measurement;
    p->out = u;

    return u;
}

// ===================================================================================
// PID task: получает последние rpm из rpm_queue и управляет моторами (apply_control)
// ===================================================================================
void pid_task(void *arg)
{
    const TickType_t Ts_ticks = pdMS_TO_TICKS(100); // 100 ms
    const float Ts = 0.1f;
    TickType_t last_wake = xTaskGetTickCount();

    pid_t pidA, pidB, pidC, pidD;
    // Начальные коэффициенты — подбор эмпирически!
    pid_init(&pidA, 6.0f, 1.0f, 1.5f, -100.0f, 100.0f);
    pid_init(&pidB, 6.0f, 1.0f, 1.5f, -100.0f, 100.0f);
    pid_init(&pidC, 6.0f, 1.0f, 1.5f, -100.0f, 100.0f);
    pid_init(&pidD, 6.0f, 1.0f, 1.5f, -100.0f, 100.0f);

    float setA = 60.0f, setB = 60.0f, setC = 60.0f, setD = 60.0f;

    rpm_msg_t msg;
    for (;;) {
        vTaskDelayUntil(&last_wake, Ts_ticks);

        // Забираем самый свежий элемент из очереди (если есть)
        bool have = false;
        while (xQueueReceive(rpm_queue, &msg, 0) == pdTRUE) {
            have = true; // продолжаем, чтобы прочитать последний
        }

        if (!have) {
            // Нет новых данных — можно пропускать или держать предыдущие значения
            ESP_LOGW(TAG, "No rpm message this cycle");
            continue;
        }

        // Проверка значений
        if (!isfinite(msg.rpmA) || !isfinite(msg.rpmB) || !isfinite(msg.rpmC) || !isfinite(msg.rpmD)) {
            ESP_LOGW(TAG, "Invalid rpm data, skipping");
            continue;
        }

        // Вычисляем PID для каждого колеса (выход в % PWM, может быть отрицательным)
        float uA = pid_update(&pidA, setA, msg.rpmA, Ts);
        float uB = pid_update(&pidB, setB, msg.rpmB, Ts);
        float uC = pid_update(&pidC, setC, msg.rpmC, Ts);
        float uD = pid_update(&pidD, setD, msg.rpmD, Ts);

        // Применяем — apply_control расставит dir и mcpwm_set_duty
        apply_control(0, uA);
        apply_control(1, uB);
        apply_control(2, uC);
        apply_control(3, uD);

        ESP_LOGI(TAG, "PID out A:%.2f B:%.2f C:%.2f D:%.2f | meas A:%.2f B:%.2f C:%.2f D:%.2f",
                 uA, uB, uC, uD, msg.rpmA, msg.rpmB, msg.rpmC, msg.rpmD);
    }
}


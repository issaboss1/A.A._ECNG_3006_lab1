#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_spi_flash.h"

#define LED_GPIO    2
#define UART_PORT   UART_NUM_0   // default UART connected to USB
#define BUF_SIZE    1024
#define DEBOUNCE_MS 500

typedef enum {
    STATE_OFF,
    STATE_ON
} fsm_state_t;

static fsm_state_t state = STATE_OFF;
static char last_char = 0;
static TickType_t last_tick = 0;

void fsm_task(void *pv) {
    uint8_t data;
    TickType_t now;

    while (1) {
        int len = uart_read_bytes(UART_PORT, &data, 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            now = xTaskGetTickCount() * portTICK_PERIOD_MS;

            if (data == last_char && (now - last_tick) < DEBOUNCE_MS) {
                // ignore repeated char
            } else {
                last_char = data;
                last_tick = now;

                switch (state) {
                    case STATE_OFF:
                        gpio_set_level(LED_GPIO, 1);
                        state = STATE_ON;
                        printf("FSM: switched to ON (char=%c)\n", data);
                        break;

                    case STATE_ON:
                        gpio_set_level(LED_GPIO, 0);
                        state = STATE_OFF;
                        printf("FSM: switched to OFF (char=%c)\n", data);
                        break;
                }
            }
        }
    }
}

void app_main(void) {
    // Configure LED
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);

    // Configure UART0
    const uart_config_t uart_config = {
        .baud_rate = 74880,   // ESP8266 default boot baud
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_PORT, &uart_config);
    uart_driver_install(UART_PORT, BUF_SIZE, 0, 0, NULL, 0);

    // Start FSM task
    xTaskCreate(fsm_task, "fsm_task", 2048, NULL, 5, NULL);
}

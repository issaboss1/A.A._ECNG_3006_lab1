/* 
   816035242
   Issa Mohamed
   ecnr3006_lab1_Q5
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#define SLOTX 4
#define CYCLEX 5
#define SLOT_T 5000 // 5 sec slot time (ms)

void one() {
    printf("task 1 running\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Replacement for sleep function and 1s delay
}
void two() {
    printf("task 2 running\n");
    vTaskDelay(2000 / portTICK_PERIOD_MS);//2s
}
void three() {
    printf("task 3 running\n");
    vTaskDelay(3000 / portTICK_PERIOD_MS);//3s
}
void four() {
    printf("task 4 running\n");
    vTaskDelay(4000 / portTICK_PERIOD_MS);//4s
}
void five() {
    printf("task 5 running\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);//5s
}
void burn() {
    TickType_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(SLOT_T)) {// burn CPU cycles for SLOT_T ms and replacement for Time function
    }
    TickType_t elapsed = xTaskGetTickCount() - start;
    printf("burn time = %d ms\n\n", elapsed * portTICK_PERIOD_MS);
}

//table
void (*ttable[SLOTX][CYCLEX])() = {
    {one, two, burn, burn, burn},
    {one, three, burn, burn, burn},
    {one, four, burn, burn, burn},
    {burn, burn, burn, burn, burn}
};

void dispatcher_task(void *pvParameter) {
    while (1) {
        for (int slot = 0; slot < SLOTX; slot++) {
            for (int cycle = 0; cycle < CYCLEX; cycle++) {
                (*ttable[slot][cycle])(); // run next scheduled task
            }
        }
    }
}

void app_main() {
    printf("Starting cyclic task dispatcher...\n");
    xTaskCreate(&dispatcher_task, "dispatcher_task", 2048, NULL, 5, NULL);
}

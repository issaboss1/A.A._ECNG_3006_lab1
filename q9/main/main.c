//816035242
//ecng3006 lab1 question 9 adc code

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_spi_flash.h"
#include <math.h>

#define I2C_MASTER_NUM 0
#define I2C_MASTER_SCL_IO 0//ports gp0 connected to scl
#define I2C_MASTER_SDA_IO 2//port gp2 connected to sda
#define I2C_MASTER_FREQ_HZ      100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

#define ADS1115_ADDR 0x48 // wiring connectting address register to ground therefore 0x48 from ads1115 datasheet
#define ADS1115_REG_CONVERT 0x00
#define ADS1115_REG_CONFIG 0x01

#define NUM_SAMPLES             1000
#define SAMPLE_RATE_HZ          500//sample rate for this question asked to sample at 1000hz but with the esp8266  module 1 that is impossible with a maximum sampling rate of 960 i think cant remember.

static const char *TAG = "ADS1115_APP";
static uint16_t DataBuf[NUM_SAMPLES];
static uint32_t TimeBuf[NUM_SAMPLES];

static uint32_t int_sqrt(uint32_t x) {
    uint32_t res = 0;
    uint32_t bit = 1UL << 30;
    while (bit > x)
        bit >>= 2;
    while (bit != 0) {
        if (x >= res + bit) {
            x -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

static uint8_t int_log2(uint32_t val) {
    uint8_t r = 0;
    while (val >>= 1)
        r++;
    return r;
}

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = 1;
    conf.scl_pullup_en = 1;
    conf.clk_stretch_tick = 300; // default stretch

    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_LOGI(TAG, "I2C initialized");
    return ESP_OK;
}

static uint16_t ads1115_read(void)
{
    uint8_t reg = ADS1115_REG_CONVERT;
    uint8_t data[2];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ((data[0] << 8) | data[1]);
}

void sample_task(void *pv)
{
    uint32_t start_time, diff, max_diff, min_diff, jitter;
    uint64_t sum = 0, sum_sq = 0;
    uint32_t avg, stddev, snr;
    uint8_t enob;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        start_time = esp_timer_get_time(); // microseconds
        DataBuf[i] = ads1115_read();
        TimeBuf[i] = start_time;
        vTaskDelay(pdMS_TO_TICKS(1000 / SAMPLE_RATE_HZ)); // 2ms
    }
    max_diff = 0;
    min_diff = 0xFFFFFFFF;
    for (int i = 1; i < NUM_SAMPLES; i++) {
        diff = TimeBuf[i] - TimeBuf[i - 1];
        if (diff > max_diff) max_diff = diff;
        if (diff < min_diff) min_diff = diff;
    }
    jitter = max_diff - min_diff;
    ESP_LOGI(TAG, "Jitter: %u us", jitter);
    for (int i = 0; i < NUM_SAMPLES; i++) sum += DataBuf[i];
    avg = sum / NUM_SAMPLES;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        int32_t d = DataBuf[i] - avg;
        sum_sq += d * d;
    }
    stddev = int_sqrt(sum_sq / NUM_SAMPLES);

    snr = (stddev == 0) ? 0 : (avg / stddev);
    enob = int_log2(snr);

    ESP_LOGI(TAG, "Signal (avg): %u", avg);
    ESP_LOGI(TAG, "Noise (stddev): %u", stddev);
    ESP_LOGI(TAG, "SNR: %u", snr);
    ESP_LOGI(TAG, "ENOB: %u bits", enob);

    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    xTaskCreate(sample_task, "sample_task", 4096, NULL, 5, NULL);
}

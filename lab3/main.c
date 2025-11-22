#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_NUM 0
#define I2C_SCL_PIN 0//GPIO pin for SDA
#define I2C_SDA_PIN 2//pin for scl 
#define I2C_FREQ 100000

#define ADS1115_ADDR 0x48//ground connected to addr 
#define ADS1115_CONVERT_REG 0x00
#define ADS1115_CONFIG_REG  0x01

static const char *TAG = "ADS1115_APP";

// I2C helpers (ESP8266 SAFE)
static bool write_reg(uint8_t reg, uint16_t value) {
    uint8_t data[3];
    data[0] = reg;
    data[1] = (value >> 8);
    data[2] = (value & 0xFF);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, 3, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK);
}

static bool read_reg(uint8_t reg, uint16_t *value) {
    uint8_t data[2];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    *value = ((data[0] << 8) | data[1]);
    return (ret == ESP_OK);
}


// REQUIRED ROUTINES
bool init_adc(void) {
    ESP_LOGI(TAG, "Init ADC");

    uint16_t config =
        (0x01 << 12) |   // AIN0 single-ended
        (0x01 << 9)  |   // ±4.096V
        (0x01 << 8)  |   // continuous mode
        (0x04 << 5)  |   // 128 SPS
        (0x00);

    return write_reg(ADS1115_CONFIG_REG, config);
}

bool start_adc(void) {
    uint16_t cfg;
    if (!read_reg(ADS1115_CONFIG_REG, &cfg)) return false;

    cfg |= (1 << 15);    // Start conversion

    return write_reg(ADS1115_CONFIG_REG, cfg);
}

bool stop_adc(void) {
    uint16_t cfg;

    if (!read_reg(ADS1115_CONFIG_REG, &cfg)) return false;

    cfg &= ~(1 << 8);    // Disable continuous mode

    return write_reg(ADS1115_CONFIG_REG, cfg);
}

bool read_adc(uint16_t *value) {
    return read_reg(ADS1115_CONVERT_REG, value);
}

// ESP8266-COMPATIBLE I2C INIT
static void init_i2c(void) {
    i2c_config_t conf;

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_PIN;
    conf.scl_io_num = I2C_SCL_PIN;
    conf.sda_pullup_en = 1;
    conf.scl_pullup_en = 1;
    conf.clk_stretch_tick = 300;

    // MUST call install BEFORE param_config on ESP8266
    i2c_driver_install(I2C_MASTER_NUM, conf.mode);
    i2c_param_config(I2C_MASTER_NUM, &conf);
}

// TASK
void adc_task(void *arg) {
    uint16_t value;

    if (!init_adc()) { ESP_LOGE(TAG, "Init fail"); vTaskDelete(NULL); }
    if (!start_adc()) { ESP_LOGE(TAG, "Start fail"); vTaskDelete(NULL); }

    ESP_LOGI(TAG, "Sampling...");

    while (1) {
        if (read_adc(&value)) {
            ESP_LOGI(TAG, "ADC: %u", value);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    stop_adc();
}

// MAIN
void app_main(void) {
    init_i2c();
    xTaskCreate(adc_task, "adc_task", 4096, NULL, 5, NULL);
}

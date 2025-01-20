/*******************************************************************************
 * main.cpp
 * 
 * Main program for testing sensor libraries.
 * Sense-AI
********************************************************************************
*******************************************************************************/
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"


// ESP I2C pins
#define I2C_MASTER_NUM1          I2C_NUM_0  // I2C port number for master dev
#define I2C_MASTER_NUM2          I2C_NUM_1  // I2C port number for master dev
#define I2C_MASTER_SDA_IO1       GPIO_NUM_8 // GPIO number for I2C data line
#define I2C_MASTER_SCL_IO1       GPIO_NUM_18 // GPIO number for I2C clock line
#define I2C_MASTER_SDA_IO2       GPIO_NUM_35 // GPIO number for I2C data line
#define I2C_MASTER_SCL_IO2       GPIO_NUM_36 // GPIO number for I2C clock line
#define I2C_MASTER_FREQ_HZ      400000     // Frequency of I2C clock
#define I2C_MASTER_TIMEOUT_MS   1000       // Timeout for I2C operation


static void scanI2cTask0(void* pvParameters) {
    esp_err_t ret;

    // I2C configuration
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO1,
        .scl_io_num = I2C_MASTER_SCL_IO1,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .clk_flags = 0
    };
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    ret = i2c_param_config(I2C_MASTER_NUM1, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C0", "Failed to configure I2C0: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM1, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C0", "Failed to install I2C0 driver: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI("I2C0_SCAN", "Starting I2C0 scan...");
    while (true) {
        ESP_LOGI("I2C0_SCAN", "Scanning for I2C0 devices...");
        for (uint8_t address = 1; address < 127; address++) {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(cmd);
            ret = i2c_master_cmd_begin(I2C_MASTER_NUM1, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
            i2c_cmd_link_delete(cmd);

            if (ret == ESP_OK) {
                ESP_LOGI("I2C0_SCAN", "Device found at address 0x%02X", address);
            }
        }
        ESP_LOGI("I2C0_SCAN", "Scan complete. Waiting before next scan...");
        vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 5 seconds before the next scan
    }
}

static void scanI2cTask1(void* pvParameters) {
    esp_err_t ret;

    // I2C configuration
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO2,
        .scl_io_num = I2C_MASTER_SCL_IO2,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    };
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    ret = i2c_param_config(I2C_MASTER_NUM2, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C1", "Failed to configure I2C1: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM2, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C1", "Failed to install I2C1 driver: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI("I2C1_SCAN", "Starting I2C1 scan...");
    while (true) {
        ESP_LOGI("I2C1_SCAN", "Scanning for I2C devices...");
        for (uint8_t address = 1; address < 127; address++) {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(cmd);
            ret = i2c_master_cmd_begin(I2C_MASTER_NUM2, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
            i2c_cmd_link_delete(cmd);

            if (ret == ESP_OK) {
                ESP_LOGI("I2C1_SCAN", "Device found at address 0x%02X", address);
            }
        }
        ESP_LOGI("I2C1_SCAN", "Scan complete. Waiting before next scan...");
        vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 5 seconds before the next scan
    }
}


extern "C" void app_main() {
    xTaskCreate(scanI2cTask0, "I2CScanTask", 4096, NULL, 5, NULL);
    xTaskCreate(scanI2cTask1, "I2CScanTask", 4096, NULL, 5, NULL);
}


void doorCallback(void* arg) {
    printf("Interrupt!\n");
}
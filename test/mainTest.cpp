#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

extern "C" void app_main() {
    // Try to reconfigure GPIO45 early
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << 45); // GPIO 45
    gpio_config(&io_conf);
    gpio_set_level(GPIO_NUM_45, 1); // Set it high to match I2C idle state
    
    printf("ESP32-S3 Simple Test Program\n");
    printf("Reconfigured GPIO 45 for I2C use\n");
    
    int counter = 0;
    while(true) {
        printf("Hello World! Counter: %d\n", counter++);
        vTaskDelay(pdMS_TO_TICKS(1000));  // 1 second delay
    }
}
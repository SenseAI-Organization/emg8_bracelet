/*******************************************************************************
 * main.cpp
 * 
 * Main program for testing sensor libraries.
 * Sense-AI
********************************************************************************
*******************************************************************************/
#include <mutex>
#include <cstring>
#include "esp_task_wdt.h"
#include "ADS1015.hpp"
#include "actuators_sense.hpp"
#include "switch_sense.hpp"

constexpr gpio_num_t kSDA1 = GPIO_NUM_8;
constexpr gpio_num_t kSCL1 = GPIO_NUM_18;
I2C i2c1(I2C_NUM_1, kSDA1, kSCL1, 400000, false);

constexpr gpio_num_t kSDA0 = GPIO_NUM_35;
constexpr gpio_num_t kSCL0 = GPIO_NUM_36;
I2C i2c0(I2C_NUM_0, kSDA0, kSCL0, 400000, false);

struct AdcTaskParams {
    const char* name;
    I2C* i2c;
    int startChannel;
};

struct sensorData {
    uint16_t channels[8];
};
sensorData sData;

extern "C" void app_main() {
    // Initialize I2C
    i2c0.init();

    // Initialize ADS1015
    ADS1015 adc(i2c0);
    adc.init();

    while (true) {
        //printf("ADC Readings: ");
        for (int i = 0; i < 4; i++) {
            uint16_t value = adc.readSingleEndedSigned(i);
            printf("%d ", value);
        }
        printf("\n");

        vTaskDelay(pdMS_TO_TICKS(1));  // Delay 100ms
    }
}
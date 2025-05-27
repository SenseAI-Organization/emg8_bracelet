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

constexpr gpio_num_t kSDA1 = GPIO_NUM_45;
constexpr gpio_num_t kSCL1 = GPIO_NUM_47;
I2C i2c1(I2C_NUM_1, kSDA1, kSCL1, 400000, false);

constexpr gpio_num_t kSDA0 = GPIO_NUM_6;
constexpr gpio_num_t kSCL0 = GPIO_NUM_7;
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

uint8_t buffer[2];

extern "C" void app_main() {
    // Initialize I2C
    i2c0.init();
    i2c1.init();

    ADS1015 ads1(i2c0, ADS1015::ADS111X_Address::ADS111X_ADDR_GND);
    ADS1015 ads2(i2c0, ADS1015::ADS111X_Address::ADS111X_ADDR_VCC);
    ADS1015 ads3(i2c1, ADS1015::ADS111X_Address::ADS111X_ADDR_GND);
    ADS1015 ads4(i2c1, ADS1015::ADS111X_Address::ADS111X_ADDR_VCC);

    if (!ads1.checkForDevice()){
        printf("ADS1 not found\n");
    }
    if (!ads2.checkForDevice()){
        printf("ADS2 not found\n");
    }
    if (!ads3.checkForDevice()){
        printf("ADS2 not found\n");
    }
    if (!ads4.checkForDevice()){
        printf("ADS2 not found\n");
    }


    while (true) {
        //printf("ADC Readings: ");
        for (int i = 0; i < 4; i++) {
            uint16_t value1 = ads1.readSingleEndedSigned(i);
            uint16_t value2 = ads2.readSingleEndedSigned(i);
            uint16_t value3 = ads3.readSingleEndedSigned(i);
            uint16_t value4 = ads4.readSingleEndedSigned(i);
            printf("%d %d %d %d ", value1, value2, value3, value4);
        }
        printf("\n");

        vTaskDelay(pdMS_TO_TICKS(1));  // Delay 100ms
    }
}
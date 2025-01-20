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

std::mutex sDataMutex;
volatile bool newDataAvailable_adc0 = false;
volatile bool newDataAvailable_adc1 = false;
SemaphoreHandle_t adc0Semaphore;
SemaphoreHandle_t adc1Semaphore;


void adc_task(void* pvParameters) {
    AdcTaskParams* params = static_cast<AdcTaskParams*>(pvParameters);
    ADS1015 ads(*params->i2c);
    esp_err_t err = params->i2c->init();
    if (err) {
        printf("Error initializing I2C: %s\n", esp_err_to_name(err));
        vTaskDelete(NULL);
    }

    err = ads.init();
    if (err) {
        printf("Error initializing ADC: %s\n", esp_err_to_name(err));
        vTaskDelete(NULL);
    }

    printf("ADC task %s initialized.\n", params->name);

    esp_task_wdt_add(NULL); // Add the task to the watchdog

    SemaphoreHandle_t semaphore = (strcmp(params->name, "adc0") == 0) ? adc0Semaphore : adc1Semaphore;

    while (true) {
        // Wait for the semaphore to be given
        if (xSemaphoreTake(semaphore, portMAX_DELAY) == pdTRUE) {
            // Read ADC values for all 4 channels
            for (int i = 0; i < 4; ++i) {
                uint16_t value = ads.readSingleEndedSigned(i);

                // Lock the mutex to safely update shared data
                sDataMutex.lock();
                sData.channels[params->startChannel + i] = value;
                sDataMutex.unlock();

                // Reset watchdog after processing each channel
                esp_task_wdt_reset();

                // Yield task execution briefly
                taskYIELD();
            }

            // Mark new data available
            if (strcmp(params->name, "adc0") == 0) {
                newDataAvailable_adc0 = true;
            } else {
                newDataAvailable_adc1 = true;
            }

            // Reset watchdog again after finishing the batch
            esp_task_wdt_reset();
            taskYIELD();
        }
        taskYIELD();
    }
}


extern "C" void app_main() {
    // Initialize semaphores
    adc0Semaphore = xSemaphoreCreateBinary();
    adc1Semaphore = xSemaphoreCreateBinary();

    AdcTaskParams* params0 = new AdcTaskParams{"adc0", &i2c0, 0};
    xTaskCreatePinnedToCore(adc_task, "ADC0 Task", 8192, params0, 10, NULL, 1);

    AdcTaskParams* params1 = new AdcTaskParams{"adc1", &i2c1, 4};
    xTaskCreatePinnedToCore(adc_task, "ADC1 Task", 8192, params1, 10, NULL, 1);

    while (true) {
        // Signal tasks to read data
        xSemaphoreGive(adc0Semaphore);
        xSemaphoreGive(adc1Semaphore);

        // Wait for new data from both tasks
        while (!newDataAvailable_adc0 || !newDataAvailable_adc1) {
            taskYIELD();
        }

        // Print the data
        sDataMutex.lock();
        printf("%d %d %d %d %d %d %d %d\n",
            sData.channels[0], sData.channels[1], sData.channels[2], sData.channels[3],
            sData.channels[4], sData.channels[5], sData.channels[6], sData.channels[7]);
        sDataMutex.unlock();

        // Reset data availability flags
        newDataAvailable_adc0 = false;
        newDataAvailable_adc1 = false;

        // Short delay to control output frequency
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

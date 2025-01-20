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


uint32_t sampleTime = 1;   // 500ms sample time
const uint8_t numSensors = 8;
bool dataReadyAds1 = false;
bool dataReadyAds2 = false;

// ESP I2C pins
constexpr gpio_num_t kSDA1 = GPIO_NUM_8;
constexpr gpio_num_t kSCL1 = GPIO_NUM_18;
I2C i2c1(I2C_NUM_1, kSDA1, kSCL1, 400000, false); // Change I2C pins

constexpr gpio_num_t kSDA0 = GPIO_NUM_35;
constexpr gpio_num_t kSCL0 = GPIO_NUM_36;
I2C i2c0(I2C_NUM_0, kSDA0, kSCL0, 400000, false); // Change I2C pins

struct AdcTaskParams {
    const char* name;
    I2C i2c;
};

enum measureState {
    INIT,
    MEASURE,
    OUTPUT
};

struct sensorData {
        uint16_t channels[numSensors];
    };
sensorData sData = {0, 0, 0, 0, 0, 0, 0, 0};

volatile uint8_t buttonFlag = 0;
void userButtonCallback(void* arg);

void adc_task(void* pvParameters) {
    AdcTaskParams* params = static_cast<AdcTaskParams*>(pvParameters);
    I2C& i2c = params->i2c;
    measureState currentState = INIT;

    printf("Starting ADC task %s\n", params->name);

    ADS1015 ads(i2c);   // SDO to GND
    esp_err_t err = i2c.init();
    if (err) {
        printf("Error initializing I2C: %s\n", esp_err_to_name(err));
        while(1);
    }

    err = ads.init(); //metodo general para inicializar el ADS (init)
    if (err) {
        printf("Error initializing adc: %s\n", esp_err_to_name(err));
        while(1);
    }
    printf("Initialized %s\n", params->name);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    while (1){
        switch (currentState) {
            case INIT:
                currentState = MEASURE;
                break;

            case MEASURE:
                ads.measure();
                if (strcmp(params->name, "adc0") == 0) {
                    for (int i = 0; i < 4; ++i) {
                        sData.channels[i] = ads.getChannel(i);
                    }
                    dataReadyAds1 = true;
                } else {
                    for (int i = 0; i < 4; ++i) {
                        sData.channels[i+4] = ads.getChannel(i);
                    }
                    dataReadyAds2 = true;
                }
                currentState = OUTPUT;
                break;

            case OUTPUT:
                if (!dataReadyAds1 && !dataReadyAds2) {
                    currentState = MEASURE;
                }
                vTaskDelay(pdMS_TO_TICKS(100));
                break;

        default:
                currentState = INIT;
                break;
        }
    }
}

void signalSwitch_task(void* pvParameters) {
    //Init digital output for the Mux
    uint8_t muxState = 0;
    const gpio_num_t kExternalLedPin = GPIO_NUM_14;
    LED mux(kExternalLedPin);

    esp_err_t err = mux.init();
    if(err) {
        printf("Mux couldn't be initialized.\n");
        printf("%s\n", esp_err_to_name(err));
        // while(1);
    }

    //Init button reading task
    const bool kExternalPullUp = false;
    Switch userButton(
        GPIO_NUM_21,
        Switch::SwitchMode::kNormallyOpen,
        kExternalPullUp
    );

    err = userButton.configureInterrupt(GPIO_INTR_POSEDGE, 
                                                  userButtonCallback, nullptr);
    if (err) {
        printf("Interrupt configuration error %d", err);
    }

    // Verify task size in case of error while running the program.
    userButton.startHandlerTask("ButtonHandlerTask", 5, 2048);

    err = userButton.init();
    if (err) {
        printf("User button init error %d\n", err);
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    while (1) {
        switch (muxState) {
            case 0:
                if (userButton.isPressed()) {
                    err = mux.turnOff();
                    if(err) {
                        printf("Mux couldn't be turned off.\n");
                        printf("%s\n", esp_err_to_name(err));
                    }
                }
                break;
            case 1:
                if (userButton.isPressed()) {
                    err = mux.turnOn();
                        if(err) {
                            printf("Mux couldn't be turned on.\n");
                            printf("%s\n", esp_err_to_name(err));
                        }
                }
                break;
            default:
                break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void app_main() {
    AdcTaskParams* params0 = new AdcTaskParams{"adc0", i2c0};
    xTaskCreate(adc_task, "ADC Task0", 4096, params0, 5, NULL);
    
    AdcTaskParams* params1 = new AdcTaskParams{"adc1", i2c1};
    xTaskCreate(adc_task, "ADC Task1", 4096, params1, 5, NULL);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    while (true){
        if (dataReadyAds1 && dataReadyAds2) {
            printf("%d %d %d %d %d %d %d %d\n", 
                    sData.channels[0], sData.channels[1], sData.channels[2], sData.channels[3],
                    sData.channels[4], sData.channels[5], sData.channels[6], sData.channels[7]);
            dataReadyAds1 = false;
            dataReadyAds2 = false;
        }
        vTaskDelay(sampleTime / portTICK_PERIOD_MS);
    }
    //xTaskCreate(signalSwitch_task, "Mux Task", 4096, NULL, 5, NULL);
    //xTaskCreate(mux_task, "Mux Task", 2048, NULL, 5, NULL);
}

void userButtonCallback(void* arg) {
    buttonFlag = 1;
    printf("Switching Mux!\n");
}
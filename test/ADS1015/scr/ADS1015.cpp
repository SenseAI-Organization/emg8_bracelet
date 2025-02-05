/*
 * Copyright (c) 2016 Ruslan V. Uss <unclerus@gmail.com>
 * Copyright (c) 2020 Lucio Tarantino <https://github.com/dianlight>
 * @file ads111x.c

 * ESP-IDF driver for ADS1113/ADS1114/ADS1115, ADS1013/ADS1014/ADS1015 I2C ADC
 *
 * Ported from esp-open-rtos
 * BSD Licensed as described in the file LICENSE
 */


#include "ADS1015.hpp"

esp_err_t checkForDevice(I2C& i2cInstance) {
    uint8_t data[1] = {0};
    for (int addr = ADS111X_ADDR_GND; addr <= ADS111X_ADDR_SCL; ++addr) {
        esp_err_t err = i2cInstance.write(addr, REG_CONFIG, data, 1);
        if (err) {
            return err;
        }
    }
    //printf("ADS seems to be working\n");
    return ESP_OK;
}

esp_err_t initADS(I2C& i2cInstance) {
    uint8_t id = checkForDevice(i2cInstance);
    switch (id) {
        case ADS111X_ADDR_GND:
        case ADS111X_ADDR_VCC:
        case ADS111X_ADDR_SDA:
        case ADS111X_ADDR_SCL:
            // id matches one of the enum values
            break;
        default:
            return ESP_ERR_NOT_FOUND;
    }
    printf("ADS found at address: %d\n", id);
    return ESP_OK;
}

esp_err_t read_conf_bits(I2C& i2cInstance, uint8_t * data)
    {
        esp_err_t err = i2cInstance.read(ADS111X_ADDR_GND, REG_CONFIG, data, 2);
        if (err) {
            return err;
        }
        printf("Data: %d %d\n", data[0], data[1]);
        std::bitset<16> bits0(data[0]);
        std::bitset<16> bits1(data[1]);
        std::cout << "Data as bits: " << bits0 << " " << bits1 << std::endl;

        return ESP_OK;
    }

bool available(I2C& i2cInstance)
{
    uint8_t response[2];
    esp_err_t err = i2cInstance.read(ADS111X_ADDR_GND, ADS1015_POINTER_CONFIG, response, 2);
    
    if (err != ESP_OK) {
        return false;
    }

    uint16_t value = (response[0] << 8) | response[1];
    return ((value & ADS1015_CONFIG_OS_READY) > 0); // If the OS bit is 1 : the device is not currently performing a conversion (i.e. data is available)
}


esp_err_t getSingleEnded(I2C& i2cInstance, uint8_t channel, uint8_t * data)
{
    if (channel > 3)
    {
        return 0;
    }

    uint16_t config = ADS1015_CONFIG_OS_SINGLE |
                    ADS1015_CONFIG_CQUE_NONE |
                    ADS1015_CONFIG_RATE_1600HZ;

    config |= ADS1015_CONFIG_PGA_2;
    config |= ADS1015_CONFIG_MODE_SINGLE;

    switch (channel)
    {
        case (0):
            config |= ADS1015_CONFIG_MUX_SINGLE_0;
            break;
        case (1):
            config |= ADS1015_CONFIG_MUX_SINGLE_1;
            break;
        case (2):
            config |= ADS1015_CONFIG_MUX_SINGLE_2;
            break;
        case (3):
            config |= ADS1015_CONFIG_MUX_SINGLE_3;
            break;
    }

    uint8_t configArray[2];
    configArray[0] = (config >> 8) & 0xFF; // High byte
    configArray[1] = config & 0xFF;        // Low byte

    esp_err_t err = i2cInstance.write(ADS111X_ADDR_GND, ADS1015_POINTER_CONFIG, configArray, 2);

    while (!available(i2cInstance))     // Convert milliseconds to ticks
        vTaskDelay(pdMS_TO_TICKS(1)); // Delay for approximately 0.625 milliseconds

    err = i2cInstance.read(ADS111X_ADDR_GND, ADS1015_POINTER_CONVERT, data, 2);
    
    return err;
}

int16_t convertUnsignedToSigned(uint16_t value)
{
    if (value & 0x8000)
    {
        return (int16_t)(value - 0x10000);
    }
    return (int16_t)value;
}

// Returns the sensor channel single-ended input as int16_t (two's complement)
int16_t getSingleEndedSigned(I2C& i2cInstance, uint8_t channel)
{
    u_int8_t data[2];
    esp_err_t err = getSingleEnded(i2cInstance, channel, data);
    if (err) {
        printf("Error reading data: %s\n", esp_err_to_name(err));
        return 0;
    }

    uint16_t rawValue = (data[0] << 8) | data[1];
    uint16_t result = rawValue >> 4;

    return (convertUnsignedToSigned(result)); // Convert without ambiguity
}


// esp_err_t checkForDevice(I2C& i2cInstance, uint8_t *data) {
    
//     uint8_t data[2];
//     for (int addr = ADS111X_ADDR_GND; addr <= ADS111X_ADDR_SCL; ++addr) {
//         esp_err_t err = i2cInstance.read(addr, REG_CONVERSION, data, 2);
//         if (err) {
//             return err;
//         }
//     }

//     return ESP_OK;
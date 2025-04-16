
/*******************************************************************************
 * @file ADS1015.cpp
 * @brief Contains the definitions of the ADS1015 class methods.
 *
 * This device is a four channel ADC with 12-bit resolution and a programmable
 * gain amplifier. The device can be configured to use one of four I2C addresses.
 * 
 * @version v0.0.1
 * @date 2025-01-16
 * @authord daniel@sense-ai.co, Sense AI
 *******************************************************************************/

#include "ICM42605.hpp"
#include "esp_log.h"

/* ICM20689 object, input the I2C bus and address */
ICM42605::ICM42605(I2C &bus,uint8_t address){
    _i2c = &bus; // I2C bus
    _address = address; // I2C address
    _useSPI = false; // set to use I2C
}

/* ICM20689 object, input the SPI bus and chip select pin */
ICM42605::ICM42605(SPI &bus,uint8_t csPin){
    _spi = &bus; // SPI bus
    _csPin = csPin; // chip select pin
    _useSPI = true; // set to use SPI
}

int ICM42605::begin(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR)
{
    aRes = getAres(Ascale);
    gRes = getGres(Gscale);

    esp_err_t err = ESP_OK;

    if (_useSPI) { // using SPI for communication
        // use low speed SPI for register setting
        _useSPIHS = false;
        // setting CS pin to output
        gpio_set_direction((gpio_num_t)_csPin, GPIO_MODE_OUTPUT);
        // setting CS pin high
        gpio_set_level((gpio_num_t)_csPin, 1);
        // begin SPI communication
        err = whoAmI();
        if (err != ESP_OK) {
            printf("SPI initialization failed: %s\n", esp_err_to_name(err));
            return -1;
        }
    } else { // using I2C for communication
        // starting the I2C bus
        esp_err_t err = _i2c->init();
        if (err != ESP_OK) {
            printf("I2C initialization failed: %s\n", esp_err_to_name(err));
            return -1;
        }
        // setting the I2C clock
        // _i2c->setClock(_i2cRate); this method is not currently exposed
    }

    reset();

    // check the WHO AM I byte, expected value is 0x42 (decimal 66)
    if (whoAmI() != 66) {
        return -1;
    }
    // enable gyro and accel in low noise mode
    if (writeRegister(Register::PWR_MGMT0, 0x0F) < 0) {
        return -2;
    }
    // gyro full scale and data rate
    if (writeRegister(Register::GYRO_CONFIG0, GODR | Gscale << 5) < 0) {
        return -3;
    }
    // set accel full scale and data rate
    if (writeRegister(Register::ACCEL_CONFIG0, AODR | Ascale << 5) < 0) {
        return -4;
    }
    // set temperature sensor low pass filter to 5Hz, use first order gyro filter
    if (writeRegister(Register::GYRO_CONFIG1, 0xD0) < 0) {
        return -5;
    }

    offsetBias(accelBias, gyroBias);

    return 1;
}

uint8_t ICM42605::getChipID() {
    uint8_t chipID;
    readRegisters(Register::WHO_AM_I, 15, &chipID);
    return chipID;
}

void ICM42605::offsetBias(float * dest1, float * dest2)
{
    int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
    int32_t sum[7] = {0, 0, 0, 0, 0, 0, 0};

    printf("Calculate accel and gyro offset biases: keep sensor flat and motionless!\n");
    vTaskDelay(10);

    for (int ii = 0; ii < 128; ii++)
    {
    readSensor(temp);
    sum[1] += temp[1];
    sum[2] += temp[2];
    sum[3] += temp[3];
    sum[4] += temp[4];
    sum[5] += temp[5];
    sum[6] += temp[6];
    vTaskDelay(50);
    }

    dest1[0] = sum[1] * _aRes / 128.0f;
    dest1[1] = sum[2] * _aRes / 128.0f;
    dest1[2] = sum[3] * _aRes / 128.0f;
    dest2[0] = sum[4] * _gRes / 128.0f;
    dest2[1] = sum[5] * _gRes / 128.0f;
    dest2[2] = sum[6] * _gRes / 128.0f;

    if (dest1[0] > 0.8f)  {
    dest1[0] -= 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
    }
    if (dest1[0] < -0.8f) {
    dest1[0] += 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
    }
    if (dest1[1] > 0.8f)  {
    dest1[1] -= 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
    }
    if (dest1[1] < -0.8f) {
    dest1[1] += 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
    }
    if (dest1[2] > 0.8f)  {
    dest1[2] -= 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
    }
    if (dest1[2] < -0.8f) {
    dest1[2] += 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
    }

}

int ICM42605::readSensor(int16_t * destination)
{
    _useSPIHS = true; // use the high speed SPI for data readout
    if (readRegisters(Register::TEMP_DATA1, 15, _buffer) < 0) {
    return -1;
    }
    // Turn the MSB and LSB into a signed 16-bit value
    destination[0] = ((int16_t)_buffer[0] << 8) | _buffer[1] ; //Temperature
    destination[1] = ((int16_t)_buffer[2] << 8) | _buffer[3] ; //Accel X
    destination[2] = ((int16_t)_buffer[4] << 8) | _buffer[5] ; //Accel Y
    destination[3] = ((int16_t)_buffer[6] << 8) | _buffer[7] ; //Accel Z
    destination[4] = ((int16_t)_buffer[8] << 8) | _buffer[9] ; //Gyro X
    destination[5] = ((int16_t)_buffer[10] << 8) | _buffer[11] ; //Gyro Y
    destination[6] = ((int16_t)_buffer[12] << 8) | _buffer[13] ; //Gyro Z

    _t = ((float)destination[0] / 132.48) + 25; // (TEMP_DATA / 132.48) + 25 

// Now we'll calculate the accleration value into actual g's
    _acc[0] = (float)destination[1]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
    _acc[1] = (float)destination[2]*aRes - accelBias[1];   
    _acc[2] = (float)destination[3]*aRes - accelBias[2];  

// Calculate the gyro value into actual degrees per second
    _gyro[0] = (float)destination[4]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
    _gyro[1] = (float)destination[5]*gRes - gyroBias[1];  
    _gyro[2] = (float)destination[6]*gRes - gyroBias[2]; 

    return 1;
}

/* returns the accelerometer measurement in the x direction, m/s/s */
double ICM42605::getAccelX() {
    return _acc[0];
}

/* returns the accelerometer measurement in the y direction, m/s/s */
double ICM42605::getAccelY() {
    return _acc[1];
}

/* returns the accelerometer measurement in the z direction, m/s/s */
double ICM42605::getAccelZ() {
    return _acc[2];
}

/* returns the gyroscope measurement in the x direction, rad/s */
double ICM42605::getGyroX() {
    return _gyro[0];
}

/* returns the gyroscope measurement in the y direction, rad/s */
double ICM42605::getGyroY() {
    return _gyro[1];
}

/* returns the gyroscope measurement in the z direction, rad/s */
double ICM42605::getGyroZ() {
    return _gyro[2];
}

/* returns the die temperature, C */
double ICM42605::getTemperature_C() {
    return _t;
}

float ICM42605::getAres(uint8_t Ascale) {
    switch (Ascale)
    {
        // Possible accelerometer scales (and their register bit settings) are:
        // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        case RegistersBank2::AFS_2G:
            _aRes = 2.0f / 32768.0f;
            return _aRes;
            break;
        case RegistersBank2::AFS_4G:
            _aRes = 4.0f / 32768.0f;
            return _aRes;
            break;
        case RegistersBank2::AFS_8G:
            _aRes = 8.0f / 32768.0f;
            return _aRes;
            break;
        case RegistersBank2::AFS_16G:
            _aRes = 16.0f / 32768.0f;
            return _aRes;
            break;
        }
    return -1;
}

float ICM42605::getGres(uint8_t Gscale) {
    switch (Gscale)
    {
    case RegistersBank2::GFS_15_125DPS:
        _gRes = 15.125f / 32768.0f;
        return _gRes;
        break;
    case RegistersBank2::GFS_31_25DPS:
        _gRes = 31.25f / 32768.0f;
        return _gRes;
        break;
    case RegistersBank2::GFS_62_5DPS:
        _gRes = 62.5f / 32768.0f;
        return _gRes;
        break;
    case RegistersBank2::GFS_125DPS:
        _gRes = 125.0f / 32768.0f;
        return _gRes;
        break;
    case RegistersBank2::GFS_250DPS:
        _gRes = 250.0f / 32768.0f;
        return _gRes;
        break;
    case RegistersBank2::GFS_500DPS:
        _gRes = 500.0f / 32768.0f;
        return _gRes;
        break;
    case RegistersBank2::GFS_1000DPS:
        _gRes = 1000.0f / 32768.0f;
        return _gRes;
        break;
    case RegistersBank2::GFS_2000DPS:
        _gRes = 2000.0f / 32768.0f;
        return _gRes;
        break;
    }
    return -1;
}

void ICM42605::reset()
{
    // reset device
    writeRegister(Register::DEVICE_CONFIG,0x01); // reset the ICM42605
    vTaskDelay(1); // wait for ICM42605 to come back up
}

uint8_t ICM42605::status()
{
    // reset device
    uint8_t temp;
    readRegisters(Register::ADDRESS, Register::INT_STATUS, &temp);
    return temp;
}

// Write a register to the ICM20689
int ICM42605::writeRegister(uint8_t subAddress, uint8_t data) {
    printf("Writing to Address: 0x%02X\n", subAddress);
    if (_useSPI) {
        uint8_t tx_data[2] = {subAddress, data};
        esp_err_t err = _spi->write(tx_data, 2);
        if (err != ESP_OK) {
            printf("SPI write failed: %s\n", esp_err_to_name(err));
            return -1;
        }
        printf("Data sent: 0x%02X\n", data);
    } else {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, subAddress, true);
        i2c_master_write_byte(cmd, data, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);
        if (ret != ESP_OK) {
            printf("I2C write failed: %s\n", esp_err_to_name(ret));
            return -1;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    readRegisters(subAddress, 1, _buffer);
    printf("Data Received: 0x%02X\n", _buffer[0]);

    if (_buffer[0] == data) {
        return 1;
    } else {
        return -1;
    }
}

// Read a register from the ICM20689
int ICM42605::readRegisters(uint8_t subAddress, uint8_t count, uint8_t *dest) {
    if (_useSPI) {
        uint8_t tx_data = subAddress | SPI_READ;
        esp_err_t err = _spi->transfer(&tx_data, dest, count);
        if (err != ESP_OK) {
            printf("SPI read failed: %s\n", esp_err_to_name(err));
            return -1;
        }
        return 1;
    } else {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, subAddress, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, dest, count, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);
        if (ret != ESP_OK) {
            printf("I2C read failed: %s\n", esp_err_to_name(ret));
            return -1;
        }
        return 1;
    }
}

int ICM42605::whoAmI() {
    // read the WHO AM I register
    if (readRegisters(ICM42605::Register::WHO_AM_I, 1, _buffer) < 0) {
        return -1;
    }
    // return the register value
    printf("Who Am I?: \n");
    int count = sizeof(_buffer) / sizeof(_buffer[0]);
    for (int i = 0; i < count; i++) {
        printf("%02X", _buffer[i]);
    }
    printf(" - First char: %02X\n\n", _buffer[0]);
    return _buffer[0];
}
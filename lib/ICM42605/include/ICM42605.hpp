/*******************************************************************************
 * @file ADS1015.hpp
 * @brief Contains the declarations of the ADS1015 class methods.
 *
 * This device is a four channel ADC with 12-bit resolution and a programmable
 * gain amplifier. The device can be configured to use one of four I2C addresses.
 * 
 * @version v0.0.1
 * @date 2025-01-16
 * @authord daniel@sense-ai.co, Sense AI
 *******************************************************************************/

#pragma once
#include "esp_err.h"
#include <cstdint>
#include "smart_sensor_sense.hpp"


class ICM42605 {
public:
    enum Register : uint8_t {
        WHO_AM_I = 0x75,
        DEVICE_CONFIG = 0x11,
        PWR_MGMT0 = 0x4E,
        ACCEL_CONFIG0 = 0x50,
        GYRO_CONFIG0 = 0x4F,
        ACCEL_DATA_X1 = 0x1F,
        ACCEL_DATA_X0 = 0x20,
        GYRO_DATA_X1 = 0x25,
        GYRO_DATA_X0 = 0x26,
        DRIVE_CONFIG = 0x13,
        INT_CONFIG = 0x14,
        FIFO_CONFIG = 0x16,
        TEMP_DATA1 = 0x1D,
        TEMP_DATA0 = 0x1E,
        ACCEL_DATA_Y1 = 0x21,
        ACCEL_DATA_Y0 = 0x22,
        ACCEL_DATA_Z1 = 0x23,
        ACCEL_DATA_Z0 = 0x24,
        GYRO_DATA_Y1 = 0x27,
        GYRO_DATA_Y0 = 0x28,
        GYRO_DATA_Z1 = 0x29,
        GYRO_DATA_Z0 = 0x2A,
        TMST_FSYNCH = 0x2B,
        TMST_FSYNCL = 0x2C,
        INT_STATUS = 0x2D,
        FIFO_COUNTH = 0x2E,
        FIFO_COUNTL = 0x2F,
        FIFO_DATA = 0x30,
        APEX_DATA0 = 0x31,
        APEX_DATA1 = 0x32,
        APEX_DATA2 = 0x33,
        APEX_DATA3 = 0x34,
        APEX_DATA4 = 0x35,
        APEX_DATA5 = 0x36,
        INT_STATUS2 = 0x37,
        INT_STATUS3 = 0x38,
        SIGNAL_PATH_RESET = 0x4B,
        INTF_CONFIG0 = 0x4C,
        INTF_CONFIG1 = 0x4D,
        GYRO_CONFIG1 = 0x51,
        GYRO_ACCEL_CONFIG0 = 0x52,
        ACCEL_CONFIG1 = 0x53,
        TMST_CONFIG = 0x54,
        APEX_CONFIG0 = 0x56,
        SMD_CONFIG = 0x57,
        FIFO_CONFIG1 = 0x5F,
        FIFO_CONFIG2 = 0x60,
        FIFO_CONFIG3 = 0x61,
        FSYNC_CONFIG = 0x62,
        INT_CONFIG0 = 0x63,
        INT_CONFIG1 = 0x64,
        INT_SOURCE0 = 0x65,
        INT_SOURCE1 = 0x66,
        INT_SOURCE3 = 0x68,
        INT_SOURCE4 = 0x69,
        FIFO_LOST_PKT0 = 0x6C,
        FIFO_LOST_PKT1 = 0x6D,
        SELF_TEST_CONFIG = 0x70,
        REG_BANK_SEL = 0x76,
        SENSOR_CONFIG0 = 0x03,
        GYRO_CONFIG_STATIC2 = 0x0B,
        GYRO_CONFIG_STATIC3 = 0x0C,
        GYRO_CONFIG_STATIC4 = 0x0D,
        GYRO_CONFIG_STATIC5 = 0x0E,
        GYRO_CONFIG_STATIC6 = 0x0F,
        GYRO_CONFIG_STATIC7 = 0x10,
        GYRO_CONFIG_STATIC8 = 0x11,
        GYRO_CONFIG_STATIC9 = 0x12,
        GYRO_CONFIG_STATIC10 = 0x13,
        XG_ST_DATA = 0x5F,
        YG_ST_DATA = 0x60,
        ZG_ST_DATA = 0x61,
        TMSTVAL0 = 0x62,
        TMSTVAL1 = 0x63,
        TMSTVAL2 = 0x64,
        INTF_CONFIG4 = 0x7A,
        INTF_CONFIG5 = 0x7B,
        INTF_CONFIG6 = 0x7C,
        ACCEL_CONFIG_STATIC2 = 0x03,
        ACCEL_CONFIG_STATIC3 = 0x04,
        ACCEL_CONFIG_STATIC4 = 0x05,
        XA_ST_DATA = 0x3B,
        YA_ST_DATA = 0x3C,
        ZA_ST_DATA = 0x3D,
        GYRO_ON_OFF_CONFIG = 0x0E,
        APEX_CONFIG1 = 0x40,
        APEX_CONFIG2 = 0x41,
        APEX_CONFIG3 = 0x42,
        APEX_CONFIG4 = 0x43,
        APEX_CONFIG5 = 0x44,
        APEX_CONFIG6 = 0x45,
        APEX_CONFIG7 = 0x46,
        APEX_CONFIG8 = 0x47,
        APEX_CONFIG9 = 0x48,
        ACCEL_WOM_X_THR = 0x4A,
        ACCEL_WOM_Y_THR = 0x4B,
        ACCEL_WOM_Z_THR = 0x4C,
        INT_SOURCE6 = 0x4D,
        INT_SOURCE7 = 0x4E,
        INT_SOURCE8 = 0x4F,
        INT_SOURCE9 = 0x50,
        INT_SOURCE10 = 0x51,
        OFFSET_USER0 = 0x77,
        OFFSET_USER1 = 0x78,
        OFFSET_USER2 = 0x79,
        OFFSET_USER3 = 0x7A,
        OFFSET_USER4 = 0x7B,
        OFFSET_USER5 = 0x7C,
        OFFSET_USER6 = 0x7D,
        OFFSET_USER7 = 0x7E,
        OFFSET_USER8 = 0x7F,
        ADDRESS = 0x69
    };

    enum RegistersBank2 : uint8_t {
        AFS_2G = 0x03,
        AFS_4G = 0x02,
        AFS_8G = 0x01,
        AFS_16G = 0x00, // default

        GFS_2000DPS = 0x00, // default
        GFS_1000DPS = 0x01,
        GFS_500DPS = 0x02,
        GFS_250DPS = 0x03,
        GFS_125DPS = 0x04,
        GFS_62_5DPS = 0x05,
        GFS_31_25DPS = 0x06,
        GFS_15_125DPS = 0x07,

        AODR_8000Hz = 0x03,
        AODR_4000Hz = 0x04,
        AODR_2000Hz = 0x05,
        AODR_1000Hz = 0x06, // default
        AODR_200Hz = 0x07,
        AODR_100Hz = 0x08,
        AODR_50Hz = 0x09,
        AODR_25Hz = 0x0A,
        AODR_12_5Hz = 0x0B,
        AODR_6_25Hz = 0x0C,
        AODR_3_125Hz = 0x0D,
        AODR_1_5625Hz = 0x0E,
        AODR_500Hz = 0x0F,

        GODR_8000Hz = 0x03,
        GODR_4000Hz = 0x04,
        GODR_2000Hz = 0x05,
        GODR_1000Hz = 0x06, // default
        GODR_200Hz = 0x07,
        GODR_100Hz = 0x08,
        GODR_50Hz = 0x09,
        GODR_25Hz = 0x0A,
        GODR_12_5Hz = 0x0B,
        GODR_500Hz = 0x0F
    };

    ICM42605(I2C &bus, uint8_t address);
    ICM42605(SPI &bus, uint8_t csPin);
    float getAres(uint8_t Ascale);
    float getGres(uint8_t Gscale);
    uint8_t getChipID();
    int begin(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR);
    void offsetBias(float *dest1, float *dest2);
    void reset();
    // void selfTest();
    int readSensor(int16_t *destination);
    uint8_t status();
    double getAccelX();
    double getAccelY();
    double getAccelZ();
    double getGyroX();
    double getGyroY();
    double getGyroZ();
    double getTemperature_C();

private:
    float _aRes, _gRes;
    uint8_t _buffer[15] = {};
    uint8_t _address = 0;
    I2C *_i2c = {};
    const uint32_t _i2cRate = 400000; // 400 kHz
    size_t _numBytes = 0; // number of bytes received from I2C
    SPI *_spi = {};
    uint8_t _csPin = 0;
    bool _useSPI = false;
    bool _useSPIHS = false;
    const uint8_t SPI_READ = 0x80;
    const uint32_t SPI_LS_CLOCK = 400000; // 1 MHz
    const uint32_t SPI_HS_CLOCK = 1000000; // 7 MHz
    int16_t _accCounts[3] = {};
    int16_t _gyroCounts[3] = {};
    int16_t _tcounts = 0;
    double _acc[3] = {};
    double _gyro[3] = {};
    double _t = 0.0;
    float aRes, gRes; // scale resolutions per LSB for the accel and gyro sensor2
    float accelBias[3] = {0., 0., 0.}, gyroBias[3] = {0., 0., 0.}; // offset biases for the accel and gyro

    int writeRegister(uint8_t subAddress, uint8_t data);
    int readRegisters(uint8_t subAddress, uint8_t count, uint8_t *dest);
    int whoAmI();
};
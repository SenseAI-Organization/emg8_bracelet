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

#include <bitset>
#include <iostream>

#include "smart_sensor_sense.hpp"

/**
 * @class ADS1015
 * @brief Class for the ADS1015 ADC.
 */

class ADS1015 : public Sensor {
public:

    /**
     * @enum ADS111X_Address
     * @brief Enumeration for the device I2C addresses.
     */
    enum ADS111X_Address {
        ADS111X_ADDR_GND = 0x48, //!< I2C device address with ADDR pin connected to ground
        ADS111X_ADDR_VCC = 0x49, //!< I2C device address with ADDR pin connected to VCC
        ADS111X_ADDR_SDA = 0x4a, //!< I2C device address with ADDR pin connected to SDA
        ADS111X_ADDR_SCL = 0x4b  //!< I2C device address with ADDR pin connected to SCL
    };
    
    /**
     * @enum ConfigMode
     * @brief Enumeration for the device operating mode.
     */
    enum class ConfigOS : uint16_t {
        No       = 0x0000,
        Single   = 0x8000,
        NotReady = 0x0000,
        Ready    = 0x8000
    };

    /**
     * @enum ConfigMode
     * @brief Enumeration for the device operating mode.
     */
    enum class ConfigMode : uint16_t {
        Continuous  = 0x0000,
        Single      = 0x0100
    };

    /**
     * @enum ConfigMux
     * @brief Enumeration for the input multiplexer configuration.
     */
    enum class ConfigMux : uint16_t {
        Single_0    = 0x4000,
        Single_1    = 0x5000,
        Single_2    = 0x6000,
        Single_3    = 0x7000,
        Diff_P0_N1  = 0x0000,
        Diff_P0_N3  = 0x1000,
        Diff_P1_N3  = 0x2000,
        Diff_P2_N3  = 0x3000
    };

    /**
     * @enum ConfigRate
     * @brief Enumeration for the data rate.
     */
    enum class ConfigRate : uint16_t {
        Rate_128Hz  = 0x0000,
        Rate_250Hz  = 0x0020,
        Rate_490Hz  = 0x0040,
        Rate_920Hz  = 0x0060,
        Rate_1600Hz = 0x0080,
        Rate_2400Hz = 0x00A0,
        Rate_3300Hz = 0x00C0
    };

    /**
     * @enum ConfigPGA
     * @brief Enumeration for the programmable gain amplifier configuration.
     */
    enum class ConfigPGA : uint16_t {
        TwoThirds   = 0x0000,
        One         = 0x0200,
        Two         = 0x0400,
        Four        = 0x0600,
        Eight       = 0x0800,
        Sixteen     = 0x0A00
    };

    /**
     * @enum ConfigComparatorMode
     * @brief Enumeration for the comparator mode.
     */
    enum class ConfigComparatorMode : uint16_t {
        Traditional = 0x0000,
        Window      = 0x0010
    };

    /**
     * @enum ConfigComparatorPolarity
     * @brief Enumeration for the comparator polarity.
     */
    enum class ConfigComparatorPolarity : uint16_t {
        ActiveLow   = 0x0000,
        ActiveHigh  = 0x0008
    };

    /**
     * @enum ConfigComparatorLatching
     * @brief Enumeration for the comparator latching.
     */
    enum class ConfigComparatorLatching : uint16_t {
        NonLatching = 0x0000,
        Latching    = 0x0004
    };

    /**
     * @enum ConfigComparatorQueue
     * @brief Enumeration for the comparator queue.
     */
    enum class ConfigComparatorQueue : uint16_t {
        OneConversion   = 0x0000,
        TwoConversions  = 0x0001,
        FourConversions = 0x0002,
        None            = 0x0003
    };

    /**
     * @enum Register
     * @brief Enumeration for the register addresses.
     */
    enum class Register : uint8_t {
        Conversion  = 0,
        Config      = 1,
        ThreshLow   = 2,
        ThreshHigh  = 3
    };

    /**
     * @enum ConfigMask
     * @brief Enumeration for the configuration masks.
     */
    enum class ConfigMask : uint16_t {
        CompQue  = 0x03,
        CompLat  = 0x01,
        CompPol  = 0x01,
        CompMode = 0x01,
        DataRate = 0x07,
        Mode     = 0x01,
        PGA      = 0x07,
        Mux      = 0x07,
        OS       = 0x01
    };

    /**
     * @enum ConfigOffset
     * @brief Enumeration for the configuration offsets.
     */
    enum class ConfigOffset : uint8_t {
        CompQue  = 0,
        CompLat  = 2,
        CompPol  = 3,
        CompMode = 4,
        DataRate = 5,
        Mode     = 8,
        PGA      = 9,
        Mux      = 12,
        OS       = 15
    };
    
   /**
 * @brief Constructor for ADS1015.
 * @param i2cInstance Reference to the I2C instance.
 * @param address I2C address of the device.
 */
ADS1015(I2C& i2cInstance, uint8_t address = ADS111X_Address::ADS111X_ADDR_GND);

/**
 * @brief Destructor for ADS1015.
 */
~ADS1015();

/**
 * @brief Initialize the ADS1015 device.
 * @return esp_err_t Error code indicating success or failure.
 */
esp_err_t init(void) override;

/**
 * @brief Initialize the ADS1015 device with specific gain and sample frequency.
 * @param gain Gain setting for the ADC.
 * @param sampleFrequency Sample frequency setting for the ADC.
 * @return esp_err_t Error code indicating success or failure.
 */
esp_err_t init(uint8_t gain, uint8_t sampleFrequency);

/**
 * @brief Perform a measurement on all channels.
 * @return esp_err_t Error code indicating success or failure.
 */
esp_err_t measure(void) override;

/**
 * @brief Perform a measurement on a specific channel.
 * @param channel Channel to measure.
 * @return esp_err_t Error code indicating success or failure.
 */
esp_err_t measure(uint8_t channel);

/**
 * @brief Get the ID of the ADS1015 device.
 * @return uint8_t Device ID.
 */
uint8_t getID(void) const override;

/**
 * @brief Get a report of the ADS1015 device status.
 * @param _buff Buffer to store the report.
 * @return esp_err_t Error code indicating success or failure.
 */
esp_err_t getReport(char* _buff) const override;

/**
 * @brief Get the measurement value of a specific channel.
 * @param channel Channel to get the measurement from.
 * @return int16_t Measurement value.
 */
int16_t getChannel(uint8_t channel);

/**
 * @brief Set the configuration of the ADS1015 device.
 * @param channel Channel to configure.
 * @param gain Gain setting for the ADC.
 * @param sampleFrequency Sample frequency setting for the ADC.
 * @return esp_err_t Error code indicating success or failure.
 */
esp_err_t setConfig(uint8_t channel, uint8_t gain, uint8_t sampleFrequency);

/**
 * @brief Check if the ADS1015 device is available.
 * @return esp_err_t Error code indicating success or failure.
 */
esp_err_t checkForDevice();

/**
 * @brief Read a single-ended value from a specific channel.
 * @param channel Channel to read from.
 * @return int8_t Measurement value.
 */
int8_t readSingleEnded(uint8_t channel);

/**
 * @brief Read a single-ended signed value from a specific channel.
 * @param channel Channel to read from.
 * @return int16_t Measurement value.
 */
int16_t readSingleEndedSigned(uint8_t channel);

private:
    I2C& i2c_; ///< Reference to the I2C instance.
    ADS111X_Address address_; ///< I2C address of the device.

    /**
     * @brief Check if a sample is ready.
     * @param sampleReady Pointer to a boolean to store the result.
     * @return esp_err_t Error code indicating success or failure.
     */
    esp_err_t available(bool * sampleReady);

    /**
     * @brief Convert an unsigned value to a signed value.
     * @param value Unsigned value to convert.
     * @return int16_t Signed value.
     */
    int16_t convertUnsignedToSigned(uint16_t value);

    /**
     * @brief Read configuration bits from the ADS1015 device.
     * @param data Pointer to store the configuration bits.
     * @return esp_err_t Error code indicating success or failure.
     */
    esp_err_t readConfBits(uint8_t * data);

    /**
     * @brief Set the sample frequency in the configuration.
     * @param config Current configuration.
     * @param sampleFrequency Sample frequency to set.
     * @return uint16_t Updated configuration.
     */
    uint16_t setSampleFrequency(uint16_t config, uint8_t sampleFrequency);

    /**
     * @brief Set the gain in the configuration.
     * @param config Current configuration.
     * @param gain Gain to set.
     * @return uint16_t Updated configuration.
     */
    uint16_t setGain(uint16_t config, uint8_t gain);

    /**
     * @brief Set the channel in the configuration.
     * @param config Current configuration.
     * @param channel Channel to set.
     * @return uint16_t Updated configuration.
     */
    uint16_t setChannel(uint16_t config, uint8_t channel);

    uint8_t gain_ = 1; ///< Gain setting for the ADC.
    uint8_t sampleFrequency_ = 6; ///< Sample frequency setting for the ADC.

    struct adcRawData {
        uint8_t channels[8]; ///< Raw data for the ADC channels.
    }; 
    adcRawData rawData = {{0, 0, 0, 0, 0, 0, 0, 0}};
    
    struct adcData {
        uint16_t channels[4]; ///< Processed data for the ADC channels.
    };
    adcData adcData = {0, 0, 0, 0};
};
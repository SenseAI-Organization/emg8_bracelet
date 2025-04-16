
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

#include "ADS1015.hpp"

ADS1015::ADS1015(I2C& i2cInstance, uint8_t address)
    : i2c_(i2cInstance), address_(static_cast<ADS1015::ADS111X_Address>(address)) {}

ADS1015::~ADS1015() {}

esp_err_t ADS1015::init() {
    // Use i2c_ instead of i2cInstance
    uint8_t data[1] = {0};
    esp_err_t err = i2c_.write(address_, static_cast<uint8_t>(Register::Config), data, 1);
        if (err) {
            return err;
        }
    err = setConfig(0, gain_, sampleFrequency_);
    if (err) {
        return err;
    }
    return ESP_OK;
}

esp_err_t ADS1015::init(uint8_t gain, uint8_t sampleFrequency){
    gain_ = gain;
    sampleFrequency_ = sampleFrequency;
    return init();
}

esp_err_t ADS1015::measure(void) {
    adcData.channels[0] = readSingleEndedSigned(0);
    adcData.channels[1] = readSingleEndedSigned(1);
    adcData.channels[2] = readSingleEndedSigned(2);
    adcData.channels[3] = readSingleEndedSigned(3);
    return ESP_OK;
}

int16_t ADS1015::getChannel(uint8_t channel){
    return adcData.channels[channel];
}

esp_err_t ADS1015::measure(uint8_t channel){
    
    esp_err_t err = setConfig(channel, gain_, sampleFrequency_);
    
    if (err) {
        printf("Error setting config: %s\n", esp_err_to_name(err));
        return err;
    }

    bool sampleReady = false;
    while (!sampleReady)     // Convert milliseconds to ticks
    {   available(&sampleReady);
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }

    err = i2c_.read(address_, static_cast<uint8_t>(Register::Conversion), &rawData.channels[channel * 2], 2);
    return err;
}

uint8_t ADS1015::getID(void) const {
    return address_;
}

esp_err_t ADS1015::getReport(char* _buff) const {
    if (_buff == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    snprintf(_buff, 128, "ID: %X, ch1: %d, ch2: %d, ch3: %d, ch4: %d", getID(),
            adcData.channels[0], adcData.channels[1], adcData.channels[2], adcData.channels[3]);
    printf("%s\n", _buff);

    return ESP_OK;
}

esp_err_t ADS1015::setConfig(uint8_t channel, uint8_t gain, uint8_t sampleFrequency) {
    if (channel > 3) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t config = static_cast<uint16_t>(ConfigOS::Single) |
                      static_cast<uint16_t>(ConfigComparatorQueue::None);

    config = setSampleFrequency(config, sampleFrequency);
    config = setGain(config, gain);
    config |= static_cast<uint16_t>(ConfigMode::Single);
    config = setChannel(config, channel);

    uint8_t configArray[2];
    configArray[0] = (config >> 8) & 0xFF; // High byte
    configArray[1] = config & 0xFF;        // Low byte

    esp_err_t err = i2c_.write(address_, static_cast<uint8_t>(Register::Config), configArray, 2);
    return err;
}

esp_err_t ADS1015::checkForDevice() {
    // Use i2c_ instead of i2cInstance
    uint8_t data[1] = {0};
    for (int addr = ADS111X_ADDR_GND; addr <= ADS111X_ADDR_SCL; ++addr) {
        esp_err_t err = i2c_.write(addr, static_cast<uint8_t>(Register::Config), data, 1);
        if (err) {
            return err;
        }
    }
    //printf("ADS seems to be working\n");
    return ESP_OK;
}

int8_t ADS1015::readSingleEnded(uint8_t channel){
    esp_err_t err = measure(channel);
    if (err) {
        printf("Error reading data: %s\n", esp_err_to_name(err));
        return 0;
    }
    
    uint16_t rawValue = (rawData.channels[channel * 2] << 8) | rawData.channels[channel * 2 + 1];
    uint16_t result = rawValue >> 4;

    return (result); // Convert without ambiguity
}

// Returns the sensor channel single-ended input as int16_t (two's complement)
int16_t ADS1015::readSingleEndedSigned(uint8_t channel)
{
    esp_err_t err = measure(channel);
    if (err) {
        printf("Error reading data: %s\n", esp_err_to_name(err));
        return 0;
    }
    
    uint16_t rawValue = (rawData.channels[channel * 2] << 8) | rawData.channels[channel * 2 + 1];
    uint16_t result = rawValue >> 4;

    return (convertUnsignedToSigned(result)); // Convert without ambiguity
}

esp_err_t ADS1015::available(bool * sampleReady) {
    uint8_t response[2];
    esp_err_t err = i2c_.read(address_, static_cast<uint8_t>(Register::Config), response, 2);
    
    if (err != ESP_OK) {
        return err;
    }

    uint16_t value = (response[0] << 8) | response[1];
    *sampleReady = bool(((value & static_cast<uint16_t>(ConfigOS::Ready)) > 0)); // If the OS bit is 1 : the device is not currently performing a conversion (i.e. data is available)
    return err;
}

int16_t ADS1015::convertUnsignedToSigned(uint16_t value) {
    if (value & 0x8000) {
        return (int16_t)(value - 0x10000);
    }
    return (int16_t)value;
}

esp_err_t ADS1015::readConfBits(uint8_t * data) {
    // Use i2c_ instead of i2cInstance
    esp_err_t err = i2c_.read(address_, static_cast<uint8_t>(Register::Config), data, 2);
    if (err) {
        return err;
    }
    printf("Data: %d %d\n", data[0], data[1]);
    std::bitset<16> bits0(data[0]);
    std::bitset<16> bits1(data[1]);
    std::cout << "Data as bits: " << bits0 << " " << bits1 << std::endl;

    return ESP_OK;
}

uint16_t ADS1015::setSampleFrequency(uint16_t config, uint8_t sampleFrequency) {
    switch (sampleFrequency) {
        case 0:
            return config | static_cast<uint16_t>(ConfigRate::Rate_128Hz);
        case 1:
            return config | static_cast<uint16_t>(ConfigRate::Rate_250Hz);
        case 2:
            return config | static_cast<uint16_t>(ConfigRate::Rate_490Hz);
        case 3:
            return config | static_cast<uint16_t>(ConfigRate::Rate_920Hz);
        case 4:
            return config | static_cast<uint16_t>(ConfigRate::Rate_1600Hz);
        case 5:
            return config | static_cast<uint16_t>(ConfigRate::Rate_2400Hz);
        case 6:
            return config | static_cast<uint16_t>(ConfigRate::Rate_3300Hz);
        default:
            return config;
    }
}

uint16_t ADS1015::setGain(uint16_t config, uint8_t gain) {
    switch (gain) {
        case 0:
            return config | static_cast<uint16_t>(ConfigPGA::TwoThirds);
        case 1:
            return config | static_cast<uint16_t>(ConfigPGA::One);
        case 2:
            return config | static_cast<uint16_t>(ConfigPGA::Two);
        case 3:
            return config | static_cast<uint16_t>(ConfigPGA::Four);
        case 4:
            return config | static_cast<uint16_t>(ConfigPGA::Eight);
        case 5:
            return config | static_cast<uint16_t>(ConfigPGA::Sixteen);
        default:
            return config;
    }
}

uint16_t ADS1015::setChannel(uint16_t config, uint8_t channel) {
    switch (channel) {
        case 0:
            return config | static_cast<uint16_t>(ConfigMux::Single_0);
        case 1:
            return config | static_cast<uint16_t>(ConfigMux::Single_1);
        case 2:
            return config | static_cast<uint16_t>(ConfigMux::Single_2);
        case 3:
            return config | static_cast<uint16_t>(ConfigMux::Single_3);
        default:
            return config;
    }
}

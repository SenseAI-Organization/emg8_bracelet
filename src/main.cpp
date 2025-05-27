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
#include "sd_storage_sense.hpp"
#include "spi_gp_sense.hpp"

// I2C configs
constexpr gpio_num_t kSDA1 = GPIO_NUM_45;
constexpr gpio_num_t kSCL1 = GPIO_NUM_47;
I2C i2c1(I2C_NUM_1, kSDA1, kSCL1, 400000, false);

constexpr gpio_num_t kSDA0 = GPIO_NUM_6;
constexpr gpio_num_t kSCL0 = GPIO_NUM_7;
I2C i2c0(I2C_NUM_0, kSDA0, kSCL0, 400000, false);
//-------------------------------------------------

// SD Card configs
constexpr gpio_num_t kCsPin = GPIO_NUM_10;
constexpr gpio_num_t kSclPin = GPIO_NUM_12;
constexpr gpio_num_t kMosiPin = GPIO_NUM_11;
constexpr gpio_num_t kMisoPin = GPIO_NUM_13;

SPI* spiMaster = nullptr;
SD* sdCard = nullptr;
//------------------------------------------------- 

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

// SD CARD FUNCTIONS
bool initSDCard() {
    printf("\n\nInitializing SD Card...\n");
    
    // Initialize SPI bus
    spiMaster = new SPI(SPI::SpiMode::kMaster, SPI2_HOST, kMosiPin, kMisoPin, kSclPin);
    esp_err_t err = spiMaster->init();
    if (err) {
        printf("SPI error while init: %s\n", esp_err_to_name(err));
        return false;
    }

    // Initialize SD card
    sdCard = new SD(spiMaster, kCsPin);
    err = sdCard->init();
    if (err) {
        printf("SD error while init: %s\n", esp_err_to_name(err));
        return false;
    } else {
        printf("SD was initialized!\n");
    }

    // Mount SD card
    FRESULT error = sdCard->mountCard();
    if (error) {
        printf("SD error while mounting: %s\n", sdCard->getFastFsErrName(error));
        return false;
    } else {
        std::string currentPath = sdCard->getCurrentDir();
        printf("Card mounted, root path: %s\n", currentPath.c_str());
    }

    // Print SD card info
    if (sdCard->sdCardInfo_.is_mem) {
        printf("SD is memory card.\n");
    }
    printf("Max freq speed (kHz): %d\n", (int)sdCard->sdCardInfo_.max_freq_khz);
    printf("Real freq speed (kHz): %d\n", sdCard->sdCardInfo_.real_freq_khz);
    
    return true;
}

bool createAndWriteFile(const std::string& dirPath, const std::string& fileName, const std::string& content) {
    if (!sdCard) {
        printf("SD card not initialized\n");
        return false;
    }
    
    // Create directory structure if it doesn't exist
    std::string currentDir = sdCard->getCurrentDir();
    FRESULT error;
    
    // Check if we need to create/navigate to a directory
    if (!dirPath.empty() && dirPath != "/") {
        // Create directory (this will fail if already exists, but that's OK)
        error = sdCard->createDir(dirPath);
        
        // Navigate to the directory
        error = sdCard->goToDir(sdCard->getCurrentDir() + "/" + dirPath);
        if (error) {
            printf("SD error at goToDir: %s\n", sdCard->getFastFsErrName(error));
            return false;
        } else {
            //printf("Directory changed, current path: %s\n", sdCard->getCurrentDir().c_str());
        }
    }
    
    // Create file
    error = sdCard->createFile(fileName);
    if (error && error != FR_EXIST) {  // Ignore if file already exists
        printf("SD error at createFile: %s\n", sdCard->getFastFsErrName(error));
        return false;
    }
    
    // Open file for writing
    error = sdCard->openFile(fileName, SD::openMode::kOpenAppend);
    if (error) {
        printf("SD error at openFile: %s\n", sdCard->getFastFsErrName(error));
        return false;
    }
    
    // Write to file
    //printf("Writing to file: '%s'\n", content.c_str());
    error = sdCard->fileWrite(content);
    if (error) {
        printf("SD error at fileWrite: %s\n", sdCard->getFastFsErrName(error));
        sdCard->closeFile();
        return false;
    }
    
    // Close file
    error = sdCard->closeFile();
    if (error) {
        printf("SD error at closeFile: %s\n", sdCard->getFastFsErrName(error));
        return false;
    }
    
    //printf("File write completed successfully!\n");
    
    // Return to original directory
    sdCard->goToDir(currentDir);
    
    return true;
}

std::string readFile(const std::string& dirPath, const std::string& fileName) {
    if (!sdCard) {
        printf("SD card not initialized\n");
        return "";
    }
    
    std::string fileContents;
    std::string currentDir = sdCard->getCurrentDir();
    FRESULT error;
    
    // Navigate to directory if needed
    if (!dirPath.empty() && dirPath != "/") {
        error = sdCard->goToDir(dirPath.substr(0, 1) == "/" ? dirPath : currentDir + "/" + dirPath);
        if (error) {
            printf("SD error at goToDir: %s\n", sdCard->getFastFsErrName(error));
            return "";
        }
    }
    
    // Open file for reading
    error = sdCard->openFile(fileName, SD::openMode::kOpenReadOnly);
    if (error) {
        printf("SD error at openFile: %s\n", sdCard->getFastFsErrName(error));
        sdCard->goToDir(currentDir);  // Return to original directory
        return "";
    }
    
    // Read file contents
    error = sdCard->fileRead(fileContents);
    if (error) {
        printf("SD error at fileRead: %s\n", sdCard->getFastFsErrName(error));
        sdCard->closeFile();
        sdCard->goToDir(currentDir);  // Return to original directory
        return "";
    }
    
    // Close file
    sdCard->closeFile();
    
    // Return to original directory
    sdCard->goToDir(currentDir);
    
    return fileContents;
}

void cleanupSDCard() {
    if (sdCard) {
        sdCard->unmountCard();
        printf("Card unmounted\n");
        delete sdCard;
        sdCard = nullptr;
    }
    
    if (spiMaster) {
        delete spiMaster;
        spiMaster = nullptr;
    }
}


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

    // Initialize SD card
    bool sdInitialized = initSDCard();
    if (!sdInitialized) {
        printf("Failed to initialize SD card\n");
    } else {
        // Create a test file and write to it
        std::string testData = "EMG Bracelet Test Data\n";
        if (createAndWriteFile("emg_data", "test.txt", testData)) {
            // Read back the file
            std::string fileContents = readFile("emg_data", "test.txt");
            printf("Read from file: %s\n", fileContents.c_str());
        }
    }

    // Start ADC reading loop
    vTaskDelay(pdMS_TO_TICKS(3000));  // Initial delay to allow setup to complete
    int sampleCount = 0;
    std::string adcDataBuffer;
    
    while (true) {
        for (int i = 0; i < 4; i++) {
            uint16_t value1 = ads1.readSingleEndedSigned(i);
            uint16_t value2 = ads2.readSingleEndedSigned(i);
            uint16_t value3 = ads3.readSingleEndedSigned(i);
            uint16_t value4 = ads4.readSingleEndedSigned(i);
            
            char dataLine[100];
            // Format data for SD card storage
            if (i == 3){
                snprintf(dataLine, sizeof(dataLine), "%d,%d,%d,%d", value1, value2, value3, value4);
            } else {
                // Add a comma after each value except the last one
                snprintf(dataLine, sizeof(dataLine), "%d,%d,%d,%d,", value1, value2, value3, value4);
            }
            adcDataBuffer += dataLine;
        }
        adcDataBuffer += "\n";
        
        // Save data to SD card every 100 samples
        sampleCount++;
        if (sdInitialized && sampleCount >= 9) {
            char filename[32];
            snprintf(filename, sizeof(filename), "emg_data_%lld.csv", esp_timer_get_time() / 1000000); // Use timestamp for filename
            createAndWriteFile("emg_data", filename, adcDataBuffer);
            sampleCount = 0;

            printf(adcDataBuffer.c_str());  // Print data to console
            adcDataBuffer.clear();
        }
        vTaskDelay(pdMS_TO_TICKS(1));  // Delay 1ms
    }
    
    // Cleanup (note: this won't be reached in this loop)
    cleanupSDCard();
}
/**
 * 
 */

#include "SGP30.h"

static const uint8_t CMD_INIT_AIR_QUALITY[2] = {0x20, 0x03};
static const uint8_t CMD_MEASURE_AIR_QUALITY[2] = {0x20, 0x08};
static const uint8_t CMD_GET_BASELINE[2] = {0x20, 0x15};
static const uint8_t CMD_SET_BASELINE[2] = {0x20, 0x1E};
static const uint8_t CMD_SET_HUMIDITY[2] = {0x20, 0x61};
static const uint8_t CMD_MEASURE_TEST[2] = {0x20, 0x32};
static const uint8_t CMD_GET_FEATURESET_VERSION[2] = {0x20, 0x2F};
static const uint8_t CMD_GET_SERIAL[2] = {0x36, 0x82};
static const uint8_t CMD_MEASURE_RAW_SIGNAL[2] = {0x20, 0x50};

static esp_err_t SGP30_InitAirQuality();
static uint8_t _SGP30_getCheckSum(uint16_t data);


/**
 *
 * @param i2cPort
 * @return
 */
esp_err_t SGP30_Init(i2c_port_t i2cPort) {
    _i2cPort = i2cPort;
    CHECK_ERROR(SGP30_GetSerial());
    CHECK_ERROR(SGP30_GetFeatureSet());
    CHECK_ERROR(SGP30_InitAirQuality());
    isInitialized = true;
    return ESP_OK;
}

esp_err_t SGP30_InitWithSemaphore(i2c_port_t i2cPort, SemaphoreHandle_t semaphore) {
    _semaphore = semaphore;
    return SGP30_Init(i2cPort);
}

esp_err_t SGP30_GetAirQuality() {
    if(!isInitialized) return SGP30_NOT_INITIALIZED;
    if(_semaphore != NULL) {
        xSemaphoreTake(_semaphore, portMAX_DELAY);
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    CHECK_ERROR(i2c_master_start(cmd));
    CHECK_ERROR(i2c_master_write_byte(cmd, (SGP30_ADDRESS << 1) | I2C_MASTER_WRITE, true));
    CHECK_ERROR(i2c_master_write(cmd, CMD_MEASURE_AIR_QUALITY, 2, true));

    CHECK_ERROR(i2c_master_stop(cmd)); // P State
    CHECK_ERROR(i2c_master_cmd_begin(_i2cPort, cmd, 1000 / portTICK_RATE_MS));

    vTaskDelay(REAL_MS(24));
    CHECK_ERROR(i2c_master_start(cmd));
    CHECK_ERROR(i2c_master_write_byte(cmd, (SGP30_ADDRESS << 1) | I2C_MASTER_READ, true));
    uint8_t* reply = (uint8_t*)calloc(sizeof(uint8_t), 6);
    CHECK_ERROR(i2c_master_read(cmd, reply, 6, true));

    CHECK_ERROR(i2c_master_stop(cmd)); // P State
    CHECK_ERROR(i2c_master_cmd_begin(_i2cPort, cmd, 1000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);
    if(_semaphore != NULL) {
        xSemaphoreGive(_semaphore);
    }
    return ESP_OK;
}

esp_err_t SGP30_GetBaseline() {
    if(!isInitialized) return SGP30_NOT_INITIALIZED;
    if(_semaphore != NULL) {
        xSemaphoreTake(_semaphore, portMAX_DELAY);
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    CHECK_ERROR(i2c_master_start(cmd));
    CHECK_ERROR(i2c_master_write_byte(cmd, (SGP30_ADDRESS << 1) | I2C_MASTER_WRITE, true));
    CHECK_ERROR(i2c_master_write(cmd, CMD_INIT_AIR_QUALITY, 2, true));

    CHECK_ERROR(i2c_master_stop(cmd)); // P State
    CHECK_ERROR(i2c_master_cmd_begin(_i2cPort, cmd, 1000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);
    if(_semaphore != NULL) {
        xSemaphoreGive(_semaphore);
    }
    return ESP_OK;
}

esp_err_t SGP30_SetBaseline(uint16_t baseTVoC, uint16_t baseECO2) {
    if(!isInitialized) return SGP30_NOT_INITIALIZED;
    if(_semaphore != NULL) {
        xSemaphoreTake(_semaphore, portMAX_DELAY);
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    CHECK_ERROR(i2c_master_start(cmd));
    CHECK_ERROR(i2c_master_write_byte(cmd, (SGP30_ADDRESS << 1) | I2C_MASTER_WRITE, true));
    CHECK_ERROR(i2c_master_write(cmd, CMD_INIT_AIR_QUALITY, 2, true));

    CHECK_ERROR(i2c_master_stop(cmd)); // P State
    CHECK_ERROR(i2c_master_cmd_begin(_i2cPort, cmd, 1000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);
    if(_semaphore != NULL) {
        xSemaphoreGive(_semaphore);
    }
    return ESP_OK;
}

esp_err_t SGP30_SelfTest() {
    if(!isInitialized) return SGP30_NOT_INITIALIZED;
    if(_semaphore != NULL) {
        xSemaphoreTake(_semaphore, portMAX_DELAY);
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    CHECK_ERROR(i2c_master_start(cmd));
    CHECK_ERROR(i2c_master_write_byte(cmd, (SGP30_ADDRESS << 1) | I2C_MASTER_WRITE, true));
    CHECK_ERROR(i2c_master_write(cmd, CMD_INIT_AIR_QUALITY, 2, true));

    CHECK_ERROR(i2c_master_stop(cmd)); // P State
    CHECK_ERROR(i2c_master_cmd_begin(_i2cPort, cmd, 1000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);
    if(_semaphore != NULL) {
        xSemaphoreGive(_semaphore);
    }
    return ESP_OK;
}

esp_err_t SGP30_Reset() {
    if(_semaphore != NULL) {
        xSemaphoreTake(_semaphore, portMAX_DELAY);
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    CHECK_ERROR(i2c_master_start(cmd));
    CHECK_ERROR(i2c_master_write_byte(cmd, (00 << 1) | I2C_MASTER_WRITE, true));
    CHECK_ERROR(i2c_master_write_byte(cmd, 0x06, true));

    CHECK_ERROR(i2c_master_stop(cmd)); // P State
    CHECK_ERROR(i2c_master_cmd_begin(_i2cPort, cmd, 1000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);
    if(_semaphore != NULL) {
        xSemaphoreGive(_semaphore);
    }
    isInitialized = false;
    return ESP_OK;
}

esp_err_t SGP30_GetSerial() {
    if(_semaphore != NULL) {
        xSemaphoreTake(_semaphore, portMAX_DELAY);
    }
    I2C_MAKE_LINK;
    I2C_CHECK_ERROR(i2c_master_start(cmd));
    CHECK_ERROR(i2c_master_write_byte(cmd, (SGP30_ADDRESS << 1) | I2C_MASTER_WRITE, true));
    CHECK_ERROR(i2c_master_write(cmd, CMD_GET_SERIAL, 2, true));

    CHECK_ERROR(i2c_master_cmd_begin(_i2cPort, cmd, 1000 / portTICK_RATE_MS));

    vTaskDelay(REAL_MS(1));

    CHECK_ERROR(i2c_master_write_byte(cmd, (SGP30_ADDRESS << 1) | I2C_MASTER_READ, true));
    uint8_t* replies = (uint8_t*)calloc(sizeof(uint8_t), 9);
    CHECK_ERROR(i2c_master_read(cmd, replies, 9, true));

    CHECK_ERROR(i2c_master_stop(cmd)); // P State

    CHECK_ERROR(i2c_master_cmd_begin(_i2cPort, cmd, 1000 / portTICK_RATE_MS));

    i2c_cmd_link_delete(cmd);
    if(_semaphore != NULL) {
        xSemaphoreGive(_semaphore);
    }
    return ESP_OK;
}

esp_err_t SGP30_GetFeatureSet() {
    if(_semaphore != NULL) {
        xSemaphoreTake(_semaphore, portMAX_DELAY);
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    CHECK_ERROR(i2c_master_start(cmd));
    CHECK_ERROR(i2c_master_write_byte(cmd, (SGP30_ADDRESS << 1) | I2C_MASTER_WRITE, true));
    CHECK_ERROR(i2c_master_write(cmd, CMD_INIT_AIR_QUALITY, 2, true));

    CHECK_ERROR(i2c_master_stop(cmd)); // P State
    CHECK_ERROR(i2c_master_cmd_begin(_i2cPort, cmd, 1000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);
    if(_semaphore != NULL) {
        xSemaphoreGive(_semaphore);
    }
    return ESP_OK;
}

esp_err_t SGP30_SetHumidity(double relativeHumidity) {
    if(!isInitialized) return SGP30_NOT_INITIALIZED;
    if(_semaphore != NULL) {
        xSemaphoreTake(_semaphore, portMAX_DELAY);
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    CHECK_ERROR(i2c_master_start(cmd));
    CHECK_ERROR(i2c_master_write_byte(cmd, (SGP30_ADDRESS << 1) | I2C_MASTER_WRITE, true));
    CHECK_ERROR(i2c_master_write(cmd, CMD_SET_HUMIDITY, 2, true));

    CHECK_ERROR(i2c_master_stop(cmd)); // P State
    CHECK_ERROR(i2c_master_cmd_begin(_i2cPort, cmd, 1000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);
    if(_semaphore != NULL) {
        xSemaphoreGive(_semaphore);
    }
    return ESP_OK;
}

esp_err_t SGP30_MeasureRawSignal() {
    if(!isInitialized) return SGP30_NOT_INITIALIZED;
    if(_semaphore != NULL) {
        xSemaphoreTake(_semaphore, portMAX_DELAY);
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    CHECK_ERROR(i2c_master_start(cmd));
    CHECK_ERROR(i2c_master_write_byte(cmd, (SGP30_ADDRESS << 1) | I2C_MASTER_WRITE, true));
    CHECK_ERROR(i2c_master_write(cmd, CMD_MEASURE_RAW_SIGNAL, 2, true));

    CHECK_ERROR(i2c_master_stop(cmd)); // P State
    CHECK_ERROR(i2c_master_cmd_begin(_i2cPort, cmd, 1000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);
    if(_semaphore != NULL) {
        xSemaphoreGive(_semaphore);
    }
    return ESP_OK;
}

esp_err_t SGP30_InitAirQuality() {
    if(_semaphore != NULL) {
        xSemaphoreTake(_semaphore, portMAX_DELAY);
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    CHECK_ERROR(i2c_master_start(cmd));
    CHECK_ERROR(i2c_master_write_byte(cmd, (SGP30_ADDRESS << 1) | I2C_MASTER_WRITE, true));
    CHECK_ERROR(i2c_master_write(cmd, CMD_INIT_AIR_QUALITY, 2, true));

    CHECK_ERROR(i2c_master_stop(cmd)); // P State
    CHECK_ERROR(i2c_master_cmd_begin(_i2cPort, cmd, 1000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);
    if(_semaphore != NULL) {
        xSemaphoreGive(_semaphore);
    }
    return ESP_OK;
}

/**
 * Credit by Sparkfun
 * @param data
 * @return
 */
uint8_t _SGP30_getCheckSum(uint16_t data) {
    uint8_t crc = 0xFF; //Init with 0xFF

    crc ^= (data >> 8); // XOR-in the first input byte

    for (uint8_t i = 0 ; i < 8 ; i++) {
        if ((crc & 0x80) != 0)
            crc = (uint8_t)((crc << 1) ^ 0x31);
        else
            crc <<= 1;
    }
    crc ^= (uint8_t)data; // XOR-in the last input byte

    for (uint8_t i = 0 ; i < 8 ; i++) {
        if ((crc & 0x80) != 0)
            crc = (uint8_t)((crc << 1) ^ 0x31);
        else
            crc <<= 1;
    }

    return crc; //No output reflection
}
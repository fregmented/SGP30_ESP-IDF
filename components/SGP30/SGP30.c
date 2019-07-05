//
// Created by hanwool on 2018-10-15.
//

#include "SGP30.h"

static const char* TAG = "SGP30";
static sgp30_param_t spg30_param;

static esp_err_t sgp30_SendCommand(uint16_t cmd);
static esp_err_t sgp30_SendCommandWithArgument(uint16_t cmd, uint16_t* args, uint8_t argCount);
static esp_err_t sgp30_ReadFromI2C(uint8_t* data, uint8_t length);
static uint8_t sgp30_CalcChecksum(uint16_t data);
static uint16_t sgp30_doubleToFixedPoint(double number);

esp_err_t sgp30_Init(sgp30_param_t params) {
    if(params.using_library_i2c) {
        if(params.scl == SGP30_HAL_UNDEFINED ||
           params.sda == SGP30_HAL_UNDEFINED ||
                params.i2c_port == I2C_NUM_MAX) {
            return ESP_ERR_INVALID_ARG;
        }
        if(params.i2c_freq_hz > SGP30_MAX_I2C_MASTER_FREQ_HZ) {
            ESP_LOGE(TAG, "SCD30's maximum i2c frequency is %dkHz not %dkHz", SGP30_MAX_I2C_MASTER_FREQ_HZ/1000, params.i2c_freq_hz/1000);
            return ESP_ERR_INVALID_ARG;
        }
        i2c_config_t conf;
        conf.mode = I2C_MODE_MASTER;
        ESP_LOGD(TAG, "sda_io_num %d", params.sda);
        conf.sda_io_num = params.sda;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        ESP_LOGD(TAG, "scl_io_num %d", params.scl);
        conf.scl_io_num = params.scl;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        ESP_LOGD(TAG, "clk_speed %d", params.i2c_freq_hz);
        conf.master.clk_speed = params.i2c_freq_hz;
        ESP_LOGD(TAG, "i2c_param_config %d", conf.mode);
        ESP_ERROR_CHECK(i2c_param_config(params.i2c_port, &conf));
        ESP_LOGD(TAG, "i2c_driver_install %d", I2C_NUM_1);
        ESP_ERROR_CHECK(i2c_driver_install(params.i2c_port, conf.mode, 0, 0, 0));
    }
    spg30_param = params;

    sgp30_Data.tVoC = -1;
    sgp30_Data.eCO2 = -1;
    sgp30_Data.H2 = -1;
    sgp30_Data.Ethanol = -1;
    sgp30_Data.tVoC_baseline = -1;
    sgp30_Data.eCO2_baseline = -1;
    sgp30_Data.featureSetVer = -1;
    sgp30_Data.serialID = 0;

    esp_err_t err = ESP_OK;

    if((err = sgp30_GetSerialID()) != ESP_OK) {
        ESP_LOGE(TAG, "Get serial id failed: %04X", err);
        return err;
    } else {
        ESP_LOGI(TAG, "SGP30 Serial ID: %04X-%04X", (uint32_t)(sgp30_Data.serialID>>32), (uint32_t)sgp30_Data.serialID);
    }
    if((err = sgp30_GetFeatureSetVersion()) != ESP_OK) {
        ESP_LOGE(TAG, "Get feature set version failed: %04X", err);
        return err;
    } else {
        ESP_LOGI(TAG, "SGP30 ProductType: 0x%X, ProductVersion: 0x%X", (sgp30_Data.featureSetVer >> 12), (uint8_t)sgp30_Data.featureSetVer);
    }
    return err;
}

esp_err_t sgp30_ReadData() {
    esp_err_t err;
    if((err = sgp30_SendCommand(SGP_CMD_GET_MEASURE)) != ESP_OK) {
//        ESP_LOGE(TAG, "Data read failed: %04X", err);
        return err;
    }

    vTaskDelay(12 / portTICK_PERIOD_MS);

    uint8_t* data = (uint8_t*)calloc(sizeof(uint8_t), 6);
    err = sgp30_ReadFromI2C(data, 6);
    if(err != ESP_OK) {
//        ESP_LOGE(TAG, "I2C Read failed 0x%04X", err);
        free(data);
        return err;
    }

    uint8_t crc;

    sgp30_Data.eCO2 = ((uint16_t)data[0] << 8) | ((uint16_t)data[1]);
    crc = sgp30_CalcChecksum(sgp30_Data.eCO2);
    if(crc != data[2]) {
        err = SGP30_ERR_BAD_CRC;
        ESP_LOGW(TAG, "%s::CRC not matched at eCO2 expected(0x%X) got(0x%X)", __func__, data[2], crc);
    }
    sgp30_Data.tVoC = ((uint16_t)data[3] << 8) | ((uint16_t)data[4]);
    crc = sgp30_CalcChecksum(sgp30_Data.tVoC);
    if(crc != data[5]) {
        err = SGP30_ERR_BAD_CRC;
        ESP_LOGW(TAG, "%s::CRC not matched at tVoC expected(0x%X) got(0x%X)", __func__, data[2], crc);
    }

    free(data);
    return err;
}

esp_err_t sgp30_InitMeasure() {
    esp_err_t err = ESP_OK;
    if((err = sgp30_SendCommand(SGP_CMD_INIT_MEASURE)) != ESP_OK) {
//        ESP_LOGE(TAG, "Data read failed: %04X", err);
        return err;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
    return err;
}

esp_err_t sgp30_GetBaseline() {
    esp_err_t err = ESP_OK;
    if((err = sgp30_SendCommand(SGP_CMD_GET_BASELINE)) != ESP_OK) {
//        ESP_LOGE(TAG, "Data read failed: %04X", err);
        return err;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);

    uint8_t* data = (uint8_t*)calloc(sizeof(uint8_t), 6);
    err = sgp30_ReadFromI2C(data, 6);
    if(err != ESP_OK) {
//        ESP_LOGE(TAG, "I2C Read failed 0x%04X", err);
        free(data);
        return err;
    }

    uint8_t crc;

    sgp30_Data.eCO2_baseline = ((uint16_t)data[0] << 8) | ((uint16_t)data[1]);
    crc = sgp30_CalcChecksum(sgp30_Data.eCO2);
    if(crc != data[2]) {
        err = SGP30_ERR_BAD_CRC;
        ESP_LOGW(TAG, "%s::CRC not matched at eCO2_baseline expected(0x%X) got(0x%X)", __func__, data[2], crc);
    }
    sgp30_Data.tVoC_baseline = ((uint16_t)data[3] << 8) | ((uint16_t)data[4]);
    crc = sgp30_CalcChecksum(sgp30_Data.tVoC);
    if(crc != data[5]) {
        err = SGP30_ERR_BAD_CRC;
        ESP_LOGW(TAG, "%s::CRC not matched at tVoC_baseline expected(0x%X) got(0x%X)", __func__, data[2], crc);
    }

    free(data);
    return err;
}

esp_err_t sgp30_SetBaseline(uint16_t tVoC, uint16_t eCO2) {
    esp_err_t err = ESP_OK;
    uint16_t args[2] = {tVoC, eCO2};
    if((err = sgp30_SendCommandWithArgument(SGP_CMD_SET_BASELINE, args, 2)) != ESP_OK) {
//        ESP_LOGE(TAG, "Data read failed: %04X", err);
        return err;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
    return err;
}

esp_err_t sgp30_SetHumidity(double absoluteHumidity) {
    esp_err_t err = ESP_OK;
    uint16_t args[1] = {sgp30_doubleToFixedPoint(absoluteHumidity)};
    if((err = sgp30_SendCommandWithArgument(SGP_CMD_SET_HUMIDITY, args, 1)) != ESP_OK) {
//        ESP_LOGE(TAG, "Data read failed: %04X", err);
        return err;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
    return err;
}

esp_err_t sgp30_SelfTest() {
    esp_err_t err = ESP_OK;
    if((err = sgp30_SendCommand(SGP_CMD_SELF_TEST)) != ESP_OK) {
//        ESP_LOGE(TAG, "Data read failed: %04X", err);
        return err;
    }

    vTaskDelay(220 / portTICK_PERIOD_MS);

    uint8_t* data = (uint8_t*)calloc(sizeof(uint8_t), 3);
    err = sgp30_ReadFromI2C(data, 3);
    if(err != ESP_OK) {
//        ESP_LOGE(TAG, "I2C Read failed 0x%04X", err);
        free(data);
        return err;
    }

    uint8_t crc;
    uint16_t result;

    result = ((uint16_t)data[0] << 8) | ((uint16_t)data[1]);
    crc = sgp30_CalcChecksum(result);
    if(crc != data[2]) {
        err = SGP30_ERR_BAD_CRC;
        ESP_LOGW(TAG, "%s::CRC not matched at result expected(0x%X) got(0x%X)", __func__, data[2], crc);
    }
    if(result != 0xD400) {
        err = SGP30_ERR_SELF_TEST_FAILED;
//        ESP_LOGE(TAG, "SGP30 Self test failed. replace the sensor");
    }

    free(data);
    return err;
}

esp_err_t sgp30_GetFeatureSetVersion() {
    esp_err_t err = ESP_OK;
    if((err = sgp30_SendCommand(SGP_CMD_GET_FEATURE_SET_VER)) != ESP_OK) {
//        ESP_LOGE(TAG, "Data read failed: %04X", err);
        return err;
    }

    vTaskDelay(2 / portTICK_PERIOD_MS);

    uint8_t* data = (uint8_t*)calloc(sizeof(uint8_t), 3);
    err = sgp30_ReadFromI2C(data, 3);
    if(err != ESP_OK) {
//        ESP_LOGE(TAG, "I2C Read failed 0x%04X", err);
        free(data);
        return err;
    }

    uint8_t crc;

    sgp30_Data.featureSetVer = ((uint16_t)data[0] << 8) | ((uint16_t)data[1]);
    crc = sgp30_CalcChecksum(sgp30_Data.featureSetVer);
    if(crc != data[2]) {
        err = SGP30_ERR_BAD_CRC;
        ESP_LOGW(TAG, "%s::CRC not matched at featureSetVer expected(0x%X) got(0x%X)", __func__, data[2], crc);
    }

    free(data);
    return err;
}

esp_err_t sgp30_GetSerialID() {
    esp_err_t err = ESP_OK;
    if((err = sgp30_SendCommand(SGP_CMD_GET_SERIAL_ID)) != ESP_OK) {
//        ESP_LOGE(TAG, "Data read failed: %04X", err);
        return err;
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);

    uint8_t* data = (uint8_t*)calloc(sizeof(uint8_t), 9);
    err = sgp30_ReadFromI2C(data, 9);
    if(err != ESP_OK) {
//        ESP_LOGE(TAG, "I2C Read failed 0x%04X", err);
        free(data);
        return err;
    }

    uint8_t crc;
    uint16_t serialHigh, serialMid, serialLow;

    serialHigh = ((uint16_t)data[0] << 8) | ((uint16_t)data[1]);
    crc = sgp30_CalcChecksum(serialHigh);
    if(crc != data[2]) {
        err = SGP30_ERR_BAD_CRC;
        ESP_LOGW(TAG, "CRC not matched at serialHigh expected(0x%X) got(0x%X)", data[2], crc);
    }
    serialMid = ((uint16_t)data[3] << 8) | ((uint16_t)data[4]);
    crc = sgp30_CalcChecksum(serialMid);
    if(crc != data[5]) {
        err = SGP30_ERR_BAD_CRC;
        ESP_LOGW(TAG, "CRC not matched at serialMid expected(0x%X) got(0x%X)", data[5], crc);
    }
    serialLow = ((uint16_t)data[6] << 8) | ((uint16_t)data[7]);
    crc = sgp30_CalcChecksum(serialLow);
    if(crc != data[8]) {
        err = SGP30_ERR_BAD_CRC;
        ESP_LOGW(TAG, "CRC not matched at serialLow expected(0x%X) got(0x%X)", data[8], crc);
    }

    sgp30_Data.serialID = ((uint64_t)serialHigh << 32) | ((uint64_t)serialMid << 16) | ((uint64_t)serialLow);

    free(data);
    return err;
}

esp_err_t sgp30_ReadRawSignal() {
    esp_err_t err;
    if((err = sgp30_SendCommand(SGP_CMD_GET_RAW_SIGNAL)) != ESP_OK) {
//        ESP_LOGE(TAG, "Data read failed: %04X", err);
        return err;
    }

    vTaskDelay(25 / portTICK_PERIOD_MS);

    uint8_t* data = (uint8_t*)calloc(sizeof(uint8_t), 6);
    err = sgp30_ReadFromI2C(data, 6);
    if(err != ESP_OK) {
//        ESP_LOGE(TAG, "I2C Read failed 0x%04X", err);
        free(data);
        return err;
    }

    uint8_t crc;

    sgp30_Data.H2 = ((uint16_t)data[0] << 8) | ((uint16_t)data[1]);
    crc = sgp30_CalcChecksum(sgp30_Data.H2);
    if(crc != data[2]) {
        err = SGP30_ERR_BAD_CRC;
        ESP_LOGW(TAG, "%s::CRC not matched at H2 expected(0x%X) got(0x%X)", __func__, data[2], crc);
    }
    sgp30_Data.Ethanol = ((uint16_t)data[3] << 8) | ((uint16_t)data[4]);
    crc = sgp30_CalcChecksum(sgp30_Data.Ethanol);
    if(crc != data[5]) {
        err = SGP30_ERR_BAD_CRC;
        ESP_LOGW(TAG, "%s::CRC not matched at Ethanol expected(0x%X) got(0x%X)", __func__, data[2], crc);
    }

    free(data);
    return err;
}

esp_err_t sgp30_Reset() {
    esp_err_t ret = ESP_OK;
    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, ( 0 ) | I2C_MASTER_WRITE, 1);

    i2c_master_write_byte(i2c_cmd, 0x06, true);

    i2c_master_stop(i2c_cmd);
    ret = i2c_master_cmd_begin(spg30_param.i2c_port, i2c_cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(i2c_cmd);
    if(ret != ESP_OK) {
//        ESP_LOGE(TAG, "I2C Write failed 0x%04X", ret);
        return ret;
    }
    return ret;
}

esp_err_t sgp30_SendCommand(uint16_t cmd) {
    esp_err_t ret = ESP_OK;
    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, ( SGP30_I2C_ADDR << 1 ) | I2C_MASTER_WRITE, 1);

    i2c_master_write_byte(i2c_cmd, (uint8_t)(cmd >> 8), true);
    i2c_master_write_byte(i2c_cmd, (uint8_t)cmd, true);

    i2c_master_stop(i2c_cmd);
    ret = i2c_master_cmd_begin(spg30_param.i2c_port, i2c_cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(i2c_cmd);
    if(ret != ESP_OK) {
//        ESP_LOGE(TAG, "I2C Write failed 0x%04X", ret);
        return ret;
    }
    return ret;
}

esp_err_t sgp30_SendCommandWithArgument(uint16_t cmd, uint16_t* args, uint8_t argCount) {
    esp_err_t ret = ESP_OK;
    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, ( SGP30_I2C_ADDR << 1 ) | I2C_MASTER_WRITE, 1);

    i2c_master_write_byte(i2c_cmd, (uint8_t)(cmd >> 8), true);
    i2c_master_write_byte(i2c_cmd, (uint8_t)cmd, true);
    for(int i = 0; i < argCount; i++) {
        i2c_master_write_byte(i2c_cmd, (uint8_t) (args[i] >> 8), true);
        i2c_master_write_byte(i2c_cmd, (uint8_t) args[i], true);
        i2c_master_write_byte(i2c_cmd, sgp30_CalcChecksum(args[i]), true);
    }
    i2c_master_stop(i2c_cmd);
    ret = i2c_master_cmd_begin(spg30_param.i2c_port, i2c_cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(i2c_cmd);
    if(ret != ESP_OK) {
//        ESP_LOGE(TAG, "I2C Write failed 0x%04X", ret);
        return ret;
    }
    return ret;
}

esp_err_t sgp30_ReadFromI2C(uint8_t* data, uint8_t length) {
    esp_err_t err = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SGP30_I2C_ADDR << 1 ) | I2C_MASTER_READ, 1);
    i2c_master_read(cmd, data, (size_t)(length-1), I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[length-1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(spg30_param.i2c_port, cmd, 2000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

uint8_t sgp30_CalcChecksum(uint16_t data) {
    uint8_t CRC = 0xFF; //inital value
    CRC ^= (uint8_t)(data >> 8); //start with MSB
    CRC = Sensirion_CRC8LookupTable[CRC >> 4][CRC & 0xF]; //look up table [MSnibble][LSnibble]
    CRC ^= (uint8_t)data; //use LSB
    CRC = Sensirion_CRC8LookupTable[CRC >> 4][CRC & 0xF]; //look up table [MSnibble][LSnibble]
    return CRC;
}

uint16_t sgp30_doubleToFixedPoint(double number) {
    int power = 1 << 8;
    double number2 = number * power;
    uint16_t value = (uint16_t)floor(number2 + 0.5);
    return value;
}

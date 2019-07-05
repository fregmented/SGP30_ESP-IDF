//
// Created by hanwool on 2018-10-15.
//

#ifndef __SGP30_H
#define __SGP30_H

#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"


#if defined(__cplusplus)
extern "C" {
#endif

#ifndef SGP30_HAL_UNDEFINED
#define SGP30_HAL_UNDEFINED (-1)
#endif

#define SGP30_I2C_ADDR 0x58
#define SGP30_MAX_I2C_MASTER_FREQ_HZ          400000

#define SGP30_ERR_BAD_CRC 0x9000
#define SGP30_ERR_SELF_TEST_FAILED 0x9001


#define SGP30_ESP32_HAL_DEFAULT {false, SGP30_HAL_UNDEFINED, SGP30_HAL_UNDEFINED, I2C_NUM_MAX, SGP30_MAX_I2C_MASTER_FREQ_HZ};

typedef struct {
    bool using_library_i2c; // if true, initialize i2c driver in library. if false, must initialize i2c driver outside of library
    gpio_num_t sda; // data pin for I²C
    gpio_num_t scl; // clock pin for I²C
    i2c_port_t i2c_port; // i2c port number, defined in i2c.h
    uint32_t i2c_freq_hz; // i2c clock speed maximum SGP30_MAX_I2C_MASTER_FREQ_HZ
} sgp30_param_t;

struct _S_SGP_DATA {
    int16_t tVoC;
    int16_t eCO2;
    int16_t H2;
    int16_t Ethanol;
    int16_t tVoC_baseline;
    int16_t eCO2_baseline;
    int16_t featureSetVer;
    uint64_t serialID;
} sgp30_Data;

static const uint16_t SGP_CMD_INIT_MEASURE = 0x2003;
static const uint16_t SGP_CMD_GET_MEASURE = 0x2008;
static const uint16_t SGP_CMD_GET_BASELINE = 0x2015;
static const uint16_t SGP_CMD_SET_BASELINE = 0x201E;
static const uint16_t SGP_CMD_SET_HUMIDITY = 0x2061;
static const uint16_t SGP_CMD_SELF_TEST = 0x2032;
static const uint16_t SGP_CMD_GET_FEATURE_SET_VER = 0x202F;
static const uint16_t SGP_CMD_GET_SERIAL_ID = 0x3682;
static const uint16_t SGP_CMD_GET_RAW_SIGNAL = 0x2050;

#ifndef SENSIRION_CRC8_TABLE
#define SENSIRION_CRC8_TABLE
static const uint8_t Sensirion_CRC8LookupTable[16][16] = {
        {0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97, 0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E},
        {0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4, 0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D},
        {0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11, 0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8},
        {0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52, 0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB},
        {0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA, 0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13},
        {0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9, 0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50},
        {0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C, 0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95},
        {0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F, 0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6},
        {0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED, 0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54},
        {0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE, 0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17},
        {0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B, 0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2},
        {0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28, 0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91},
        {0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0, 0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69},
        {0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93, 0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A},
        {0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56, 0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF},
        {0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15, 0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC}
};
#endif

/**
 * Initialize SGP30 and library.
 * @param params sgp30_param_t
 *     - using_library_i2c if true, initialize i2c driver in library. if false, must initialize i2c driver outside of library
 *     - sda data pin for I²C
 *     - scl clock pin for I²C
 *     - i2c_port i2c port number, defined in i2c.h
 *     - i2c_freq_hz i2c clock speed maximum SGP30_MAX_I2C_MASTER_FREQ_HZ
 * @return I2C command result
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 *     - ESP_ERR_INVALID_ARG Invalid parameter value.
 *     - SGP30_ERR_BAD_CRC CRC if not matched.
 */
esp_err_t sgp30_Init(sgp30_param_t params);

/**
 * Read TVOC data and eCO2 data.
 * Data will saved in sgp30_Data.
 * @return I2C command result
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 *     - SGP30_ERR_BAD_CRC CRC if not matched.
 */
esp_err_t sgp30_ReadData();

/**
 * Measurement initialize.
 * @return I2C command result
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 *     - SGP30_ERR_BAD_CRC CRC if not matched.
 */
esp_err_t sgp30_InitMeasure();

/**
 * Get SGP30 calibration baseline data.
 * Data will saved in sgp30_Data.
 * This method MUST call every 12hours and save baseline data into NVS.
 * @return I2C command result
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 *     - SGP30_ERR_BAD_CRC CRC if not matched.
 */
esp_err_t sgp30_GetBaseline();

/**
 * Set SGP30 calibration baseline data.
 * This method MUST call every Power on Reset. Otherwise, all measured data will invalid until the SGP30 has been calibrated for 12 hours.
 * @param tVoC
 * @param eCO2
 * @return I2C command result
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 *     - SGP30_ERR_BAD_CRC CRC if not matched.
 */
esp_err_t sgp30_SetBaseline(uint16_t tVoC, uint16_t eCO2);

/**
 * Set absolute humidity.
 * this method MUST call when humidity is changed over 10%.
 * calculation:
 * double eSat = 6.11 * pow(10.0, (7.5 * tempInCelsius / (237.7 + tempInCelsius)));
 * double vaporPressure = (relativeHumidity * eSat) / 100; //millibars
 * double absHumidity = 1000 * vaporPressure * 100 / ((tempInCelsius + 273) * 461.5);
 *
 * @param absoluteHumidity
 * @return I2C command result
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 *     - SGP30_ERR_BAD_CRC CRC if not matched.
 */
esp_err_t sgp30_SetHumidity(double absoluteHumidity);

/**
 * Do self test for SGP30.
 * @return I2C command result
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 *     - SGP30_ERR_BAD_CRC CRC if not matched.
 */
esp_err_t sgp30_SelfTest();

/**
 * Get hardware version of SGP30
 * @return I2C command result
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 *     - SGP30_ERR_BAD_CRC CRC if not matched.
 */
esp_err_t sgp30_GetFeatureSetVersion();

/**
 * Get hardware serial ID of SGP30
 * Data will saved in sgp30_Data
 * @return I2C command result
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 *     - SGP30_ERR_BAD_CRC CRC if not matched.
 */
esp_err_t sgp30_GetSerialID();

/**
 * Read Raw material data(H2 and ethanol).
 * Data will saved in sgp30_Data
 * @return I2C command result
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 *     - SGP30_ERR_BAD_CRC CRC if not matched.
 */
esp_err_t sgp30_ReadRawSignal();

/**
 * Send General Call Reset(0x06)
 * @return I2C command result
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
esp_err_t sgp30_Reset();


#if defined(__cplusplus)
}
#endif

#endif //__SGP30_H

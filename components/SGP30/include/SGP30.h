/**
 * 
 **/

#ifndef __SGP30_H
#define __SGP30_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "driver/i2c.h"

#define SGP30_SELF_TEST_ERROR 0x9001
#define SGP30_NOT_INITIALIZED 0x9002

#if defined(__cplusplus)
extern "C" {
#endif

#define SGP30_ADDRESS 0x58

#ifndef REAL_MS
#define REAL_MS(n) ((n) / portTICK_PERIOD_MS)
#endif
#ifndef REAL_SEC
#define REAL_SEC(n) ((n)*1000 / portTICK_PERIOD_MS)
#endif
#ifndef CHECK_ERROR
#define CHECK_ERROR(x) if(x != ESP_OK) {return x;}
#endif

uint16_t TVoC = 0;
uint16_t eCO2 = 0;
uint16_t BaseTVoC = 0;
uint16_t BaseECO2 = 0;
uint16_t RawH2 = 0;
uint16_t RawEthanol = 0;

uint16_t SerialNumber = 0;
uint8_t ProductVersion = 0;


static i2c_port_t _i2cPort;
static SemaphoreHandle_t _semaphore = NULL;
static bool isInitialized = false;

esp_err_t SGP30_Init(i2c_port_t i2cPort);
esp_err_t SGP30_InitWithSemaphore(i2c_port_t i2cPort, SemaphoreHandle_t semaphore);
esp_err_t SGP30_GetAirQuality();
esp_err_t SGP30_GetBaseline();
esp_err_t SGP30_SetBaseline(uint16_t baseTVoC, uint16_t baseECO2);
esp_err_t SGP30_SelfTest();
esp_err_t SGP30_Reset();
esp_err_t SGP30_GetSerial();
esp_err_t SGP30_GetFeatureSet();
esp_err_t SGP30_SetHumidity(double relativeHumidity);
esp_err_t SGP30_MeasureRawSignal();


#if defined(__cplusplus)
}
#endif 

#endif // __SGP30_H
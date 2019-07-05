/**
 *
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "nvs.h"

#include "SGP30.h"


#define NVS_BASE_NAME "storage"
#define SGP_TVOC_BASE_KEY "SGP_TVOC"
#define SGP_ECO2_BASE_KEY "SGP_ECO2"

#ifndef REAL_MS
#define REAL_MS(n) ((n) / portTICK_PERIOD_MS)
#endif
#ifndef REAL_SEC
#define REAL_SEC(n) ((n)*1000 / portTICK_PERIOD_MS)
#endif
#ifndef CHECK_ERROR
#define CHECK_ERROR(x) if(x != ESP_OK) {return x;}
#endif

static void init_sgp30();

static void sgpTask(void* pvParams) {
    while(true) {
        esp_err_t err;
        if((err = sgp30_ReadData()) == ESP_OK) {
            ESP_LOGV(__func__, "tVoC: %dppb, eCO2: %dppm",
                     sgp30_Data.tVoC, sgp30_Data.eCO2);
        } else {
            ESP_LOGE(__func__, "SGP READ ERROR:: 0x%04X", err);
        }
        vTaskDelay(REAL_SEC(5));
    }
    vTaskDelete(NULL);
}

static void sgpBaselineTask(void* pvParams) {
    static uint8_t burnTime = 0;
    static bool isBurned = true;


    static uint16_t tVocBase = 0xFFFF;
    static uint16_t eCo2Base = 0xFFFF;
    bool loadOk = true;
    nvs_handle nvsReadHandle;
    esp_err_t ret = nvs_open(NVS_BASE_NAME, NVS_READWRITE, &nvsReadHandle);
    ESP_ERROR_CHECK(ret);

    if ((ret = nvs_get_u16(nvsReadHandle, SGP_TVOC_BASE_KEY, &tVocBase)) != ESP_OK) {
        ESP_LOGE(__func__, "SGP30::Get tVoc_base from NVS failed %s(0x%X)", esp_err_to_name(ret), ret);
        loadOk = loadOk && false;
    }
    if ((ret = nvs_get_u16(nvsReadHandle, SGP_ECO2_BASE_KEY, &eCo2Base)) != ESP_OK) {
        ESP_LOGE(__func__, "SGP30::Get eCO2_base from NVS failed %s(0x%X)", esp_err_to_name(ret), ret);
        loadOk = loadOk && false;
    }
    if (loadOk) {
        isBurned = true;
    }
    nvs_close(nvsReadHandle);

    while (true) {

        vTaskDelay(REAL_SEC(3600));

        if (++burnTime > 12) {
            isBurned = true;
        }
        if (isBurned) {
            nvs_handle nvsHandle;
            esp_err_t err = nvs_open(NVS_BASE_NAME, NVS_READWRITE, &nvsHandle);
            ESP_ERROR_CHECK(err);
            if (sgp30_GetBaseline() == ESP_OK) {
                ESP_LOGD(__func__, "SGP BASELINE CORRECTION DATA FROM SGP30 tVocBase: %d eCO2Base: %d", tVocBase, eCo2Base);

                tVocBase = sgp30_Data.tVoC_baseline;
                eCo2Base = sgp30_Data.eCO2_baseline;
                if ((err = nvs_set_u16(nvsHandle, SGP_TVOC_BASE_KEY, tVocBase)) == ESP_OK) {
                    if ((err = nvs_set_u16(nvsHandle, SGP_ECO2_BASE_KEY, eCo2Base)) == ESP_OK) {
                        if ((err = nvs_commit(nvsHandle)) == ESP_OK) {
                            ESP_LOGE(__func__, "SGP BASELINE CORRECTION INFO SAVED TO NVS");
                        } else {
                            ESP_LOGE(__func__, "SGP BASELINE CORRECTION INFO SAVE NVS ERROR %s(0x%X)", esp_err_to_name(ret), err);
                        }
                    } else {
                        ESP_LOGE(__func__, "SGP BASELINE CORRECTION INFO SAVE eCo2Base TO NVS ERROR %s(0x%X)", esp_err_to_name(ret), err);
                    }
                } else {
                    ESP_LOGE(__func__, "SGP BASELINE CORRECTION INFO SAVE tVocBase TO NVS ERROR %s(0x%X)", esp_err_to_name(ret), err);
                }
            } else {
                ESP_LOGE(__func__, "SGP BASELINE CORRECTION INFO IS WRONG");
                nvs_close(nvsHandle);
            }
        }
    }
}

void app_main() {
    init_sgp30();
    while(true);
}

void init_sgp30() {

    sgp30_param_t sgp30Param = SGP30_ESP32_HAL_DEFAULT;
    sgp30Param.i2c_freq_hz = CONFIG_I2C_SPEED * 1000;
    sgp30Param.scl = CONFIG_I2C_SCL;
    sgp30Param.sda = CONFIG_I2C_SDA;
    sgp30Param.i2c_port = I2C_NUM_1;
    sgp30Param.using_library_i2c = true;
    ESP_ERROR_CHECK(sgp30_Init(sgp30Param))

    esp_err_t err = ESP_OK;
    uint16_t tVocBase = 0xFFFF;
    uint16_t eCo2Base = 0xFFFF;

    bool loadOk = true;
    nvs_handle nvsHandle;
    esp_err_t ret = nvs_open(NVS_BASE_NAME, NVS_READWRITE, &nvsHandle);
    ESP_ERROR_CHECK(ret);

    if ((ret = nvs_get_u16(nvsHandle, SGP_TVOC_BASE_KEY, &tVocBase)) != ESP_OK) {
        ESP_LOGE(__func__, "SGP30::Get tVoc_base from NVS failed 0x%X", ret);
        loadOk = loadOk & false;
        tVocBase = 0;
        if((err = nvs_set_u16(nvsHandle, SGP_TVOC_BASE_KEY, tVocBase)) != ESP_OK) {
            ESP_LOGE(__func__, "%s: nvs_set_u16 failed %s(0x%X). failback to AQ_PM025", __func__, esp_err_to_name(err), err);
        } else {
            ESP_LOGI(__func__, "%s: nvs_set_u16(tVocBase) success", __func__);
        }
        if((err = nvs_commit(nvsHandle)) != ESP_OK) {
            ESP_LOGE(__func__, "%s: nvs_commit failed %s(0x%X). failback to AQ_PM025", __func__, esp_err_to_name(err), err);
        } else {
            ESP_LOGI(__func__, "%s: nvs_commit(tVocBase) success", __func__);
        }
    }
    if ((ret = nvs_get_u16(nvsHandle, SGP_ECO2_BASE_KEY, &eCo2Base)) != ESP_OK) {
        ESP_LOGE(__func__, "SGP30::Get eCO2_base from NVS failed 0x%X", ret);
        loadOk = loadOk & false;
        eCo2Base = 0;
        if((err = nvs_set_u16(nvsHandle, SGP_ECO2_BASE_KEY, eCo2Base)) != ESP_OK) {
            ESP_LOGE(__func__, "%s: nvs_set_u16 failed %s(0x%X). failback to AQ_PM025", __func__, esp_err_to_name(err), err);
        } else {
            ESP_LOGI(__func__, "%s: nvs_set_u16(eCo2Base) success", __func__);
        }
        if((err = nvs_commit(nvsHandle)) != ESP_OK) {
            ESP_LOGE(__func__, "%s: nvs_commit failed %s(0x%X). failback to AQ_PM025", __func__, esp_err_to_name(err), err);
        } else {
            ESP_LOGI(__func__, "%s: nvs_commit(eCo2Base) success", __func__);
        }
    }
    if (loadOk) {
        ESP_LOGD(__func__, "SGP BASELINE CORRECTION DATA FROM NVS tVocBase: %04X eCO2Base: %04X", tVocBase, eCo2Base);

    } else {
        ESP_LOGE(__func__, "SGP30::Baseline set from NVS stored data is Failed");
    }
    nvs_close(nvsHandle);

    sgp30_InitMeasure();
    if(loadOk) {
        ESP_LOGW(__func__, "SGP BASELINE CORRECTION DATA FROM NVS tVocBase: %04X eCO2Base: %04X", tVocBase, eCo2Base);
        sgp30_SetBaseline(tVocBase, eCo2Base);
    }

    xTaskCreate(sgpTask, "sgpTask", 4096, NULL, 10, NULL);
    xTaskCreate(sgpBaselineTask, "sgpBaselineTask", 4096, NULL, 11, NULL);

}

/**
 *
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"

//#include "SGP30.h"


#ifndef REAL_MS
#define REAL_MS(n) ((n) / portTICK_PERIOD_MS)
#endif
#ifndef REAL_SEC
#define REAL_SEC(n) ((n)*1000 / portTICK_PERIOD_MS)
#endif
#ifndef CHECK_ERROR
#define CHECK_ERROR(x) if(x != ESP_OK) {return x;}
#endif

SemaphoreHandle_t semaphoreHandle;
i2c_port_t i2c_master_port = I2C_NUM_1;

static void SGP30TestTask(void* pvParams) {
    while(true) {
        ESP_LOGD("SGP30TestTask", "AAAA");
        vTaskDelay(REAL_SEC(5));
    }
    vTaskDelete(NULL);
}

static void i2c_master_init()
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = CONFIG_I2C_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = CONFIG_I2C_SCL;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = CONFIG_I2C_SPEED * 1000;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

void app_main() {
    semaphoreHandle = xSemaphoreCreateMutex();
    i2c_master_init();
#if defined(CONFIG_USING_SEMAPHORE)
//    ESP_ERROR_CHECK(SGP30_InitWithSemaphore(i2c_master_port, semaphoreHandle));
#else
//    ESP_ERROR_CHECK(SGP30_Init(i2c_master_port));
#endif
    xTaskCreate(SGP30TestTask, "SGP30TestTask", 1024 * 2, NULL, 10, NULL);

}


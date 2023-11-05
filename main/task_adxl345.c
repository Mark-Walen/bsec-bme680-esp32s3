#include <string.h>
#include "adxl345/adxl345.h"
#include "task_adxl345.h"
#include "esp_log.h"

#define ADXL345_TAG "task_adxl345"

TaskHandle_t *task_adxl345_handle = NULL;
SemaphoreHandle_t int2_semphr_handle = NULL;

static void adxl345_sensor_init(adxl345_t *adxl345)
{
    int32_t ret = 0;
    uint8_t speed_ctl = ADXL345_RATE_100
                      | ADXL345_LOW_POWER;

    uint8_t power_ctl = ADXL345_PCTL_LINK
                      | ADXL345_PCTL_AUTO_SLEEP
                      | ADXL345_PCTL_MEASURE;
    
    uint8_t current_ctl = ADXL345_ACT_ACDC  
                        | ADXL345_ACT_X_EN  
                        | ADXL345_ACT_Y_EN  
                        | ADXL345_ACT_Z_EN
                        | ADXL345_INACT_ACDC
                        | ADXL345_INACT_X_EN
                        | ADXL345_INACT_Y_EN
                        | ADXL345_INACT_Z_EN;


    uint8_t gpio_ctl = ADXL345_ACTIVITY | ADXL345_INACTIVITY;

    ESP_LOGI(ADXL345_TAG, "Initializing Motion Detect");

    // 2. Init adxl345
    if ((ret = adxl345_init(adxl345)) != DEVICE_OK)
    {
        while (1)
        {
            ESP_LOGE(ADXL345_TAG, "ret = %d", ret);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    ESP_LOGI(ADXL345_TAG, "Found motion device adxl345");
    adxl345_set_register_value(adxl345, ADXL345_BW_RATE, speed_ctl);
    adxl345_set_register_value(adxl345, ADXL345_INT_ENABLE, 0x00);

    adxl345_set_activity_detection(adxl345, 0x01, ADXL345_ACT_X_EN | ADXL345_ACT_Y_EN | ADXL345_ACT_Z_EN, ADXL345_ACT_ACDC, 0x0A, ADXL345_ACTIVITY);

    adxl345_set_inactivity_detection(adxl345, 0x01, ADXL345_INACT_X_EN | ADXL345_INACT_Y_EN | ADXL345_INACT_Z_EN, ADXL345_INACT_ACDC, 0x0A, 0x01, ADXL345_INACTIVITY);

    // adxl345_set_register_value(adxl345, ADXL345_THRESH_ACT, 0x0A);
    // adxl345_set_register_value(adxl345, ADXL345_THRESH_INACT, 0x0A);
    // adxl345_set_register_value(adxl345, ADXL345_TIME_INACT, 0x01);
    // adxl345_set_register_value(adxl345, ADXL345_ACT_INACT_CTL, current_ctl);
    // adxl345_set_register_value(adxl345, ADXL345_INT_MAP, gpio_ctl);

    // uint8_t int_ctl = ADXL345_ACTIVITY | ADXL345_INACTIVITY;
    // adxl345_set_register_value(adxl345, ADXL345_INT_ENABLE, int_ctl);
    // ESP_LOGI(ADXL345_TAG, "%d", adxl345_get_register_value(adxl345, ADXL345_THRESH_ACT) == 0x0A);
    // ESP_LOGI(ADXL345_TAG, "%d", adxl345_get_register_value(adxl345, ADXL345_THRESH_INACT) == 0x0A);
    // ESP_LOGI(ADXL345_TAG, "%d", adxl345_get_register_value(adxl345, ADXL345_TIME_INACT) == 0x01);
    // ESP_LOGI(ADXL345_TAG, "%x", adxl345_get_register_value(adxl345, ADXL345_ACT_INACT_CTL));
    // ESP_LOGI(ADXL345_TAG, "%d", adxl345_get_register_value(adxl345, ADXL345_INT_MAP) == int_ctl);
    
    adxl345_set_register_value(adxl345, ADXL345_POWER_CTL, power_ctl);
    ESP_LOGI(ADXL345_TAG, "Setup over");
}

static void adxl345_task_func(void *args)
{
    // 1. Variables;
    // float x, y, z;
    uint8_t is_active;
    uint8_t int_source = 0;
    uint16_t activity_count = 0;
    adxl345_t *adxl345 = (adxl345_t *) args;

    adxl345_sensor_init(adxl345);

    while (1)
    {
        ESP_LOGI(ADXL345_TAG, "Waiting for interrupt...");
        xSemaphoreTake(int2_semphr_handle, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));
        int_source = adxl345_get_register_value(adxl345, ADXL345_INT_SOURCE);
        is_active = ((int_source & 0x10) == 0x10);
        adxl345_int_flag = 1;
        while (adxl345_int_flag)
        {
            if (!is_active)
            {
                adxl345_int_flag = 0;
                ESP_LOGI(ADXL345_TAG, "Inactive");
            }
            else
            {
                activity_count++;
                if (activity_count == 0xFFFF)
                {
                    activity_count = 0;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        ESP_LOGI(ADXL345_TAG,"Activity Count: %d", activity_count);
    }
}

void create_task_adxl345(adxl345_t *adxl345)
{
    ESP_LOGI(ADXL345_TAG, "Create Motion Detect Task");
    xTaskCreate((TaskFunction_t) adxl345_task_func,
                "TaskADXL362",
                (configSTACK_DEPTH_TYPE) ADXL345_TASK_STK_SIZE,
                (void *) adxl345,
                (UBaseType_t) ADXL345_TASK_PRIO,
                task_adxl345_handle);

    int2_semphr_handle = xSemaphoreCreateBinary();
}
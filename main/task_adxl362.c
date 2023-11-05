#include <string.h>
#include "adxl362/adxl362.h"
#include "task_adxl362.h"

TaskHandle_t *task_adxl362_handle = NULL;
SemaphoreHandle_t int1_semphr_handle = NULL;

static void adxl362_task_func(void *args)
{
    // 1. Variables;
    // float x, y, z;
    int16_t  x_g, y_g, z_g, ret;
    adxl362_t *adxl362 = (adxl362_t *)args;
    uint8_t fifo_ctl = ADXL362_FIFO_STREAM;
    uint8_t int_map = ADXL362_INTMAP1_ACT | ADXL362_INTMAP1_INACT;
    uint8_t filter_ctl = ADXL362_FILTER_CTL_RANGE(ADXL362_RANGE_2G) | ADXL362_FILTER_CTL_ODR(ADXL362_ODR_12_5_HZ) | ADXL362_FILTER_CTL_HALF_BW;
    uint8_t act_inact_ctl = ADXL362_ACT_INACT_CTL_LINKLOOP(ADXL362_MODE_LINK)
                            | ADXL362_ACT_INACT_CTL_INACT_REF
                            | ADXL362_ACT_INACT_CTL_INACT_EN
                            | ADXL362_ACT_INACT_CTL_ACT_REF
                            | ADXL362_ACT_INACT_CTL_ACT_EN;
    
    uint8_t status = 0;
    uint8_t fifo_buffer[ADXL362_FIFO_SAMPLE_SIZE];
    memset(fifo_buffer, 0, sizeof(uint8_t) * ADXL362_FIFO_SAMPLE_SIZE);

    // 2. Init adxl362
    if ((ret = adxl362_init(adxl362)) != DEVICE_OK)
    {
        while (1)
        {
            printf("ret = %d\r\n", ret);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    // 3. adxl362 settings
    // 1) Soft reset to ensure fifo data is correct.
    adxl362_software_reset(adxl362);
    vTaskDelay(pdMS_TO_TICKS(100));
    adxl362_set_power_mode(adxl362, 0);
    adxl362_set_register_value(adxl362, filter_ctl, ADXL362_REG_FILTER_CTL, 1);
    adxl362_set_register_value(adxl362, act_inact_ctl, ADXL362_REG_ACT_INACT_CTL, 1);
    adxl362_setup_activity_detection(adxl362, 0x3F, 250, 0);
    adxl362_setup_inactivity_detection(adxl362, 0x3F, 150, 3);
    adxl362_set_register_value(adxl362, int_map, ADXL362_REG_INTMAP2, 1);
    // adxl362_set_wakeup_mode(adxl362, 1);
    adxl362_set_power_mode(adxl362, 1);
    // adxl362_get_register_value(adxl362, &status, ADXL362_REG_POWER_CTL, 1);
    // printf("%d\r\n", status);
    adxl362_get_register_value(adxl362, &status, ADXL362_REG_STATUS, 1);
    printf("Status: 0x%.2x\r\n", status);
    // 4. run task
    while (1)
    {
        printf("Wait for interrupt\r\n");
        xSemaphoreTake(int1_semphr_handle, portMAX_DELAY);
        adxl362_get_register_value(adxl362, &status, ADXL362_REG_STATUS, 1);
        printf("Status: 0x%.2x\r\n", status);
    }
}

void create_task_adxl362(adxl362_t *adxl362)
{
    xTaskCreate((TaskFunction_t) adxl362_task_func,
                "TaskADXL362",
                (configSTACK_DEPTH_TYPE) ADXL362_TASK_STK_SIZE,
                (void *) adxl362,
                (UBaseType_t) ADXL362_TASK_PRIO,
                task_adxl362_handle);

    int1_semphr_handle = xSemaphoreCreateBinary();
}
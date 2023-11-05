#ifndef __TASK_ADXL362_H__
#define __TASK_ADXL362_H__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define ADXL362_TASK_PRIO           3
#define ADXL362_TASK_STK_SIZE       2048
#define ADXL362_FIFO_SAMPLE_SIZE    0x80

extern SemaphoreHandle_t int1_semphr_handle;

void create_task_adxl362(adxl362_t *adxl362);
#endif
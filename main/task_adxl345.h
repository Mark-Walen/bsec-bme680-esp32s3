#ifndef __TASK_ADXL345_H__
#define __TASK_ADXL345_H__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define ADXL345_TASK_PRIO           3
#define ADXL345_TASK_STK_SIZE       2048
#define ADXL345_FIFO_SAMPLE_SIZE    0x80

extern SemaphoreHandle_t int2_semphr_handle;
extern volatile uint8_t adxl345_int_flag;

void create_task_adxl345(adxl345_t *adxl345);
#endif
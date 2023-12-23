#ifndef __TASK_BME680_H__
#define __TASK_BME680_H__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define BME680_TASK_PRIO           3
#define BME680_TASK_STK_SIZE       4096

void create_task_bme680(void *pvParameters);

#endif
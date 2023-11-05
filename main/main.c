/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp32_s3_driver.h"
#include "task_adxl362.h"
#include "task_adxl345.h"
#include "esp_log.h"

static const char *main_tag = "main";

void app_main(void)
{
    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%uMB %s flash\n", flash_size / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());


    adxl345_t adxl345;
    uint8_t adxl345_addr = ADXL345_ADDRESS;
    i2c_port_t adxl345_port = I2C_MASTER_NUM;
    device_t i2c_dev;

    gpio_init();
    platform_init("esp32s3", get_timestamp, delay_ms, printf);
    i2c_master_init(adxl345_port);
    device_init(&i2c_dev, i2c_master_receive, i2c_master_transmit, &adxl345_port, &adxl345_addr);
    adxl345_interface_init(&adxl345, &i2c_dev);
    
    create_task_adxl345(&adxl345);
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

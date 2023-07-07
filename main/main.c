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

void app_main(void)
{
    printf("Hello world!\n");

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

    spi_device_handle_t spi;
    device_t dev;
    adxl362_t adxl362;
    // w25qxx_handle_t w25q16;
    // adxl362_t adxl362;
    device_gpio_typedef_t w25q16_nss = {
        .pin = ADXL362_NSS_PIN,
        .port = NULL
    };
    spi_init(&spi);
    
    device_init(&dev, spi_read, spi_write, delay_ms, printf, get_timestamp, &w25q16_nss, &spi);
    // w25qxx_init(&w25q16, &dev, gpio_setPin, gpio_resetPin);
    adxl362_init(&adxl362, &dev, gpio_setPin, gpio_resetPin);

    adxl362_get_id(&adxl362);
    printf("0x%.4x\r\n", adxl362.dev->chip_id);    

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

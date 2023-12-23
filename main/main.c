/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
<<<<<<< HEAD
#include <string.h>
=======
>>>>>>> 501c018 (feat: spi test passed.)
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
<<<<<<< HEAD
#include "esp_log.h"
#include "esp32_s3_driver.h"
#include "bme68x/bme68x.h"
#include "task_bme680.h"

#define TAG "main"

static bme68x_t bme680_dev;
static device_t i2c_dev;

void app_main(void)
{
=======
#include "esp32_s3_driver.h"
#include <string.h>

void app_main(void)
{
    printf("Hello world!\n");

>>>>>>> 501c018 (feat: spi test passed.)
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

<<<<<<< HEAD
    uint8_t bme680_addr = BME68X_I2C_ADDR_LOW;
    i2c_port_t bme680_i2c_port = I2C_MASTER_NUM;
    memset(&bme680_dev, 0, sizeof(bme68x_t));
    memset(&i2c_dev, 0, sizeof(i2c_dev));

    platform_init("esp32s3", get_timestamp, delay_ms, printf);
    i2c_master_init(bme680_i2c_port);
    device_init(&i2c_dev, i2c_master_receive, i2c_master_transmit, &bme680_i2c_port, &bme680_addr);
    bme68x_interface_init(&bme680_dev, &i2c_dev, I2C);

    create_task_bme680(&bme680_dev);

    while(1){

=======
    spi_device_handle_t spi;
    device_t dev;
    w25qxx_handle_t w25q16;
    // adxl362_t adxl362;
    device_gpio_typedef_t w25q16_nss = {
        .pin = ADXL362_NSS_PIN,
        .port = NULL
    };
    spi_init(&spi);
    
    device_init(&dev, spi_read, spi_write, delay_ms, printf, get_timestamp, &w25q16_nss, &spi);
    w25qxx_init(&w25q16, &dev, gpio_setPin, gpio_resetPin);

    w25qxx_read_id(&w25q16);
    printf("0x%.4x\r\n", w25q16.dev->chip_id);    

    while(1) {
>>>>>>> 501c018 (feat: spi test passed.)
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

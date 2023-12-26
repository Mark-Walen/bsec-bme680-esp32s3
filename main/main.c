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
#include "nvs_flash.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp32_s3_driver.h"
#include "net_interface.h"
#include "bme68x/bme68x.h"
#include "task_bme680.h"
#include "task_mqtt.h"
#include "events_def.h"

#define TAG "main"

static bme68x_t bme680_dev;
static device_t i2c_dev;
EventGroupHandle_t s_event_group;
static wl_handle_t wlHandle = WL_INVALID_HANDLE;

static esp_err_t initialize_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    return err;
}

static esp_err_t mount_flash_partition(void){
    const esp_vfs_fat_mount_config_t mount_config = {
        .max_files = 4,
        .format_if_mount_failed = true,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl("/bme680", "storage", &mount_config, &wlHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Mount FatFS Failed. Mount name error: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "Mount Success");
    return ESP_OK;
}

void app_main(void)
{
    uint8_t bme680_addr = BME68X_I2C_ADDR_LOW;
    i2c_port_t bme680_i2c_port = I2C_MASTER_NUM;
    memset(&bme680_dev, 0, sizeof(bme68x_t));
    memset(&i2c_dev, 0, sizeof(i2c_dev));

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    // esp_log_level_set("*", ESP_LOG_INFO);
    // esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    // esp_log_level_set("task mqtt", ESP_LOG_VERBOSE);
    // esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    // esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    // esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    // esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    if (initialize_nvs() != ESP_OK || mount_flash_partition() != ESP_OK)
    {
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
    
    platform_init("esp32s3", get_timestamp, delay_ms, printf);
    s_event_group = xEventGroupCreate();
    
    net_iface_wifi_init();
    mqtt_task_start();
    i2c_master_init(bme680_i2c_port);
    device_init(&i2c_dev, i2c_master_receive, i2c_master_transmit, &bme680_i2c_port, &bme680_addr);
    bme68x_interface_init(&bme680_dev, &i2c_dev, I2C);
    create_task_bme680(&bme680_dev);

    while(1){

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

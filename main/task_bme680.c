#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_log.h"

#include "bsec_datatypes.h"
#include "bsec_interface.h"
#include "bsec_integration.h"
#include "bme68x/bme68x.h"
#include "bsec_iaq.h"

#include "esp32_s3_driver.h"
#include "task_bme680.h"
#include "task_mqtt.h"
#include "struct/struct.h"
#include "crc/crc16.h"

#define PAYLOAD_SIZE            35
#define PAYLOAD_START_IDX       6
#define TAG "BME680"

static TaskHandle_t *task_bme680_handle = NULL;
static char *filename_state = "/bme680/bsec_iaq.state";
static uint8_t payload[PAYLOAD_SIZE];

char *get_version(void)
{
    char *buffer = (char *)malloc(sizeof(char) * 16);
    bsec_version_t bsec_version;
    bsec_get_version(&bsec_version);
    snprintf(buffer, 16, "%d.%d.%d.%d", bsec_version.major, bsec_version.minor,
             bsec_version.major_bugfix, bsec_version.minor_bugfix);
    return buffer;
}

/*
 * Handling of the ready outputs
 *
 * param[in]       timestamp       time in microseconds
 * param[in]       iaq             IAQ signal
 * param[in]       iaq_accuracy    accuracy of IAQ signal
 * param[in]       temperature     temperature signal
 * param[in]       humidity        humidity signal
 * param[in]       pressure        pressure signal
 * param[in]       raw_temperature raw temperature signal
 * param[in]       raw_humidity    raw humidity signal
 * param[in]       gas             raw gas sensor signal
 * param[in]       bsec_status     value returned by the bsec_do_steps() call
 *
 * return          none
 */
void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float static_iaq, float co2_equivalent, float breath_voc_equivalent,
                  float raw_pressure, float raw_temp, float temp, float raw_humidity, float humidity, float raw_gas, float gas_percentage,
                  float stabilization_status, float run_in_status, bsec_library_return_t bsec_status)
{
    int64_t timestamp_s = timestamp / 1000000000;

    printf("{\"IAQ Accuracy\": \"%d\", \"IAQ\":\"%.2f\", \"Static IAQ\": \"%.2f\"", iaq_accuracy, iaq, static_iaq);
    printf(", \"CO2 equivalent\": \"%.2f\", \"Breath VOC equivalent\": \"%.2f\"", co2_equivalent, breath_voc_equivalent);
    printf(", \"Raw Temperature\": \"%.2f\", \"Temperature\": \"%.2f\"", raw_temp, temp);
    printf(", \"Raw Humidity\": \"%.2f\", \"Humidity\": \"%.2f\",\"Pressure\": \"%.2f\"", raw_humidity, humidity, raw_pressure);
    printf(", \"Raw Gas\": \"%.0f\", \"Gas Percentage\":\"%.2f\"", raw_gas, gas_percentage);
    printf(", \"Stabilization status\": %.0f,\"Run in status\": %.0f,\"Status\": \"%d\"", stabilization_status, run_in_status, bsec_status);
    printf(", \"timestamp\": \"%" PRId64 "\"}\r\n", timestamp_s);
}

void output_ready_pusblish(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float static_iaq, float co2_equivalent, float breath_voc_equivalent,
                  float raw_pressure, float raw_temp, float temp, float raw_humidity, float humidity, float raw_gas, float gas_percentage,
                  float stabilization_status, float run_in_status, bsec_library_return_t bsec_status)
{
    uint16_t crc;
    int64_t timestamp_ms = timestamp / 1000000;
    uint16_t uiaq = iaq * 100;
    uint16_t siaq = static_iaq * 100;
    uint32_t co2 = co2_equivalent * 100;
    uint8_t voc = breath_voc_equivalent * 100;
    uint16_t temperature = temp * 100;
    uint16_t hum = humidity * 100;
    uint32_t pressure = raw_pressure;
    uint8_t payload_idx = PAYLOAD_START_IDX;
    uint8_t data_len = PAYLOAD_SIZE - PAYLOAD_START_IDX;
    uint8_t *ppayload = payload;
    
    memset(payload + payload_idx, 0, data_len);

    struct_pack_into(payload_idx, ppayload, "!BqB2HIB2HI", data_len, timestamp_ms, iaq_accuracy, uiaq, siaq, co2, voc, temperature, hum, pressure);

    crc = crc16_calculate(payload, PAYLOAD_SIZE - 2);
    // ppayload = payload;
    struct_pack_into(PAYLOAD_SIZE - 2, ppayload, "!H", crc);
    ppayload = NULL;
    mqtt_task_pusblish("/sensor/bme680/data", (const char *) payload, PAYLOAD_SIZE);
}

/*!
 * @brief Load binary file from non-volatile memory into buffer. Utility function
 *
 * @param[in,out]   state_buffer    buffer to hold the loaded data
 * @param[in]       n_buffer        size of the allocated buffer
 * @param[in]       filename        name of the file on the NVM
 * @param[in]       offset          offset in bytes from where to start copying
 *                                  to buffer
 * @return          number of bytes copied to buffer or zero on failure
 */
static uint32_t binary_load(uint8_t *b_buffer, uint32_t n_buffer,
                            char *filename, uint32_t offset)
{
    int32_t copied_bytes = 0;
    int8_t rslt = 0;

    FILINFO fileinfo;
    rslt = f_stat(filename, &fileinfo);
    if (rslt != 0)
    {
        perror("stating binary file");
        return 0;
    }

    uint32_t filesize = fileinfo.fsize - offset;

    if (filesize > n_buffer)
    {
        fprintf(stderr, "%s: %d > %d\n", "binary data bigger than buffer", filesize,
                n_buffer);
        return 0;
    }
    else
    {
        FILE *file_ptr;
        file_ptr = fopen(filename, "rb");
        if (!file_ptr)
        {
            perror("fopen");
            return 0;
        }
        fseek(file_ptr, offset, SEEK_SET);
        copied_bytes = fread(b_buffer, sizeof(char), filesize, file_ptr);
        if (copied_bytes == 0)
        {
            fprintf(stderr, "%s\n", "binary_load");
        }
        fclose(file_ptr);
        return copied_bytes;
    }
}

/*!
 * @brief           Load previous library state from non-volatile memory
 *
 * @param[in,out]   state_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to state_buffer
 */
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
{
    // ...
    // Load a previous library state from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no state was available, 
    // otherwise return length of loaded state string.
    // ...
    // int32_t rslt = 0;
    return binary_load(state_buffer, n_buffer, filename_state, 0);
}

/*!
 * @brief           Save library state to non-volatile memory
 *
 * @param[in]       state_buffer    buffer holding the state to be stored
 * @param[in]       length          length of the state string to be stored
 *
 * @return          none
 */
void state_save(const uint8_t *state_buffer, uint32_t length)
{
    // ...
    // Save the string some form of non-volatile memory, if possible.
    // ...
    FILE *state_w_ptr;
    state_w_ptr = fopen(filename_state, "wb");
    fwrite(state_buffer, length, 1, state_w_ptr);
    fclose(state_w_ptr);
}
 
/*!
 * @brief           Load library config from non-volatile memory
 *
 * @param[in,out]   config_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to config_buffer
 */
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
    // ...
    // Load a library config from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no config was available, 
    // otherwise return length of loaded config string.
    // ...
    // int32_t rslt = 0;
    memcpy(config_buffer, bsec_config_iaq, n_buffer);
    return n_buffer;
}

void task_bme680_func(void *pvParameters)
{    
    printf("bsec lib ver%s\n", get_version());
    return_values_init ret;
    bme68x_t bme680_dev = *(bme68x_t *) pvParameters;

    bsec_virtual_sensor_t sensor_list[13] = {
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_STATIC_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_RAW_GAS,
        BSEC_OUTPUT_STABILIZATION_STATUS,
        BSEC_OUTPUT_RUN_IN_STATUS,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
        BSEC_OUTPUT_GAS_PERCENTAGE
    };

    /* Call to the function which initializes the BSEC library
     * Switch on low-power mode and provide no temperature offset */
#if CONFIG_BME680_IAQ_33V_300S_4D
    ret = bsec_iot_init(sensor_list, 13, BSEC_SAMPLE_RATE_ULP, 0.0f, state_load, config_load, bme680_dev);
#else
    ret = bsec_iot_init(sensor_list, 13, BSEC_SAMPLE_RATE_LP, 0.0f, state_load, config_load, bme680_dev);
#endif
    if (ret.bme68x_status)
    {
        /* Could not intialize BME68x */
        printf("Could not intialize BME68x, ret.bme68x_status=%d\r\n", ret.bme68x_status);
        while(1){
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
    else if (ret.bsec_status)
    {
        /* Could not intialize BSEC library */
        printf("Could not intialize BSEC library,ret.bsec_status=%d\r\n", ret.bsec_status);
		while(1){
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    printf("Success init bsec iot\r\n");
    bsec_iot_loop(delay_ms, get_timestamp, output_ready_pusblish, state_save, 10000);
}

void create_task_bme680(void *pvParameters)
{
    ESP_LOGI(TAG, "Create Task BME680");
    esp_read_mac(payload, ESP_MAC_WIFI_STA);
    xTaskCreate(task_bme680_func,
                "Task Bme680",
                BME680_TASK_STK_SIZE,
                pvParameters,
                BME680_TASK_PRIO,
                task_bme680_handle);
}

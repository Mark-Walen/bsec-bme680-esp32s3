#include <string.h>
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp32_s3_driver.h"

//To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many. More means more memory use,
//but less overhead for setting up / finishing transfers. Make sure 240 is dividable by this.
#define PARALLEL_LINES          16
#define ESP_INTR_FLAG_DEFAULT   0

volatile uint8_t adxl345_int_flag;

static __inline void delay_clock(uint32_t ts)
{
    uint32_t start, curr;
 
    __asm__ __volatile__("rsr %0, ccount" : "=r"(start));
    do
    {
        __asm__ __volatile__("rsr %0, ccount" : "=r"(curr));
        
    }while (curr - start <= ts);
}
 
void drv_delay_us(uint32_t us)
{
    while (us--)
    {
        delay_clock(160);//CPU_Freq=160MHz
    }
}

PLATFORM_TICK_COUNT_TYPE get_timestamp(void)
{
    return esp_timer_get_time();
}

void delay_ms(uint32_t period)
{
    vTaskDelay(pdMS_TO_TICKS(period));
}

int gpio_write(uint8_t *reg_data, uint32_t length, void *fp, void *addr)
{
    (void) fp;
    (void) length;
    gpio_num_t gpio_num = *(gpio_num_t *) addr;
    uint32_t level = *reg_data;

    printf("pin_num: %d, pin_level: %d %d\r\n", gpio_num, gpio_get_level(gpio_num), level);
    return gpio_set_level(gpio_num, level);
}

int gpio_read(uint8_t *reg_data, uint32_t length, void *fp, void *addr)
{
    (void) fp;
    (void) length;
    gpio_num_t gpio_num = *(gpio_num_t *) addr;
    int level = gpio_get_level(gpio_num);
    *reg_data = (uint8_t ) level;

    return 0;
}

int i2c_master_init(i2c_port_t i2c_master_port) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

int i2c_master_transmit(uint8_t *tx_buffer, uint32_t len, void *fd, void *addr) {
    i2c_port_t i2c_port = *(i2c_port_t *) fd;
    uint8_t device_address = *(uint8_t *) addr;

    if (i2c_master_write_to_device(i2c_port, device_address, tx_buffer, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS) == ESP_FAIL)
    {
        return DEVICE_E_COM_FAIL;
    }
    
    return DEVICE_OK;
}

int i2c_master_receive(uint8_t *rx_buffer, uint32_t len, void *fd, void *addr) {
    i2c_port_t i2c_port = *(i2c_port_t *) fd;
    uint8_t device_address = *(uint8_t *) addr;

    return i2c_master_read_from_device(i2c_port, device_address, rx_buffer, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

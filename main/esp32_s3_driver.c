#include <string.h>
<<<<<<< HEAD
#include "esp_intr_alloc.h"
=======
#include "driver/i2c.h"
>>>>>>> 501c018 (feat: spi test passed.)
#include "esp_log.h"
#include "esp32_s3_driver.h"

//To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many. More means more memory use,
//but less overhead for setting up / finishing transfers. Make sure 240 is dividable by this.
<<<<<<< HEAD
#define PARALLEL_LINES          16
#define ESP_INTR_FLAG_DEFAULT   0

static void int1_isr_func(void *arg);
static void int2_isr_func(void *arg);

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
 
void delay_us(uint32_t us, void *intf_ptr)
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

PLATFORM_TICK_COUNT_TYPE get_time_us(void)
{
    return esp_timer_get_time()*1000;
=======
#define PARALLEL_LINES 16

uint32_t get_timestamp(void)
{
    return pdMS_TO_TICKS(xTaskGetTickCount());
>>>>>>> 501c018 (feat: spi test passed.)
}

void delay_ms(uint32_t period)
{
    vTaskDelay(pdMS_TO_TICKS(period));
}

<<<<<<< HEAD
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

void gpio_init()
{}

int i2c_master_init(i2c_port_t i2c_master_port) {
=======
int gpio_setPin(void *port, uint8_t pin)
{
    (void) port;
    return gpio_set_level(pin, 1);
}

int gpio_resetPin(void *port, uint8_t pin)
{
    (void) port;
    return gpio_set_level(pin, 0);
}

int spi_write(const uint8_t *tx_buffer, uint32_t len, void *fd)
{
    spi_device_handle_t spi = *(spi_device_handle_t *) fd;
    return spi_transfer(spi, tx_buffer, NULL, len);
}

int spi_read(uint8_t *rx_buffer, uint32_t len, void *fd)
{
    int ret;
    spi_device_handle_t spi = *(spi_device_handle_t *) fd;

    ret = spi_transfer(spi, NULL, rx_buffer, len);
    return ret;
}

int spi_init(spi_device_handle_t *spi) {
    esp_err_t ret;
    spi_bus_config_t buscfg={
        .miso_io_num=SPI_MASTER_MISO,
        .mosi_io_num=SPI_MASTER_MOSI,
        .sclk_io_num=SPI_MASTER_SCLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=PARALLEL_LINES*32
    };
    spi_device_interface_config_t devcfg={
        // .command_bits=0,
        // .address_bits=0,
        // .dummy_bits=0,
        .clock_speed_hz=SPI_MASTER_FREQ_8M,             //Clock out at 8 MHz
        .mode=0,                                        //SPI mode 0
        .spics_io_num=-1,                  //CS pin
        .queue_size=7,                                  //We want to be able to queue 7 transactions at a time
    };

    gpio_config_t nss_pin = {
        .intr_type      = GPIO_INTR_DISABLE,
        .mode           = GPIO_MODE_OUTPUT,
        .pin_bit_mask   = (1ULL<<ADXL362_NSS_PIN),
        .pull_up_en     = true,
    };
    ret = gpio_config(&nss_pin);
    ESP_ERROR_CHECK(ret);

    ret = gpio_set_level(ADXL362_NSS_PIN, 1);
    ESP_ERROR_CHECK(ret);
    
    //Initialize the SPI bus
    ret = spi_bus_initialize(ADXL362_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(ADXL362_HOST, &devcfg, spi);
    ESP_ERROR_CHECK(ret);

    return ret;
}

int spi_transfer(spi_device_handle_t spi, const uint8_t *tx_buffer, uint8_t *rx_buffer, uint16_t len)
{
    esp_err_t ret;
    spi_transaction_t t;
    // uint8_t *rx_buffer = (uint8_t*)malloc(sizeof(uint8_t)*len);

    if (len==0) return 0;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=tx_buffer;               //Data
    t.rx_buffer=rx_buffer;
    t.user=(void*)1;                //D/C needs to be set to 1
    ret = spi_device_polling_transmit(spi, &t);  //Transmit!
    return ret;
}

int i2c_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;

>>>>>>> 501c018 (feat: spi test passed.)
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

<<<<<<< HEAD
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

    if (i2c_master_read_from_device(i2c_port, device_address, rx_buffer, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS) == ESP_FAIL)
    {
        return DEVICE_E_COM_FAIL;
    }
    
    return DEVICE_OK;
=======
int i2c_master_receive(uint8_t slave_address, uint8_t *read_buf, uint16_t length) {
    return i2c_master_read_from_device(I2C_MASTER_NUM, slave_address, read_buf, length, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

int i2c_master_transmit(uint8_t slave_address, uint8_t *write_buf, uint16_t length) {
    return i2c_master_write_to_device(I2C_MASTER_NUM, slave_address, write_buf, length, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
>>>>>>> 501c018 (feat: spi test passed.)
}

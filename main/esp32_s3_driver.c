#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp32_s3_driver.h"

//To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many. More means more memory use,
//but less overhead for setting up / finishing transfers. Make sure 240 is dividable by this.
#define PARALLEL_LINES 16

uint32_t get_timestamp(void)
{
    return pdMS_TO_TICKS(xTaskGetTickCount());
}

void delay_ms(uint32_t period)
{
    vTaskDelay(pdMS_TO_TICKS(period));
}

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

int i2c_master_receive(uint8_t slave_address, uint8_t *read_buf, uint16_t length) {
    return i2c_master_read_from_device(I2C_MASTER_NUM, slave_address, read_buf, length, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

int i2c_master_transmit(uint8_t slave_address, uint8_t *write_buf, uint16_t length) {
    return i2c_master_write_to_device(I2C_MASTER_NUM, slave_address, write_buf, length, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

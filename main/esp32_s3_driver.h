#ifndef __ESP32_S3_DRIVER_H__
#define __ESP32_S3_DRIVER_H__

#include "esp_timer.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "w25qxx/w25qxx.h"
#include "adxl362/adxl362.h"

#define I2C_MASTER_SCL_IO           19      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           18      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                  /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define ADXL362_HOST                SPI2_HOST
#define SPI_MASTER_MISO             GPIO_NUM_13
#define SPI_MASTER_MOSI             GPIO_NUM_11
#define SPI_MASTER_SCLK             GPIO_NUM_12
#define ADXL362_NSS_PIN             GPIO_NUM_15
#define ADXL362_INT2_PIN            GPIO_NUM_9

DEVICE_TICK_COUNT_TYPE get_timestamp(void);
void delay_ms(uint32_t period);
int spi_write(const uint8_t *tx_buffer, uint32_t len, void *fd);
int spi_read(uint8_t *rx_buffer, uint32_t len, void *fd);
int gpio_setPin(void *port, uint8_t pin);
int gpio_resetPin(void *port, uint8_t pin);
int spi_init(spi_device_handle_t *spi);
// int spi_write_byte(void *spi_handle, const uint8_t data, uint16_t len);
int spi_transfer(spi_device_handle_t spi, const uint8_t *tx_buffer, uint8_t *rx_buffer, uint16_t len);
#endif
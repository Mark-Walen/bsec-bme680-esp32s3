#ifndef __ESP32_S3_DRIVER_H__
#define __ESP32_S3_DRIVER_H__

#include "esp_timer.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "w25qxx/w25qxx.h"
#include "adxl362/adxl362.h"
#include "adxl345/adxl345.h"

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
#define ADXL362_INT1_PIN            GPIO_NUM_14
#define ADXL345_INT2_PIN            GPIO_NUM_17
#define ADXL345_NSS_PIN             GPIO_NUM_16

uint32_t get_timestamp(void);
void delay_ms(uint32_t period);
void gpio_init(void);
int gpio_write(uint8_t *level, uint32_t length, void *fp, void *addr);
int gpio_read(uint8_t *level, uint32_t length, void *fp, void *addr);
int spi_write(uint8_t *tx_buffer, uint32_t len, void *fd, void *addr);
int spi_read(uint8_t *rx_buffer, uint32_t len, void *fd, void *addr);
int spi_init(spi_device_handle_t *spi);
// int spi_write_byte(void *spi_handle, const uint8_t data, uint16_t len);
int spi_transfer(spi_device_handle_t spi, const uint8_t *tx_buffer, uint8_t *rx_buffer, uint16_t len);
int i2c_master_init(i2c_port_t i2c_master_port);
int i2c_master_transmit(uint8_t *tx_buffer, uint32_t len, void *fd, void *addr);
int i2c_master_receive(uint8_t *rx_buffer, uint32_t len, void *fd, void *addr);
#endif
#ifndef TMP117_H
#define TMP117_H

#include "esp_err.h"

// I2C address of TMP117 sensor
#define TMP117_ADDR 0x48

// TMP117 Register Addresses
#define TMP117_REG_TEMP        0x00
#define TMP117_REG_CONFIG      0x01
#define TMP117_REG_HIGH_LIMIT  0x02
#define TMP117_REG_LOW_LIMIT   0x03

#define TMP117_ADDR             0x48
#define I2C_MASTER_NUM          I2C_NUM_0

// Function declarations
esp_err_t tmp117_i2c_init(void);
esp_err_t tmp117_read_raw(int16_t *raw_out);
esp_err_t tmp117_read_temperature_c(float *temp_c);
esp_err_t tmp117_set_alert_limits(float low_c, float high_c);
esp_err_t tmp117_sleep();
esp_err_t tmp117_wake();

#endif // TMP117_H
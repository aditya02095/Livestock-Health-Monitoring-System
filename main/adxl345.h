#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// ADXL345 I2C address and control registers
#define ADXL345_ADDR              0x53
#define ADXL345_REG_POWER_CTL     0x2D
#define ADXL345_REG_BW_RATE       0x2C
#define ADXL345_REG_DATA_FORMAT   0x31
#define ADXL345_REG_DATA_START    0x32

// Initializes the I2C peripheral and configures the ADXL345 for measurement mode
void adxl345_i2c_init(void);

// Puts sensor in measurement mode
void adxl345_init(void);

// Reads X, Y, Z acceleration data from the ADXL345 sensor with filtering and rounding
bool adxl345_read(int16_t *x, int16_t *y, int16_t *z);

// Sends the ADXL345 into sleep mode (low power)
void adxl345_sleep(void);

// Sends the ADXL345 into standby mode (lowest power)
void adxl345_standby(void);

// Wakes the ADXL345 from sleep and sets measurement mode
void adxl345_wake(void);

// I2C low-level access helpers
uint8_t adxl345_read_register(uint8_t reg);
void adxl345_write_register(uint8_t reg, uint8_t value);
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define DS3231_REG_CONTROL  0x0E
#define DS3231_REG_STATUS   0x0F
#define DS3231_I2C_NUM  I2C_NUM_0
#define DS3231_SDA      GPIO_NUM_21
#define DS3231_SCL      GPIO_NUM_22
#define DS3231_ADDR     0x68

void ds3231_i2c_init(void);
void ds3231_init(void);
bool ds3231_read_time(uint8_t *hr, uint8_t *min, uint8_t *sec, uint8_t *day, uint8_t *mon, uint8_t *year);
esp_err_t ds3231_set_time(uint8_t hour, uint8_t min, uint8_t sec, uint8_t day, uint8_t month, uint8_t year);

void ds3231_sleep(void);
void ds3231_wake(void);
bool ds3231_sync_with_ntp(void);
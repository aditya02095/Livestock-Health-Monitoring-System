#include "ds3231.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include <time.h>
#include <string.h> 
#include <stdbool.h> 

static uint8_t bcd_to_decimal(uint8_t val) {
    return ((val >> 4) * 10 + (val & 0x0F));
}

static esp_err_t ds3231_read(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(DS3231_I2C_NUM, DS3231_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(1000));
}

void ds3231_i2c_init(void) {
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = DS3231_SDA,
        .scl_io_num = DS3231_SCL,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,  // [POWER OPTIMIZATION] Disabled internal pullups
        .scl_pullup_en = GPIO_PULLUP_DISABLE,  // [POWER OPTIMIZATION] Disabled internal pullups
        .master.clk_speed = 400000
    };
    i2c_param_config(DS3231_I2C_NUM, &config);
    i2c_driver_install(DS3231_I2C_NUM, config.mode, 0, 0, 0);
}

void ds3231_init(void) {
    uint8_t dummy;
    i2c_master_write_read_device(DS3231_I2C_NUM, DS3231_ADDR, (uint8_t[]){0x00}, 1, &dummy, 1, pdMS_TO_TICKS(1000));
    ds3231_wake();
}


esp_err_t ds3231_set_time(uint8_t hour, uint8_t min, uint8_t sec,
                          uint8_t day, uint8_t month, uint8_t year) {
    uint8_t data[7];

    data[0] = ((sec / 10) << 4) | (sec % 10);      // Seconds
    data[1] = ((min / 10) << 4) | (min % 10);      // Minutes
    data[2] = ((hour / 10) << 4) | (hour % 10);    // Hours
    data[3] = 0x01;                                // Day of week (1 = Monday)
    data[4] = ((day / 10) << 4) | (day % 10);      // Day
    data[5] = ((month / 10) << 4) | (month % 10);  // Month
    data[6] = ((year / 10) << 4) | (year % 10);    // Year (last two digits)

    uint8_t full_data[8];
    full_data[0] = 0x00; // Start from register 0x00
    memcpy(&full_data[1], data, 7);
    return i2c_master_write_to_device(I2C_NUM_0, DS3231_ADDR, full_data, sizeof(full_data),
                                  pdMS_TO_TICKS(1000));

}

bool ds3231_read_time(uint8_t *hr, uint8_t *min, uint8_t *sec, uint8_t *day, uint8_t *mon, uint8_t *year) {
    uint8_t data[7];
    if (ds3231_read(0x00, data, 7) == ESP_OK) {
        *sec = bcd_to_decimal(data[0]);
        *min = bcd_to_decimal(data[1]);
        *hr  = bcd_to_decimal(data[2]);
        *day = bcd_to_decimal(data[4]);
        *mon = bcd_to_decimal(data[5] & 0x1F);
        *year= bcd_to_decimal(data[6]);
        return true;
    }
    return false;
}

// This function is used to put the sensor in sleep mode
void ds3231_sleep(void) {
    /* [POWER OPTIMIZATION] Disable all unnecessary functions */
    uint8_t ctrl = 0x1C; // BBSQW = 0, INTCN = 1, CONV = 0, RS2/RS1 = 0 (disable SQW)
    i2c_master_write_to_device(DS3231_I2C_NUM, DS3231_ADDR,
                               (uint8_t[]){DS3231_REG_CONTROL, ctrl}, 2, pdMS_TO_TICKS(100));
    ESP_LOGI("DS3231", "RTC in lowest power mode");
}

// This function is used to wake the sensor from the sleep mode.
void ds3231_wake(void) {
    /* [POWER OPTIMIZATION] Only enable what's needed */
    uint8_t ctrl = 0x00; // Default configuration
    i2c_master_write_to_device(DS3231_I2C_NUM, DS3231_ADDR,
                               (uint8_t[]){DS3231_REG_CONTROL, ctrl}, 2, pdMS_TO_TICKS(100));
    ESP_LOGI("DS3231", "RTC woken from sleep");
}

// This function is sets the timestamp time to internet time

bool ds3231_sync_with_ntp() {
    ESP_LOGI("DS3231", "Starting SNTP for NTP sync...");
    
    // Configure SNTP
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.nist.gov"); // Secondary server
    esp_sntp_init();

    // Set timezone (example: IST)
    setenv("TZ", "IST-5:30", 1);
    tzset();

    // Wait for time sync
    time_t now = 0;
    struct tm timeinfo = {0};
    int retry = 0;
    const int max_retries = 15;
    bool sync_success = false;

    while (timeinfo.tm_year < (2022 - 1900) && ++retry < max_retries) {
        ESP_LOGI("DS3231", "Waiting for time sync... (%d/%d)", retry, max_retries);
        vTaskDelay(pdMS_TO_TICKS(2000));
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    if (timeinfo.tm_year >= (2022 - 1900)) {
        ESP_LOGI("DS3231", "NTP time: %s", asctime(&timeinfo));

        // Set RTC time (adjust based on your ds3231_set_time implementation)
        // Note: tm_mon is 0-11, tm_year is years since 1900
        ds3231_set_time(
            timeinfo.tm_hour,
            timeinfo.tm_min,
            timeinfo.tm_sec,
            timeinfo.tm_mday,
            timeinfo.tm_mon + 1,
            timeinfo.tm_year % 100
        );

        ESP_LOGI("DS3231", "RTC synced with NTP time.");
        sync_success = true;
    } else {
        ESP_LOGW("DS3231", "NTP sync failed. Time not updated.");
    }

    esp_sntp_stop();
    return sync_success;
}
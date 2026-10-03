#include "adxl345.h"
#include "driver/i2c.h"
#include "freertos/task.h"
#include <math.h>
#include <stdint.h>

#define I2C_PORT I2C_NUM_0

// Implementation of non-static functions declared in header
uint8_t adxl345_read_register(uint8_t reg) {
    uint8_t data = 0;
    int retry_count = 3;
    
    while (retry_count--) {
        if (i2c_master_write_read_device(I2C_PORT, ADXL345_ADDR, &reg, 1, &data, 1, pdMS_TO_TICKS(100)) == ESP_OK) {
            return data;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return 0;
}

void adxl345_write_register(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    int retry_count = 3;
    
    while (retry_count--) {
        if (i2c_master_write_to_device(I2C_PORT, ADXL345_ADDR, data, sizeof(data), pdMS_TO_TICKS(100)) == ESP_OK) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// I2C and sensor initialization
void adxl345_i2c_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,  // Disabled internal pullups for power optimization
        .scl_pullup_en = GPIO_PULLUP_DISABLE,  // Disabled internal pullups for power optimization
        .master.clk_speed = 400000,
        .clk_flags = 0
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);

    adxl345_init();
}

// Puts the sensor into measurement mode with low power output rate
void adxl345_init(void) {
    adxl345_write_register(ADXL345_REG_BW_RATE, 0x08);       // 2 Hz output rate
    adxl345_write_register(ADXL345_REG_POWER_CTL, 0x08);     // Enable measurement mode
    vTaskDelay(pdMS_TO_TICKS(10));                           // Allow time to stabilize
}

// Read X, Y, Z acceleration values (rounded + filtered for ~30cm movement)
bool adxl345_read(int16_t *x_out, int16_t *y_out, int16_t *z_out) {
    static float filtered_x = 0, filtered_y = 0, filtered_z = 0;
    const float alpha = 0.95f;         // Strong low-pass filter for noise
    const int round_step = 100;        // Each step ≈ 30cm

    uint8_t data[6];
    if (i2c_master_write_read_device(
        I2C_PORT, ADXL345_ADDR, (uint8_t[]){ADXL345_REG_DATA_START}, 1, data, 6, pdMS_TO_TICKS(100)) != ESP_OK) {
        return false;
    }

    int16_t x_raw = (int16_t)((data[1] << 8) | data[0]);
    int16_t y_raw = (int16_t)((data[3] << 8) | data[2]);
    int16_t z_raw = (int16_t)((data[5] << 8) | data[4]);

    // Apply low-pass filtering
    filtered_x = alpha * filtered_x + (1 - alpha) * x_raw;
    filtered_y = alpha * filtered_y + (1 - alpha) * y_raw;
    filtered_z = alpha * filtered_z + (1 - alpha) * z_raw;

    // Round to nearest 100 units
    *x_out = ((int)filtered_x / round_step);
    *y_out = ((int)filtered_y / round_step);
    *z_out = ((int)filtered_z / round_step);

    return true;
}

// Put sensor into sleep mode (low power but still retains config)
void adxl345_sleep(void) {
    uint8_t power_ctl = adxl345_read_register(ADXL345_REG_POWER_CTL);
    power_ctl &= ~(1 << 3); // Clear MEASURE bit
    power_ctl |= (1 << 2);  // Set SLEEP bit
    adxl345_write_register(ADXL345_REG_POWER_CTL, power_ctl);
}

// Put sensor into standby (lowest power, loses config if not saved)
void adxl345_standby(void) {
    adxl345_write_register(ADXL345_REG_POWER_CTL, 0x00); // Standby mode
    vTaskDelay(pdMS_TO_TICKS(10));
}

// Wake sensor from sleep and reinitialize measurement mode
void adxl345_wake(void) {
    adxl345_write_register(ADXL345_REG_POWER_CTL, 0x08); // Enable measurement mode again
    vTaskDelay(pdMS_TO_TICKS(10));
    adxl345_init();
}
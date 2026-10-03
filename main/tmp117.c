#include "tmp117.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TIMEOUT_MS       1000
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TX_BUF_DISABLE   0

esp_err_t tmp117_sleep() {
    uint8_t config_data[3];

    // Read current config
    config_data[0] = TMP117_REG_CONFIG;
    esp_err_t ret = i2c_master_write_read_device(
        I2C_MASTER_NUM, TMP117_ADDR, &config_data[0], 1,
        &config_data[1], 2, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) return ret;

    // Modify config to set shutdown mode (bits 10:9 = 01)
    uint16_t config_val = ((uint16_t)config_data[1] << 8) | config_data[2];
    config_val &= ~(0b11 << 9);     // Clear bits 10:9
    config_val |= (0b01 << 9);      // Set shutdown mode

    // Write back config
    config_data[0] = TMP117_REG_CONFIG;
    config_data[1] = (config_val >> 8) & 0xFF;
    config_data[2] = config_val & 0xFF;

    return i2c_master_write_to_device(
        I2C_MASTER_NUM, TMP117_ADDR, config_data, 3, pdMS_TO_TICKS(100));
}

esp_err_t tmp117_i2c_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t tmp117_write_reg(uint8_t reg, uint16_t val) {
    uint8_t buf[3] = {reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF)};
    return i2c_master_write_to_device(I2C_MASTER_NUM, TMP117_ADDR, buf, 3, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

static esp_err_t tmp117_read_reg(uint8_t reg, uint16_t *val) {
    uint8_t buf[2];
    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, TMP117_ADDR, &reg, 1, buf, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    if (err == ESP_OK) {
        *val = ((uint16_t)buf[0] << 8) | buf[1];
    }
    return err;
}

esp_err_t tmp117_read_raw(int16_t *raw_out) {
    return tmp117_read_reg(TMP117_REG_TEMP, (uint16_t *)raw_out);
}

esp_err_t tmp117_read_temperature_c(float *temp_c) {
    int16_t raw;
    esp_err_t err = tmp117_read_raw(&raw);
    if (err == ESP_OK) {
        *temp_c = raw * 0.0078125f;  // TMP117 LSB = 7.8125 m°C
    }
    return err;
}

esp_err_t tmp117_set_alert_limits(float low_c, float high_c) {
    uint16_t low = (uint16_t)(low_c / 0.0078125f);
    uint16_t high = (uint16_t)(high_c / 0.0078125f);
    ESP_ERROR_CHECK(tmp117_write_reg(TMP117_REG_LOW_LIMIT, low));
    ESP_ERROR_CHECK(tmp117_write_reg(TMP117_REG_HIGH_LIMIT, high));
    return ESP_OK;
}

esp_err_t tmp117_wake() {
    // Set TMP117 to continuous conversion mode (0x0220 = continuous)
    uint8_t data[3];
    data[0] = TMP117_REG_CONFIG;
    data[1] = 0x02;  // MSB
    data[2] = 0x20;  // LSB

    return i2c_master_write_to_device(I2C_MASTER_NUM, TMP117_ADDR,
                                      data, 3, pdMS_TO_TICKS(100));
}
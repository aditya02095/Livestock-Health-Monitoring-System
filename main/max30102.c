#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "max30102.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_PORT I2C_NUM_0
#define I2C_TIMEOUT_MS 1000
#define MAX30102_ADC_MASK 0x3FFFF

/*
This declaration is made as to not get an error for not having declared this function.
This function is used in the max30102_init() function.
*/
void i2c_master_init();

/*
This function declares the max30102 sensor that we had declared in the header file.
Sets up the SpO₂ mode with:              based on what we have mentioned in the header file.
    18-bit resolution
    20Hz sampling rate (50ms)
    ADC full-scale range = 16384 nA

Sets LED brightness:              based on what we have mentioned in the header file.
    Red LED ~25.4 mA
    IR LED ~51.0 mA

This function follows modular design by encapsulating all init logic in one place,
and supports maintainability as per coding best practices.
*/
void max30102_init(void)
{
    // 1. Bring sensor out of shutdown
    max30102_write_register(REG_MODE_CONFIG, 0x00);  // Clear SHDN bit
    vTaskDelay(pdMS_TO_TICKS(10));  // Allow sensor to wake up

    // 2. Check for power ready (try 10 times)
    int retry = 0;
    bool powered_up = false;
    while (retry++ < 10) {
        uint8_t status = max30102_read_register(REG_INTR_STATUS_1);
        if (status != 0xFF) {  // 0xFF means read failed (common error)
            powered_up = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (!powered_up) {

        // Delete and reinit I2C driver
        i2c_driver_delete(I2C_PORT);
        vTaskDelay(pdMS_TO_TICKS(100));

        i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = 21,
            .scl_io_num = 22,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = 100000,
        };
        ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
        ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));
        vTaskDelay(pdMS_TO_TICKS(100));

        // Try software reset
        max30102_write_register(REG_MODE_CONFIG, 0x40);  // Reset
        vTaskDelay(pdMS_TO_TICKS(100));

        // Wait for reset bit to clear
        retry = 0;
        while (max30102_read_register(REG_MODE_CONFIG) & 0x40) {
            vTaskDelay(pdMS_TO_TICKS(10));
            if (++retry > 100) {
                return;
            }
        }
    }

    // 3. Verify part ID
    uint8_t part_id = max30102_read_register(0xFF);
    if (part_id != 0x15) {
		return;
    }

    // 4. Reset FIFO pointers
    max30102_write_register(REG_FIFO_WR_PTR, 0x00);
    max30102_write_register(REG_OVF_COUNTER, 0x00);
    max30102_write_register(REG_FIFO_RD_PTR, 0x00);

    // 5. FIFO config (sample avg = 2, no rollover)
    max30102_write_register(REG_FIFO_CONFIG, 0x20);  // 0b00100000

    // 6. SpO2 config (range 16384nA, 411us pulse, 20Hz sample rate)
    max30102_write_register(REG_SPO2_CONFIG, 
                            (ADC_RANGE_16384 << 5) | 
                            (SAMPLE_RATE_20HZ << 2) | 
                            LED_PULSE_WIDTH_411US);

    // 7. LED amplitudes
    max30102_write_register(REG_LED1_PA, RED_LED_CURRENT);
    max30102_write_register(REG_LED2_PA, IR_LED_CURRENT);

    // 8. Set mode to SpO2
    max30102_write_register(REG_MODE_CONFIG, MODE_SPO2);

    // 9. Clear any pending interrupts
    max30102_read_register(REG_INTR_STATUS_1);
    max30102_read_register(REG_INTR_STATUS_2);
}

/*
This function reads one sample of data from the Red and IR from the sensor's FIFO.
It reads it from the following pattern:
    3 bytes → Red LED reading
    3 bytes → IR LED reading
This function then returns a boolean value that indicates if the data has been sent successfully.

Implements structured data extraction and keeps logic isolated, aligning with modular design principles.
*/
bool max30102_read_fifo(uint32_t *red, uint32_t *ir)
{
    static int error_count = 0;

    // Check FIFO status
    uint8_t status = max30102_read_register(REG_INTR_STATUS_1);
    if (status & INT_FIFO_FULL_MASK) {
        max30102_reset_fifo();
    }

    // Read 6 bytes: 3 bytes RED + 3 bytes IR
    uint8_t buffer[6];
    if (!max30102_read_multiple(REG_FIFO_DATA, buffer, 6)) {
        error_count++;

        if (error_count >= 3) {
            max30102_init();  // Hard reset
            error_count = 0;
        }
        return false;
    }

    error_count = 0;

    *red = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
    *red &= MAX30102_ADC_MASK;

    *ir  = ((uint32_t)buffer[3] << 16) | ((uint32_t)buffer[4] << 8) | buffer[5];
    *ir  &= MAX30102_ADC_MASK;

    return true;
}


/*
This function sends 1 Byte data to any register that we choose. This is used for writing in registers for I2C communication.

Supports abstraction and reuse — prevents redundant I2C setup logic in higher-level code.
Improves readability and maintainability per coding standards.
*/
void max30102_write_register(uint8_t reg, uint8_t value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    /*esp_err_t ret = */i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));   //This variable is declared to be used as a log checker.
    i2c_cmd_link_delete(cmd);
}

/*
This function is designed to read registers for checking the state of the sensor in some cases.
This may include checking the overflow condition for the FIFO, or checking if the LED are up to power.

Useful for implementing polling-based event checks. Abstracting it helps prevent code duplication,
and aligns with the standard of separating low-level I2C from application logic.
*/
uint8_t max30102_read_register(uint8_t reg)
{
    uint8_t value = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &value, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    /*esp_err_t ret = */i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    return value;
}

/*
This function reads data from the FIFO data in the above function.
It then gives the data to the buffer array.

Implements bulk I2C read logic and helps reduce overhead vs. multiple single-byte reads.
Enables reuse in any part of the project needing structured, multi-byte sensor data.
*/
bool max30102_read_multiple(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    if (!buffer || length == 0) return false;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_I2C_ADDR << 1) | I2C_MASTER_READ, true);

    if (length > 1) {
        i2c_master_read(cmd, buffer, length - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buffer + length - 1, I2C_MASTER_NACK);

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        return false;
    }
    return true;
}

/*
This function changes is made to reset the FIFO after every read where the spike is calculated.
*/

void max30102_reset_fifo()
{
    max30102_write_register(REG_FIFO_WR_PTR, 0x00);
    max30102_write_register(REG_OVF_COUNTER, 0x00);
    max30102_write_register(REG_FIFO_RD_PTR, 0x00);
}

/*
This function is made so that the sensor enters the power efficiency mode.
This puts the sensor in shutdown mode and can be woken up any-time.
*/
void max30102_sleep(void)
{
    max30102_write_register(REG_MODE_CONFIG, 0x80); // Shutdown mode
}

/*
This function wakes up the sensor after it has been shutdown and allows it to stabilize.
*/
void max30102_wake(void)
{
    max30102_write_register(REG_MODE_CONFIG, 0x00); // Wake from shutdown
    vTaskDelay(pdMS_TO_TICKS(100)); // Let it stabilize
    max30102_init(); // Fully reconfigure sensor again
}

/*
This function fully powers down the MAX30102 sensor to minimize power consumption
during deep sleep. More aggressive than regular sleep mode.
*/
void max30102_power_down(void)
{
    // 1. Shutdown the sensor (sets bit 7 of MODE_CONFIG)
    max30102_write_register(REG_MODE_CONFIG, 0x80);
    
    // 2. Turn off both LEDs completely
    max30102_write_register(REG_LED1_PA, 0x00);  // Red LED off
    max30102_write_register(REG_LED2_PA, 0x00); // IR LED off
    
    // 3. Disable all interrupts
    max30102_write_register(REG_INTR_ENABLE_1, 0x00);
    max30102_write_register(REG_INTR_ENABLE_2, 0x00);
    
    // 4. Reset FIFO to minimize any active circuitry
    max30102_reset_fifo();
    
    vTaskDelay(pdMS_TO_TICKS(10)); // Allow time to power down
}
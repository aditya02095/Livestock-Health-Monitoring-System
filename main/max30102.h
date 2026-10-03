#ifndef MAX30102_H
#define MAX30102_H

#include <stdint.h>
#include <stdbool.h>

/*
This header file defines the I2C address, register addresses, bit masks, and configuration values
used for initializing and communicating with the MAX30102 pulse oximeter and heart-rate sensor.
*/

#define MAX30102_I2C_ADDR 0x57  // I2C 7-bit address

// Register addresses
#define REG_INTR_STATUS_1 0x00
#define REG_INTR_STATUS_2 0x01
#define REG_INTR_ENABLE_1 0x02
#define REG_INTR_ENABLE_2 0x03
#define REG_FIFO_WR_PTR   0x04
#define REG_OVF_COUNTER   0x05
#define REG_FIFO_RD_PTR   0x06
#define REG_FIFO_DATA     0x07
#define REG_FIFO_CONFIG   0x08
#define REG_MODE_CONFIG   0x09
#define REG_SPO2_CONFIG   0x0A
#define REG_LED1_PA       0x0C  // RED LED current control
#define REG_LED2_PA       0x0D  // IR LED current control
#define REG_TEMP_INT      0x1F
#define REG_TEMP_FRAC     0x20

// Mode configuration
#define MODE_SPO2         0x03  // Enables SpO2 mode only

// Variable to make code more understandable
#define MAX30102_ADC_MASK 0x3FFFF

// SPO2 Config values
#define LED_PULSE_WIDTH_411US 0x03  // 411us pulse width = 18-bit resolution
#define ADC_RANGE_16384       0x03  // ADC range: 0–16384nA
#define SAMPLE_RATE_20HZ      0x03  // Sampling rate = 20Hz

/* 
	LED currents (in hexadecimal corresponding to datasheet values)
	0xFF ~ 51.0mA, 0x55 ~ 25.4mA — values tuned for proper detection
	Increase these values for current incase you are getting the same readings back to back.
	Increasing the values a lot will also cause increased values.
*/
#define RED_LED_CURRENT  0x5F  // For SpO2 numerator (AC red)
#define IR_LED_CURRENT   0x7F  // For SpO2 denominator and PPG waveform

// Interrupt masks
#define INT_FIFO_FULL_MASK   0x80
#define INT_PWR_RDY_MASK     0x01

/*
Sample window configuration for processing and averaging.
SAMPLES_PER_WINDOW = SAMPLE_RATE_HZ * window duration in seconds.
*/
#define SAMPLE_RATE_HZ 12
#define SAMPLE_WINDOW_SECONDS 4                     // This value decides the time period after which you get accurate readings. The readings before this time have to be ignored.
#define SAMPLES_PER_WINDOW 72
#define SAMPLE_INTERVAL_MS (1000 / SAMPLE_RATE_HZ)  // Sampling every 50ms

// ADC reading saturation limit (safety cap for signal processing)
#define MAX_IR_LIMIT 160000

/*
Maximum number of peaks to detect in one processing window.
This avoids array overflows and constrains BPM range.
*/
#define MAX_PEAKS_IN_WINDOW 20

/*
Minimum number of samples between two peaks to reject noise and duplicate detections.
20Hz = 50ms/sample → 6 samples = 300ms
*/
#define MIN_PEAK_DISTANCE_SAMPLES 10

/*
If no valid beat is detected within this timeout, reset BPM to zero.
Ensures stale values aren't used.
*/
#define SIGNAL_TIMEOUT_SECONDS 5

/*
Exponential moving average smoothing for BPM stability.
Higher value = more responsive, lower = smoother.
*/
#define BPM_SMOOTHING_FACTOR 0.7f

// Minimum DC value required to consider signal usable for SpO2 calculation
#define SIGNAL_QUALITY_THRESHOLD 50000

// Acceptable physiological ranges for filtering invalid BPM/SpO2
#define MIN_BPM 40
#define MAX_BPM 180
#define MIN_SPO2 80
#define MAX_SPO2 100

#define MIN_PEAK_HEIGHT 1000      // Minimum amplitude difference to consider a peak
/*
	The following 2 macros decides the spo2 calculation formula.
	This is to be changed accordingly, if you are not getting accurate values.
	This value shifts the calculations accordingly in the form of a linear graph.
*/
#define SPO2_CALIBRATION_A 104.0f // SpO2 calibration parameter A
#define SPO2_CALIBRATION_B 17f  // SpO2 calibration parameter B

#define WAKEUP_PIN GPIO_NUM_0

/*
Function declarations for initializing and communicating with the MAX30102.
These follow modular design principles and are implemented in max30102.c
These have been made to avoid declaration issues in the other programmes.
*/
void max30102_init(void);
bool max30102_read_fifo(uint32_t *red, uint32_t *ir);
void max30102_write_register(uint8_t reg, uint8_t value);
uint8_t max30102_read_register(uint8_t reg);
bool max30102_read_multiple(uint8_t reg, uint8_t *buffer, uint8_t length);
void max30102_reset_fifo();
void max30102_sleep(void);
void max30102_wake(void);
void max30102_power_down(void);  // Fully powers down the sensor
void data_conversion_to_readable(void *arg);

#endif // MAX30102_H
#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include "cJSON.h"
#include <stdint.h>
#include <stdio.h>

#define RUN_DURATION_SEC    15
#define SLEEP_DURATION_SEC  60
#define MAX_READINGS_PER_BOOT (RUN_DURATION_SEC / 2)

typedef struct {
    float temperature;
    float bpm;
    float spo2;
    uint8_t hour, minute, second;
    uint8_t day, month, year;
    int16_t accel_x, accel_y, accel_z;
} reading_t;

typedef struct {
    reading_t readings[MAX_READINGS_PER_BOOT];
    int num_readings;
    float temperature;
    float bpm;
    float spo2;
    uint8_t hour, minute, second;
    uint8_t day, month, year;
    int16_t accel_x, accel_y, accel_z;
} sensor_data_t;

extern sensor_data_t global_sensor_data;

char *create_sensor_json(float temperature, int heart_rate, float ax, float ay, float az);
char *create_readings_json(const sensor_data_t *data);

#endif  // SENSOR_DATA_H
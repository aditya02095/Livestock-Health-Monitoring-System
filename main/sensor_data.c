#include "sensor_data.h"
#include "cJSON.h"
#include <stdio.h>
#include <stdlib.h>  // for free()
#include <string.h>  // for memset()

// Global sensor struct (used by other files)
sensor_data_t global_sensor_data = {
    .num_readings = 0  // initialize the reading counter
};

// Create a JSON string from a single reading
char *create_readings_json(const sensor_data_t *boot_data) {
    cJSON *root = cJSON_CreateArray();
    if (!root) return NULL;

    for (int i = 0; i < boot_data->num_readings; i++) {
        const reading_t *r = &boot_data->readings[i];
        cJSON *obj = cJSON_CreateObject();
        if (!obj) { cJSON_Delete(root); return NULL; }

        cJSON_AddNumberToObject(obj, "temp",  r->temperature);
        cJSON_AddNumberToObject(obj, "bpm",   r->bpm);
        cJSON_AddNumberToObject(obj, "spo2",  r->spo2);

        char time_buf[16];
        snprintf(time_buf, sizeof(time_buf), "%02d:%02d:%02d", r->hour, r->minute, r->second);
        cJSON_AddStringToObject(obj, "time", time_buf);

        char date_buf[16];
        snprintf(date_buf, sizeof(date_buf), "%02d-%02d-20%02d", r->day, r->month, r->year);
        cJSON_AddStringToObject(obj, "date", date_buf);

        cJSON_AddNumberToObject(obj, "x", r->accel_x);
        cJSON_AddNumberToObject(obj, "y", r->accel_y);
        cJSON_AddNumberToObject(obj, "z", r->accel_z);

        cJSON_AddItemToArray(root, obj);
    }

    char *s = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return s;
}
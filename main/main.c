#include "esp_system.h"
#define ENABLE_LOGS 1

#if ENABLE_LOGS
    #define LOGI(tag, ...) ESP_LOGI(tag,__VA_ARGS__)
    #define LOGE(tag, ...) ESP_LOGE(tag,__VA_ARGS__)
    #define LOGW(tag, ...) ESP_LOGW(tag,__VA_ARGS__)
#else
    #define LOGI(tag, ...)
    #define LOGE(tag, ...)
    #define LOGW(tag, ...)
#endif


#include "adxl345.h"
#include "cJSON.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "ds3231.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "max30102.h"
#include "tmp117.h"
#include "sensor_data.h"
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "mqtt_client.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_sleep.h"
#include "esp_sntp.h"
#include "esp_task_wdt.h"
#include <time.h>
#include <stdbool.h>
#include "freertos/event_groups.h"

#define CONFIG_WIFI_SSID       "ShalakaDevZone"
#define CONFIG_WIFI_PASSWORD   "HHK92HVK"
#define WIFI_CONNECTED_BIT BIT0

#define MAX_SAVED_DATA 5

#define WIFI_RETRY_INTERVAL_MS 30000
#define MQTT_RETRY_INTERVAL_MS 30000
#define MAX_RETRY_ATTEMPTS 5
#define MQTT_CONNECTED_BIT BIT1

RTC_DATA_ATTR sensor_data_t saved_data[MAX_SAVED_DATA];
RTC_DATA_ATTR int boot_count = 0;
RTC_DATA_ATTR int mqtt_publish_count = 0;

static bool mqtt_connected = false;

time_t task_start_time;

static EventGroupHandle_t wifi_event_group;


static const char *TAG = "MAIN";
esp_mqtt_client_handle_t client = NULL;
static SemaphoreHandle_t sleep_semaphore;

typedef struct {
    char ssid[32];
    char password[64];
    int sampling_interval_ms;  // e.g., 2000 for 2 seconds
} app_config_t;

RTC_DATA_ATTR app_config_t app_config;

 void isolate_unused_rtc_gpio(void) {
    rtc_gpio_isolate(GPIO_NUM_0);
    rtc_gpio_isolate(GPIO_NUM_2);
    rtc_gpio_isolate(GPIO_NUM_1); 
    rtc_gpio_isolate(GPIO_NUM_3);  
    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_13);
    rtc_gpio_isolate(GPIO_NUM_15);
    rtc_gpio_isolate(GPIO_NUM_25);
    rtc_gpio_isolate(GPIO_NUM_26);
    rtc_gpio_isolate(GPIO_NUM_27);
    rtc_gpio_isolate(GPIO_NUM_33);
    rtc_gpio_isolate(GPIO_NUM_34);
    rtc_gpio_isolate(GPIO_NUM_35);
    rtc_gpio_isolate(GPIO_NUM_36);
    rtc_gpio_isolate(GPIO_NUM_39);
}

void enter_deep_sleep(uint64_t sleep_time_us) {
    vTaskDelay(pdMS_TO_TICKS(100));  // Allow tasks to settle

    // Put all sensors into low power mode
    max30102_sleep();
    tmp117_sleep();
    adxl345_standby();
    ds3231_sleep();

    // Delete I2C driver if it's still active
    if (i2c_driver_delete(I2C_NUM_0) != ESP_OK) {
        LOGW(TAG, "Failed to delete I2C driver");
    }

    // Set wakeup source and power domain config
    esp_sleep_enable_timer_wakeup(sleep_time_us);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO);
    esp_deep_sleep_disable_rom_logging();
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);

    fflush(stdout);
    esp_deep_sleep_start();

    // These won't run unless deep sleep fails
    max30102_wake();
    tmp117_wake();
    adxl345_wake();
    ds3231_wake();
}

void check_wakeup_reason() {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    switch(wakeup_reason) {
        case ESP_SLEEP_WAKEUP_TIMER:
            break;
        case ESP_SLEEP_WAKEUP_EXT0:
            break;
        case ESP_SLEEP_WAKEUP_EXT1:
            break;
        default:
            break;
    }
}

// TMP117 task with finite iterations and semaphore signaling 
void tmp117_task(void *arg) {
    float temp;

    while (true) {
        // Exit if time exceeded
        time_t now;
        time(&now);
        if (now - task_start_time >= RUN_DURATION_SEC) {
            break;
        }

        if (tmp117_read_temperature_c(&temp) == ESP_OK) {
            global_sensor_data.temperature = temp;
        }

        vTaskDelay(pdMS_TO_TICKS(app_config.sampling_interval_ms));  // Delay between readings
    }

    xSemaphoreGive(sleep_semaphore);  // Signal task complete
    vTaskDelete(NULL);
}

/*
This function collects samples of RED and IR LED data from the MAX30102 sensor.
It performs the following steps:
    - Uses circular buffer for continuous real-time processing
    - Smooths IR data using a moving average filter
    - Detects peaks using slope-based logic (local maxima detection)
    - Calculates BPM usi`ng time between detected peaks
    - Calculates SpO₂ using the AC/DC ratio method
    - Filters invalid or noisy data using minimum signal quality checks
*/
void data_conversion_to_readable(void *arg)
{
    uint32_t red_buffer[SAMPLES_PER_WINDOW] = {0};
    uint32_t ir_buffer[SAMPLES_PER_WINDOW] = {0};

    float last_valid_bpm = 70.0f;
    float last_valid_spo2 = 98.0f;

    int buffer_index = 0;
    int samples_collected = 0;
    int fifo_sample_counter = 0;

    while (true)
    {
        time_t now;
        time(&now);
        if (now - task_start_time >= RUN_DURATION_SEC) {
            break;
        }

        uint32_t red = 0, ir = 0;

        if (max30102_read_fifo(&red, &ir))
        {
            if (ir > MAX_IR_LIMIT) ir = MAX_IR_LIMIT;

            red_buffer[buffer_index] = red;
            ir_buffer[buffer_index] = ir;
            buffer_index = (buffer_index + 1) % SAMPLES_PER_WINDOW;
            samples_collected++;
            fifo_sample_counter++;

            if (fifo_sample_counter >= 25)
            {
                max30102_reset_fifo();
                fifo_sample_counter = 0;
            }

            if (samples_collected >= SAMPLES_PER_WINDOW)
            {
                uint32_t ir_smoothed[SAMPLES_PER_WINDOW] = {0};
                for (int i = 0; i < SAMPLES_PER_WINDOW; i++) {
                    int start = (i - 4 >= 0) ? i - 4 : 0;
                    int end = (i + 4 < SAMPLES_PER_WINDOW) ? i + 4 : SAMPLES_PER_WINDOW - 1;
                    uint32_t sum = 0;
                    for (int j = start; j <= end; j++) sum += ir_buffer[j];
                    ir_smoothed[i] = sum / (end - start + 1);
                }

                int peaks[MAX_PEAKS_IN_WINDOW] = {0};
                int peak_count = 0;
                for (int i = 2; i < SAMPLES_PER_WINDOW - 2; i++) {
                    int prev_slope = ir_smoothed[i] - ir_smoothed[i - 1];
                    int next_slope = ir_smoothed[i + 1] - ir_smoothed[i];
                    if (prev_slope > 0 && next_slope < 0) {
                        if (peak_count == 0 || (i - peaks[peak_count - 1]) > MIN_PEAK_DISTANCE_SAMPLES) {
                            peaks[peak_count++] = i;
                            if (peak_count >= MAX_PEAKS_IN_WINDOW) break;
                        }
                    }
                }

                if (peak_count >= 2) {
                    float total_interval = 0;
                    for (int i = 1; i < peak_count; i++) {
                        total_interval += (peaks[i] - peaks[i - 1]);
                    }
                    float avg_interval = total_interval / (peak_count - 1);
                    float bpm = 60.0f * SAMPLE_RATE_HZ / avg_interval;

                    if (bpm >= MIN_BPM && bpm <= MAX_BPM) {
                        last_valid_bpm = BPM_SMOOTHING_FACTOR * bpm +
                                         (1.0f - BPM_SMOOTHING_FACTOR) * last_valid_bpm;
                    } else {
                        last_valid_bpm = 0.0f;
                    }
                } else {
                    last_valid_bpm = 0.0f;
                }

                float current_spo2 = last_valid_spo2;
                uint64_t red_dc = 0, ir_dc = 0;
                for (int i = 0; i < SAMPLES_PER_WINDOW; i++) {
                    red_dc += red_buffer[i];
                    ir_dc += ir_buffer[i];
                }
                red_dc /= SAMPLES_PER_WINDOW;
                ir_dc /= SAMPLES_PER_WINDOW;

                if (ir_dc > SIGNAL_QUALITY_THRESHOLD && red_dc > 0 && ir_dc > 0)
                {
                    float red_ac = 0, ir_ac = 0;
                    for (int i = 0; i < SAMPLES_PER_WINDOW; i++) {
                        red_ac += powf((float)red_buffer[i] - red_dc, 2);
                        ir_ac += powf((float)ir_buffer[i] - ir_dc, 2);
                    }
                    red_ac = sqrtf(red_ac / SAMPLES_PER_WINDOW);
                    ir_ac = sqrtf(ir_ac / SAMPLES_PER_WINDOW);

                    if (ir_ac > 1.0f) {
                        float r = (red_ac / red_dc) / (ir_ac / ir_dc);
                        current_spo2 = SPO2_CALIBRATION_A - (SPO2_CALIBRATION_B * r);  //This is the formula to calculate the spo2 value. this is a linear graph that can be changed with the macros in the header file.

                        if (current_spo2 >= MIN_SPO2 && current_spo2 <= MAX_SPO2) {
                            last_valid_spo2 = current_spo2;
                        }
                    }
                }

                global_sensor_data.bpm = last_valid_bpm;
                global_sensor_data.spo2 = last_valid_spo2;

                samples_collected = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(app_config.sampling_interval_ms));
    }

    xSemaphoreGive(sleep_semaphore);
    vTaskDelete(NULL);
}

//ADXL and DS3231 task with finite iterations 
void adxl_ds3231_task(void *arg) {
    while (true) {
        time_t now;
        time(&now);

        if (now - task_start_time >= RUN_DURATION_SEC) {
            break;
        }

        uint8_t hr, min, sec, day, mon, year;
        int16_t x, y, z;

        if (ds3231_read_time(&hr, &min, &sec, &day, &mon, &year) &&
            adxl345_read(&x, &y, &z)) {
            global_sensor_data.hour = hr;
            global_sensor_data.minute = min;
            global_sensor_data.second = sec;
            global_sensor_data.day = day;
            global_sensor_data.month = mon;
            global_sensor_data.year = year;
            global_sensor_data.accel_x = x;
            global_sensor_data.accel_y = y;
            global_sensor_data.accel_z = z;
        }

        vTaskDelay(pdMS_TO_TICKS(app_config.sampling_interval_ms));  // Sample every 2s
    }

    xSemaphoreGive(sleep_semaphore);  // Signal completion
    vTaskDelete(NULL);
}

void add_reading_to_buffer() {
    if (global_sensor_data.num_readings >= MAX_READINGS_PER_BOOT) return;

    int i = global_sensor_data.num_readings;
    global_sensor_data.readings[i].temperature = global_sensor_data.temperature;
    global_sensor_data.readings[i].bpm = global_sensor_data.bpm;
    global_sensor_data.readings[i].spo2 = global_sensor_data.spo2;
    global_sensor_data.readings[i].accel_x = global_sensor_data.accel_x;
    global_sensor_data.readings[i].accel_y = global_sensor_data.accel_y;
    global_sensor_data.readings[i].accel_z = global_sensor_data.accel_z;
    global_sensor_data.readings[i].hour = global_sensor_data.hour;
    global_sensor_data.readings[i].minute = global_sensor_data.minute;
    global_sensor_data.readings[i].second = global_sensor_data.second;
    global_sensor_data.readings[i].day = global_sensor_data.day;
    global_sensor_data.readings[i].month = global_sensor_data.month;
    global_sensor_data.readings[i].year = global_sensor_data.year;

    global_sensor_data.num_readings++;
}

//Print task with finite iterations
void print_task(void *arg) {
    while (global_sensor_data.num_readings < MAX_READINGS_PER_BOOT) {
        LOGI(TAG,
             "[%02d:%02d:%02d %02d-%02d-20%02d] Temp: %.2f°C BPM=%.1f SpO₂=%.1f%% Accel=X=%d Y=%d Z=%d",
             global_sensor_data.hour, global_sensor_data.minute, global_sensor_data.second,
             global_sensor_data.day,  global_sensor_data.month,  global_sensor_data.year,
             global_sensor_data.temperature, global_sensor_data.bpm, global_sensor_data.spo2,
             global_sensor_data.accel_x, global_sensor_data.accel_y, global_sensor_data.accel_z);

        add_reading_to_buffer();
        vTaskDelay(pdMS_TO_TICKS(app_config.sampling_interval_ms));
    }

    xSemaphoreGive(sleep_semaphore);
    vTaskDelete(NULL);
}

void load_app_config() {
    nvs_handle_t nvs;
    size_t size = sizeof(app_config);
    if (nvs_open("storage", NVS_READONLY, &nvs) == ESP_OK) {
        if (nvs_get_blob(nvs, "app_config", &app_config, &size) != ESP_OK) {
            // Set defaults if not found
            strcpy(app_config.ssid, CONFIG_WIFI_SSID);
            strcpy(app_config.password, CONFIG_WIFI_PASSWORD);
            app_config.sampling_interval_ms = 2000;
        }
        nvs_close(nvs);
    }
}

void save_app_config() {
    nvs_handle_t nvs;
    if (nvs_open("storage", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_blob(nvs, "app_config", &app_config, sizeof(app_config));
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}


//MQTT publish task with finite iterations
void mqtt_publish_task(void *arg) {
    // If we never connected, bail out immediately
    if (!client || !mqtt_connected) {
        xSemaphoreGive(sleep_semaphore);
        vTaskDelete(NULL);
    }

    for (int buf = 0; buf < MAX_SAVED_DATA; buf++) {
        // Skip empty buffers
        if (saved_data[buf].num_readings == 0) {
            LOGW(TAG, "No readings in saved_data[%d], skipping...", buf);
            continue;
        }

        // Convert this boot’s readings into a JSON string
        char *payload = create_readings_json(&saved_data[buf]);
        if (payload == NULL) {
            LOGW(TAG, "Failed to create JSON for saved_data[%d]", buf);
            continue;
        }

        // Publish it
        int msg_id = esp_mqtt_client_publish(
            client,
            "livestock/data1",
            payload,
            0,    // 0 == let the library strlen() it
            1,    // QoS 1
            0     // no retain
        );

        LOGI(TAG, "Published reading_%d with msg_id=%d", buf, msg_id);

        // Free the JSON payload
        free(payload);

        // Short pause so the TCP stack can flush
        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    // Clear all saved buffers for the next cycle
    memset(saved_data, 0, sizeof(saved_data));

    // Signal that we're done
    xSemaphoreGive(sleep_semaphore);
    vTaskDelete(NULL);
}

//WiFi initialization 
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, 
                               int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    static int mqtt_retry_count = 0;

    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            mqtt_connected = true;
            xEventGroupSetBits(wifi_event_group, MQTT_CONNECTED_BIT);
            mqtt_retry_count = 0;

            // Subscribe to config topic
            esp_mqtt_client_subscribe(client, "livestock/config1", 1);
            LOGI(TAG, "Subscribed to livestock/config1");
            break;

        case MQTT_EVENT_DISCONNECTED:
            mqtt_connected = false;
            xEventGroupClearBits(wifi_event_group, MQTT_CONNECTED_BIT);

            if (mqtt_retry_count < MAX_RETRY_ATTEMPTS) {
                vTaskDelay(pdMS_TO_TICKS(MQTT_RETRY_INTERVAL_MS));
                esp_mqtt_client_reconnect(client);
                mqtt_retry_count++;
            } else {
                esp_task_wdt_config_t twdt_config = {
                    .timeout_ms = 10000,
                    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
                    .trigger_panic = false
                };
                esp_task_wdt_init(&twdt_config);
                esp_task_wdt_add(NULL);
                vTaskDelay(pdMS_TO_TICKS(app_config.sampling_interval_ms));
                esp_restart();
            }
            break;

        case MQTT_EVENT_DATA: {
            // Log the received topic and data
            char topic[128] = {0};
            char data[256] = {0};

            memcpy(topic, event->topic, event->topic_len);
            memcpy(data, event->data, event->data_len);
            topic[event->topic_len] = '\0';
            data[event->data_len] = '\0';

            LOGI(TAG, "MQTT message received:");
            LOGI(TAG, "  Topic: %s", topic);
            LOGI(TAG, "  Data:  %s", data);

            if (strcmp(topic, "livestock/config1") == 0) {
                cJSON *json = cJSON_Parse(data);
                if (json) {
                    const cJSON *ssid = cJSON_GetObjectItem(json, "ssid");
                    const cJSON *pass = cJSON_GetObjectItem(json, "password");
                    const cJSON *interval = cJSON_GetObjectItem(json, "interval_ms");

                    bool changed = false;

                    if (ssid && cJSON_IsString(ssid)) {
                        strncpy(app_config.ssid, ssid->valuestring, sizeof(app_config.ssid));
                        changed = true;
                    }
                    if (pass && cJSON_IsString(pass)) {
                        strncpy(app_config.password, pass->valuestring, sizeof(app_config.password));
                        changed = true;
                    }
                    if (interval && cJSON_IsNumber(interval)) {
                        app_config.sampling_interval_ms = interval->valueint;
                        changed = true;
                        LOGI(TAG, "Sampling interval updated to %d ms", app_config.sampling_interval_ms);
                    }

                    if (changed) {
                        save_app_config();
                        LOGI(TAG, "Configuration updated via MQTT. Rebooting...");
                        esp_restart();
                    }

                    cJSON_Delete(json);
                } else {
                    LOGW(TAG, "Invalid JSON received on livestock/config1");
                }
            }
            break;
        }

        case MQTT_EVENT_ERROR:
            xEventGroupClearBits(wifi_event_group, MQTT_CONNECTED_BIT);
            LOGE(TAG, "MQTT_EVENT_ERROR occurred");
            break;

        default:
            break;
    }
}



// Modified wifi_event_handler
static void wifi_event_handler(void *arg, esp_event_base_t event_base, 
                             int32_t event_id, void *event_data) {
    static int retry_count = 0;
    
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
        
        if (retry_count < MAX_RETRY_ATTEMPTS) {
            vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_INTERVAL_MS));
            esp_wifi_connect();
            retry_count++;
        } else {
            esp_task_wdt_config_t twdt_config = {
			    .timeout_ms = 10000,           // 10 seconds
			    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  // All cores
			    .trigger_panic = false         // No panic on timeout
			};
			
			esp_task_wdt_init(&twdt_config);
			esp_task_wdt_add(NULL);
			vTaskDelay(pdMS_TO_TICKS(app_config.sampling_interval_ms));  // Let logs flush
			esp_restart();
        }
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        retry_count = 0; // Reset retry counter on successful connection
    }
}

void wifi_init_sta(void) {
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_wifi_set_ps(WIFI_PS_NONE);  // Disable power save

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {0};
    strcpy((char*)wifi_config.sta.ssid, app_config.ssid);
    strcpy((char*)wifi_config.sta.password, app_config.password);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    LOGI(TAG, "Connecting to Wi‑Fi...");
    if (xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, pdMS_TO_TICKS(WIFI_RETRY_INTERVAL_MS)) & WIFI_CONNECTED_BIT) {
        LOGI(TAG, "Wi‑Fi connected");
    } else {
        LOGW(TAG, "Wi‑Fi connect timeout");
    }
}



bool is_wifi_connected() {
    // Wait indefinitely for the connection bit to be set
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                           WIFI_CONNECTED_BIT,
                                           pdFALSE,
                                           pdTRUE,
                                           portMAX_DELAY); // block forever

    return (bits & WIFI_CONNECTED_BIT) != 0;
}

//MQTT initialization 
void mqtt_app_start(void) {
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = "mqtt://broker.emqx.io",
        },
        .network = {
            .disable_auto_reconnect = false, // Enable auto-reconnect
            .reconnect_timeout_ms = 10000,   // 10 second reconnect timeout
        },
        .session = {
            .keepalive = 60,                 // 60 second keepalive
        }
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    esp_mqtt_client_subscribe(client, "livestock/config1", 1);
    
    // Wait for MQTT connection with timeout
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, 
                                         MQTT_CONNECTED_BIT, 
                                         pdFALSE, 
                                         pdTRUE, 
                                         pdMS_TO_TICKS(WIFI_RETRY_INTERVAL_MS));
}

int load_boot_count() {
    nvs_handle_t nvs;
    int count = 0;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs);
    if (err == ESP_OK) {
        err = nvs_get_i32(nvs, "boot_count", &count);
        if (err != ESP_OK) {
            count = 0;  // fallback if key doesn't exist
        }
        nvs_close(nvs);
    }
    return count;
}

void save_boot_count(int count) {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        nvs_set_i32(nvs, "boot_count", count);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

int load_mqtt_publish_count() {
    nvs_handle_t nvs;
    int count = 0;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs);
    if (err == ESP_OK) {
        err = nvs_get_i32(nvs, "mqtt_count", &count);
        if (err != ESP_OK) {
            count = 0;  // fallback if key doesn't exist
        }
        nvs_close(nvs);
    }
    return count;
}

void save_mqtt_publish_count(int count) {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        nvs_set_i32(nvs, "mqtt_count", count);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

void app_main(void) {
    mqtt_connected = false;

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Load and increment boot count
    boot_count = load_boot_count() + 1;
    save_boot_count(boot_count);

    // Load MQTT publish count
    mqtt_publish_count = load_mqtt_publish_count();

    // Load Wi-Fi & interval settings
    load_app_config();

    // Sync DS3231 with NTP only on first boot
    if (boot_count == 1) {
        ds3231_sync_with_ntp();
        esp_netif_deinit();
        esp_event_loop_delete_default();
    }

    memset(&global_sensor_data, 0, sizeof(global_sensor_data));
    check_wakeup_reason();
    vTaskDelay(pdMS_TO_TICKS(1000));  // Stabilize after wakeup

    // Init sensors
    tmp117_i2c_init();
    adxl345_init();
    ds3231_init();
    max30102_init();

    // Create a semaphore for sleep logic
    sleep_semaphore = xSemaphoreCreateCounting(5, 0);
    time(&task_start_time);

    // Start sensor collection tasks
    xTaskCreate(tmp117_task,                 "tmp117",  4096,   NULL, 5, NULL);
    xTaskCreate(data_conversion_to_readable, "ppg",     12288,  NULL, 5, NULL);
    xTaskCreate(adxl_ds3231_task,           "accel",   4096,   NULL, 5, NULL);
    xTaskCreate(print_task,                 "print",   4096,   NULL, 4, NULL);

    // Wait for all tasks to complete
    for (int i = 0; i < 4; i++) {
        xSemaphoreTake(sleep_semaphore, portMAX_DELAY);
    }

    // Save this boot's sensor data
    int idx = (boot_count - 1) % MAX_SAVED_DATA;
    if (idx < 0 || idx >= MAX_SAVED_DATA) idx = 0;
    saved_data[idx] = global_sensor_data;

    char key[16];
    snprintf(key, sizeof(key), "reading_%d", idx);
    nvs_handle_t nvs;
    if (nvs_open("storage", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_blob(nvs, key, &saved_data[idx], sizeof(sensor_data_t));
        nvs_commit(nvs);
        nvs_close(nvs);
        LOGI(TAG, "Saved reading_%d to NVS (boot %d)", idx, boot_count);
    }

    // Publish every 5 boots
    if (boot_count % MAX_SAVED_DATA == 0) {
        // Load saved readings
        if (nvs_open("storage", NVS_READONLY, &nvs) == ESP_OK) {
            for (int i = 0; i < MAX_SAVED_DATA; i++) {
                snprintf(key, sizeof(key), "reading_%d", i);
                size_t size = sizeof(sensor_data_t);
                if (nvs_get_blob(nvs, key, &saved_data[i], &size) != ESP_OK) {
                    LOGW(TAG, "Missing reading_%d in NVS", i);
                    memset(&saved_data[i], 0, sizeof(sensor_data_t));
                }
            }
            nvs_close(nvs);
        }

        // Connect Wi-Fi + re-sync time + MQTT
        wifi_init_sta();
        mqtt_app_start();

        EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group,
        MQTT_CONNECTED_BIT,
        pdFALSE,  // don't clear
        pdTRUE,   // wait for all bits
        pdMS_TO_TICKS(3000) // wait max 30s
        );

        if (bits & MQTT_CONNECTED_BIT) {
        LOGI(TAG, "MQTT connected, starting publish task");
        xTaskCreate(mqtt_publish_task, "mqtt_pub", 10240, NULL, 5, NULL);
        xSemaphoreTake(sleep_semaphore, portMAX_DELAY);
        mqtt_publish_count++;
        save_mqtt_publish_count(mqtt_publish_count);
        } else {
        LOGE(TAG, "MQTT connection timeout. Skipping publish.");
        }

        esp_mqtt_client_stop(client);
        esp_wifi_stop();

        // Full cleanup after 25 boots
        if (mqtt_publish_count % 3 == 0) {
            save_boot_count(0);
            save_mqtt_publish_count(0);
            if (nvs_open("storage", NVS_READWRITE, &nvs) == ESP_OK) {
                for (int i = 0; i < MAX_SAVED_DATA; i++) {
                    snprintf(key, sizeof(key), "reading_%d", i);
                    nvs_erase_key(nvs, key);
                }
                nvs_commit(nvs);
                nvs_close(nvs);
            }
            esp_restart();
        }
    }

    // Sleep until next sample cycle
    enter_deep_sleep(SLEEP_DURATION_SEC * 1000000ULL);
}

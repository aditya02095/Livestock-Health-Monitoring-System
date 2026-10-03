# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.


idf_component_register(
    SRCS 
        main.c 
        adxl345.c 
        ds3231.c 
        max30102.c 
        sensor_data.c 
        tmp117.c

    INCLUDE_DIRS 
        .

    REQUIRES 
        cjson
        driver         # for i2c, gpio, rtc_io
        freertos       # for task, semphr, event_groups
        esp_timer      # used by time functions
        nvs_flash      # for non-volatile storage init
        esp_event      # for Wi-Fi and MQTT events
        esp_netif      # required for Wi-Fi stack
        esp_wifi       # for Wi-Fi driver
        esp_system     # for deep sleep
        lwip           # for SNTP (RTC sync)
        esp-mqtt       # for mqtt_client.h and mqtt handling
)

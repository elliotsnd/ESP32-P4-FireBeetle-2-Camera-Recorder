# ESP32-P4 Camera Recorder Application (V4L2-style esp_video API)

idf_component_register(
    SRCS "camera_recorder.c"
    INCLUDE_DIRS "."
    REQUIRES 
        driver
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_i2s
        esp_driver_ppa
        esp_driver_sdmmc
        nvs_flash
        esp_system
        freertos
        fatfs
        sdmmc
        esp_psram
        esp_mm
        esp_timer
        esp_video
        esp_cam_sensor
        esp_sccb_intf
        vfs
)

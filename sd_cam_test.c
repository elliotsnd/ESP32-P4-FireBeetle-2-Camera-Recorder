/*
 * ESP32-P4 v1.0 Camera Test with SD Card
 * Camera: Raspberry Pi Camera 3 (Sony IMX708)
 * Based on DFRobot CSI camera example
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

static const char *TAG = "ESP32P4_CAM_SD";

// SD Card pins (FireBeetle 2 ESP32-P4)
#define PIN_NUM_CMD  33
#define PIN_NUM_CLK  34
#define PIN_NUM_D0   37
#define PIN_NUM_D1   38
#define PIN_NUM_D2   39
#define PIN_NUM_D3   40

// LED pin
#define LED_GPIO     GPIO_NUM_15

// Camera I2C pins (from DFRobot example - works for Pi Cam 3)
#define CAM_I2C_SCL  8
#define CAM_I2C_SDA  7

// Pi Camera 3 (Sony IMX708) I2C address
#define IMX708_I2C_ADDR  0x1A

static sdmmc_card_t *sd_card = NULL;

static void test_system_info(void)
{
    ESP_LOGI(TAG, "=== ESP32-P4 v1.0 + Pi Cam 3 + SD Test ===");
    
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    ESP_LOGI(TAG, "Chip: ESP32-P4");
    ESP_LOGI(TAG, "Cores: %d", chip_info.cores);
    ESP_LOGI(TAG, "Silicon revision: %d", chip_info.revision);
    ESP_LOGI(TAG, "IDF Version: %s", esp_get_idf_version());
    
    uint32_t flash_size;
    esp_flash_get_size(NULL, &flash_size);
    ESP_LOGI(TAG, "Flash size: %lu MB", flash_size / (1024 * 1024));
    
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
}

static void test_led(void)
{
    ESP_LOGI(TAG, "=== LED Test ===");
    
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    for (int i = 0; i < 3; i++) {
        ESP_LOGI(TAG, "LED ON");
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(300));
        
        ESP_LOGI(TAG, "LED OFF");
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

static void test_i2c_scan(void)
{
    ESP_LOGI(TAG, "=== I2C Bus Scan (Camera I2C) ===");
    ESP_LOGI(TAG, "Looking for Pi Camera 3 (IMX708) at 0x%02X", IMX708_I2C_ADDR);
    
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CAM_I2C_SDA,
        .scl_io_num = CAM_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    i2c_master_bus_handle_t bus_handle;
    esp_err_t ret = i2c_new_master_bus(&bus_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C master bus init failed: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "Scanning I2C bus (addresses 0x08-0x77)...");
    int devices_found = 0;
    bool imx708_found = false;
    
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        ret = i2c_master_probe(bus_handle, addr, 100);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  Device found at 0x%02X", addr);
            devices_found++;
            if (addr == IMX708_I2C_ADDR) {
                imx708_found = true;
                ESP_LOGI(TAG, "  ^^ This is the IMX708 camera sensor!");
            }
        }
    }
    
    if (devices_found == 0) {
        ESP_LOGW(TAG, "No I2C devices found");
        ESP_LOGW(TAG, "Camera may need CSI power rail enabled first");
    } else {
        ESP_LOGI(TAG, "Total devices found: %d", devices_found);
        if (!imx708_found) {
            ESP_LOGW(TAG, "IMX708 not detected - check camera connection");
        }
    }
    
    i2c_del_master_bus(bus_handle);
}

static void test_sd_card(void)
{
    ESP_LOGI(TAG, "=== SD Card Test (SDMMC 4-bit) ===");
    
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
    
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;
    slot_config.clk = PIN_NUM_CLK;
    slot_config.cmd = PIN_NUM_CMD;
    slot_config.d0 = PIN_NUM_D0;
    slot_config.d1 = PIN_NUM_D1;
    slot_config.d2 = PIN_NUM_D2;
    slot_config.d3 = PIN_NUM_D3;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &sd_card);
    
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SD card: %s", esp_err_to_name(ret));
        }
        return;
    }
    
    ESP_LOGI(TAG, "SD Card mounted successfully");
    sdmmc_card_print_info(stdout, sd_card);
    
    // Write test
    ESP_LOGI(TAG, "Writing test file...");
    FILE *f = fopen("/sdcard/test.txt", "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        esp_vfs_fat_sdcard_unmount("/sdcard", sd_card);
        return;
    }
    
    fprintf(f, "ESP32-P4 v1.0 + Pi Cam 3 (IMX708) Test\n");
    fprintf(f, "IDF Version: %s\n", esp_get_idf_version());
    fprintf(f, "Ready for 1080p video recording!\n");
    fclose(f);
    ESP_LOGI(TAG, "File written successfully");
    
    // Read test
    ESP_LOGI(TAG, "Reading test file...");
    f = fopen("/sdcard/test.txt", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        esp_vfs_fat_sdcard_unmount("/sdcard", sd_card);
        return;
    }
    
    char line[128];
    while (fgets(line, sizeof(line), f) != NULL) {
        printf("%s", line);
    }
    fclose(f);
    
    // Speed test for video recording
    ESP_LOGI(TAG, "SD Card write speed test (4KB blocks)...");
    uint8_t *buffer = malloc(4096);
    if (buffer) {
        memset(buffer, 0xAA, 4096);
        
        f = fopen("/sdcard/speed.bin", "wb");
        if (f) {
            int64_t start = esp_timer_get_time();
            for (int i = 0; i < 1000; i++) {  // 4MB total
                fwrite(buffer, 1, 4096, f);
            }
            int64_t duration = esp_timer_get_time() - start;
            fclose(f);
            
            float speed_mbps = (4.0 * 1000 * 1000) / duration;  // MB/s
            ESP_LOGI(TAG, "Write speed: %.2f MB/s (%.2f Mbps)", 
                     speed_mbps, speed_mbps * 8);
            
            if (speed_mbps >= 5.0) {
                ESP_LOGI(TAG, "Speed OK for 1080p30 (need 5-20 MB/s)");
            } else {
                ESP_LOGW(TAG, "Speed may be insufficient for video recording");
            }
        }
        free(buffer);
    }
    
    esp_vfs_fat_sdcard_unmount("/sdcard", sd_card);
    ESP_LOGI(TAG, "SD Card test completed");
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    test_system_info();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    test_led();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    test_i2c_scan();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    test_sd_card();
    
    ESP_LOGI(TAG, "=== All tests complete ===");
    ESP_LOGI(TAG, "Next: Camera initialization with esp_cam CSI APIs");
    ESP_LOGI(TAG, "Camera: Pi Cam 3 (IMX708), I2C SCL=8 SDA=7, CSI 2-lane");
    ESP_LOGI(TAG, "Target: 1920x1080 @ 30fps (RAW10) for video recording");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

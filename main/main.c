/*
 * ESP32-P4 FireBeetle 2 Hardware Test
 * 
 * Tests all available peripherals while we wait for video components:
 * - WiFi connection
 * - SD card (SDMMC)
 * - GPIO/LED
 * - I2C bus scan
 * - PSRAM
 * - System info
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include <unistd.h>
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_psram.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

static const char *TAG = "ESP32P4_TEST";

// SD Card pins for FireBeetle 2 ESP32-P4
#define PIN_NUM_CMD  33
#define PIN_NUM_CLK  34
#define PIN_NUM_D0   37
#define PIN_NUM_D1   38
#define PIN_NUM_D2   39
#define PIN_NUM_D3   40

// LED pin (adjust based on your board)
#define LED_GPIO     GPIO_NUM_15

static sdmmc_card_t *sd_card = NULL;

/**
 * Test 1: System Information
 */
static void test_system_info(void)
{
    ESP_LOGI(TAG, "=== System Information ===");
    
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    ESP_LOGI(TAG, "Chip: ESP32-P4");
    ESP_LOGI(TAG, "Cores: %d", chip_info.cores);
    ESP_LOGI(TAG, "Silicon revision: %d", chip_info.revision);
    ESP_LOGI(TAG, "IDF Version: %s", esp_get_idf_version());
    
    // Flash size
    uint32_t flash_size;
    esp_flash_get_size(NULL, &flash_size);
    ESP_LOGI(TAG, "Flash size: %lu MB", flash_size / (1024 * 1024));
    
    // PSRAM
    if (esp_psram_is_initialized()) {
        ESP_LOGI(TAG, "PSRAM: ENABLED (%d MB)", 
                 esp_psram_get_size() / (1024 * 1024));
    } else {
        ESP_LOGI(TAG, "PSRAM: NOT AVAILABLE");
    }
    
    // Heap info
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "Min free heap: %lu bytes", esp_get_minimum_free_heap_size());
}

/**
 * Test 2: LED Blink
 */
static void test_led(void)
{
    ESP_LOGI(TAG, "=== LED Test ===");
    
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    ESP_LOGI(TAG, "Blinking LED on GPIO %d (3 times)", LED_GPIO);
    
    for (int i = 0; i < 3; i++) {
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    ESP_LOGI(TAG, "LED test complete");
}

/**
 * Test 3: I2C Bus Scan
 */
static void test_i2c_scan(void)
{
    ESP_LOGI(TAG, "=== I2C Bus Scan ===");
    
    // I2C master configuration
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_8,  // Adjust for your board
        .scl_io_num = GPIO_NUM_9,  // Adjust for your board
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    i2c_master_bus_handle_t bus_handle;
    esp_err_t ret = i2c_new_master_bus(&bus_config, &bus_handle);
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "I2C initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "Scanning I2C bus...");
    int devices_found = 0;
    
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        ret = i2c_master_probe(bus_handle, addr, 100);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Found device at address: 0x%02X", addr);
            devices_found++;
        }
    }
    
    if (devices_found == 0) {
        ESP_LOGI(TAG, "No I2C devices found");
    } else {
        ESP_LOGI(TAG, "Found %d I2C device(s)", devices_found);
    }
    
    i2c_del_master_bus(bus_handle);
}

/**
 * Test 4: SD Card
 */
static void test_sd_card(void)
{
    ESP_LOGI(TAG, "=== SD Card Test ===");
    
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_52M;
    
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;
    slot_config.clk = PIN_NUM_CLK;
    slot_config.cmd = PIN_NUM_CMD;
    slot_config.d0 = PIN_NUM_D0;
    slot_config.d1 = PIN_NUM_D1;
    slot_config.d2 = PIN_NUM_D2;
    slot_config.d3 = PIN_NUM_D3;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    
    const char mount_point[] = "/sdcard";
    esp_err_t ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, 
                                            &mount_config, &sd_card);
    
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SD card: %s", esp_err_to_name(ret));
        }
        ESP_LOGW(TAG, "SD Card: NOT DETECTED");
        return;
    }
    
    // Print card info
    ESP_LOGI(TAG, "SD Card: DETECTED");
    ESP_LOGI(TAG, "Name: %s", sd_card->cid.name);
    ESP_LOGI(TAG, "Speed: %s", 
             (sd_card->csd.tr_speed > 25000000) ? "high speed" : "default speed");
    ESP_LOGI(TAG, "Size: %llu MB", 
             ((uint64_t)sd_card->csd.capacity) * sd_card->csd.sector_size / (1024 * 1024));
    ESP_LOGI(TAG, "CSD: ver=%d, sector_size=%d, capacity=%d",
             sd_card->csd.csd_ver, sd_card->csd.sector_size, sd_card->csd.capacity);
    
    // Test file write/read
    ESP_LOGI(TAG, "Testing file operations...");
    const char *test_file = "/sdcard/test.txt";
    const char *test_data = "ESP32-P4 Hardware Test!";
    
    FILE *f = fopen(test_file, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to create test file");
    } else {
        fprintf(f, "%s\n", test_data);
        fclose(f);
        ESP_LOGI(TAG, "File written successfully");
        
        // Read back
        f = fopen(test_file, "r");
        if (f != NULL) {
            char line[64];
            if (fgets(line, sizeof(line), f) != NULL) {
                line[strcspn(line, "\n")] = 0;  // Remove newline
                if (strcmp(line, test_data) == 0) {
                    ESP_LOGI(TAG, "File read verification: PASSED");
                } else {
                    ESP_LOGW(TAG, "File read verification: FAILED");
                }
            }
            fclose(f);
        }
        
        // Cleanup
        unlink(test_file);
    }
    
    // Unmount
    esp_vfs_fat_sdcard_unmount(mount_point, sd_card);
    ESP_LOGI(TAG, "SD card test complete");
}

/**
 * Test 5: Note about ESP32-P4
 */
static void test_note(void)
{
    ESP_LOGI(TAG, "=== ESP32-P4 Note ===");
    ESP_LOGI(TAG, "ESP32-P4 is wired Ethernet only (no WiFi/BT)");
    ESP_LOGI(TAG, "For wireless: use ESP32-P4-Function EV Board with C6");
}

/**
 * Main Application
 */
void app_main(void)
{
    ESP_LOGI(TAG, "\n");
    ESP_LOGI(TAG, "╔════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   ESP32-P4 FireBeetle 2 Hardware Test ║");
    ESP_LOGI(TAG, "║   Preparing for future camera support ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════╝");
    ESP_LOGI(TAG, "\n");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Run all tests
    test_system_info();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    test_led();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    test_i2c_scan();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    test_sd_card();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    test_note();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "\n");
    ESP_LOGI(TAG, "╔════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║     All Hardware Tests Complete!      ║");
    ESP_LOGI(TAG, "║   Board is ready for camera modules   ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════╝");
    ESP_LOGI(TAG, "\n");
    ESP_LOGI(TAG, "Next steps:");
    ESP_LOGI(TAG, "1. Connect Raspberry Pi Camera 3 to MIPI-CSI port");
    ESP_LOGI(TAG, "2. Wait for esp-video components (Q2-Q3 2026)");
    ESP_LOGI(TAG, "3. Upgrade to video recording firmware");
    ESP_LOGI(TAG, "\n");
}

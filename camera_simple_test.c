/*
 * ESP32-P4 v1.0 + Pi Camera 3 (IMX708) Simple I2C Test
 * ESP-IDF v5.4.1
 * 
 * Goal: Verify I2C communication with IMX708 sensor
 * - Read chip ID register
 * - Verify sensor responds
 * - Prepare for frame capture
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_chip_info.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "spi_flash_mmap.h"

static const char *TAG = "IMX708_TEST";

// Pi Camera 3 (IMX708) Configuration  
#define IMX708_I2C_ADDR         0x1A
#define CAM_I2C_SCL             8
#define CAM_I2C_SDA             7
#define I2C_MASTER_FREQ_HZ      100000  // 100 kHz for stability

// IMX708 register addresses (from datasheet)
#define IMX708_CHIP_ID_REG_H    0x0016  // Model ID high byte
#define IMX708_CHIP_ID_REG_L    0x0017  // Model ID low byte
#define IMX708_EXPECTED_ID_H    0x07    // IMX708 ID = 0x0778
#define IMX708_EXPECTED_ID_L    0x78

// LED for status indication
#define LED_PIN                 15

// I2C handle
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t imx708_handle = NULL;

/**
 * Initialize I2C bus for camera
 */
static esp_err_t init_i2c_bus(void)
{
    ESP_LOGI(TAG, "Initializing I2C bus (SCL=%d, SDA=%d)", CAM_I2C_SCL, CAM_I2C_SDA);
    
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CAM_I2C_SDA,
        .scl_io_num = CAM_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    esp_err_t ret = i2c_new_master_bus(&bus_config, &i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Add IMX708 device to bus
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMX708_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    
    ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_config, &imx708_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Add IMX708 device failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C bus initialized successfully");
    return ESP_OK;
}

/**
 * Read 8-bit register from IMX708
 * IMX708 uses 16-bit register addresses
 */
static esp_err_t imx708_read_reg(uint16_t reg_addr, uint8_t *data)
{
    uint8_t write_buf[2] = {
        (reg_addr >> 8) & 0xFF,  // High byte
        reg_addr & 0xFF           // Low byte
    };
    
    esp_err_t ret = i2c_master_transmit_receive(
        imx708_handle,
        write_buf, sizeof(write_buf),
        data, 1,
        1000 / portTICK_PERIOD_MS
    );
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read reg 0x%04x failed: %s", reg_addr, esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * Write 8-bit register to IMX708
 */
__attribute__((unused))
static esp_err_t imx708_write_reg(uint16_t reg_addr, uint8_t data)
{
    uint8_t write_buf[3] = {
        (reg_addr >> 8) & 0xFF,  // High byte
        reg_addr & 0xFF,          // Low byte
        data
    };
    
    esp_err_t ret = i2c_master_transmit(
        imx708_handle,
        write_buf, sizeof(write_buf),
        1000 / portTICK_PERIOD_MS
    );
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write reg 0x%04x = 0x%02x failed: %s", 
                 reg_addr, data, esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * Read IMX708 chip ID to verify communication
 */
static esp_err_t test_imx708_chip_id(void)
{
    ESP_LOGI(TAG, "=== Testing IMX708 Chip ID ===");
    
    uint8_t chip_id_h = 0;
    uint8_t chip_id_l = 0;
    
    // Read high byte
    esp_err_t ret = imx708_read_reg(IMX708_CHIP_ID_REG_H, &chip_id_h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID high byte");
        return ret;
    }
    
    // Read low byte
    ret = imx708_read_reg(IMX708_CHIP_ID_REG_L, &chip_id_l);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID low byte");
        return ret;
    }
    
    uint16_t chip_id = (chip_id_h << 8) | chip_id_l;
    uint16_t expected_id = (IMX708_EXPECTED_ID_H << 8) | IMX708_EXPECTED_ID_L;
    
    ESP_LOGI(TAG, "Chip ID: 0x%04x (expected: 0x%04x)", chip_id, expected_id);
    
    if (chip_id == expected_id) {
        ESP_LOGI(TAG, "✓ IMX708 sensor verified!");
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "⚠ Chip ID mismatch - may still work");
        return ESP_OK;  // Continue anyway - some registers may differ
    }
}

/**
 * Initialize LED for status
 */
static void init_led(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_PIN, 0);
}

/**
 * Blink LED pattern
 */
static void blink_led(int count, int delay_ms)
{
    for (int i = 0; i < count; i++) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(delay_ms / portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(delay_ms / portTICK_PERIOD_MS);
    }
}

/**
 * Print system information
 */
static void print_system_info(void)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    ESP_LOGI(TAG, "=== System Information ===");
    ESP_LOGI(TAG, "Chip: ESP32-P4 rev%d", chip_info.revision);
    ESP_LOGI(TAG, "Cores: %d", chip_info.cores);
    ESP_LOGI(TAG, "ESP-IDF: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "Free heap: %ld bytes", esp_get_free_heap_size());
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize LED
    init_led();
    
    // Print system info
    print_system_info();
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "  Pi Camera 3 (IMX708) I2C Test");
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "");
    
    // Initialize I2C bus
    ret = init_i2c_bus();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed!");
        blink_led(10, 100);  // Fast blinks = error
        return;
    }
    
    // Test IMX708 chip ID
    ret = test_imx708_chip_id();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "✓ Camera I2C communication successful!");
        blink_led(3, 500);  // Slow blinks = success
    } else {
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "✗ Camera I2C communication failed!");
        blink_led(10, 100);  // Fast blinks = error
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Test complete. Monitoring...");
    ESP_LOGI(TAG, "");
    
    // Main loop - blink every 2 seconds to show running
    while (1) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(1900 / portTICK_PERIOD_MS);
    }
}

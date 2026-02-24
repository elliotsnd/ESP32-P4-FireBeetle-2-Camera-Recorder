/*
 * Minimal I2C recovery test for IMX708
 * Only I2C - no LDO, no CSI, no ISP
 * Tests if camera needs power cycle
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "nvs_flash.h"

static const char *TAG = "I2C_TEST";
#define LED_PIN 15

void app_main(void)
{
    nvs_flash_init();
    
    gpio_config_t io = { .pin_bit_mask = (1ULL << LED_PIN), .mode = GPIO_MODE_OUTPUT };
    gpio_config(&io);
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    gpio_set_level(LED_PIN, 0);

    ESP_LOGI(TAG, "=== I2C Recovery Test ===");
    
    // Wait for serial
    for (int i = 5; i > 0; i--) {
        ESP_LOGI(TAG, "Starting in %d...", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
    // Test 1: I2C without MCLK
    ESP_LOGI(TAG, "--- Test 1: I2C without MCLK ---");
    i2c_master_bus_handle_t bus = NULL;
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = 7, .scl_io_num = 8,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_new_master_bus(&bus_cfg, &bus);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Scan
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(bus, addr, 100) == ESP_OK)
            ESP_LOGI(TAG, "  Found 0x%02X", addr);
    }
    
    // Try reading Chip ID
    i2c_master_dev_handle_t cam = NULL;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x1A,
        .scl_speed_hz = 100000,
    };
    i2c_master_bus_add_device(bus, &dev_cfg, &cam);
    
    uint8_t addr_buf[2], val;
    
    // Read 0x0016 (Model ID High)
    addr_buf[0] = 0x00; addr_buf[1] = 0x16;
    val = 0xFF;
    i2c_master_transmit_receive(cam, addr_buf, 2, &val, 1, 1000);
    ESP_LOGI(TAG, "  No MCLK: Reg 0x0016 = 0x%02X", val);
    
    addr_buf[0] = 0x00; addr_buf[1] = 0x17;
    val = 0xFF;
    i2c_master_transmit_receive(cam, addr_buf, 2, &val, 1, 1000);
    ESP_LOGI(TAG, "  No MCLK: Reg 0x0017 = 0x%02X", val);
    
    // Test 2: Start MCLK on GPIO47 then read
    ESP_LOGI(TAG, "--- Test 2: Start MCLK on GPIO47, then read ---");
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_1_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 24000000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_cfg);
    ledc_channel_config_t chan_cfg = {
        .gpio_num = 47,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 1,
        .hpoint = 0,
    };
    ledc_channel_config(&chan_cfg);
    ESP_LOGI(TAG, "  MCLK started on GPIO47");
    vTaskDelay(500 / portTICK_PERIOD_MS);  // Wait 500ms for clock
    
    addr_buf[0] = 0x00; addr_buf[1] = 0x16;
    val = 0xFF;
    i2c_master_transmit_receive(cam, addr_buf, 2, &val, 1, 1000);
    ESP_LOGI(TAG, "  With MCLK: Reg 0x0016 = 0x%02X", val);
    
    addr_buf[0] = 0x00; addr_buf[1] = 0x17;
    val = 0xFF;
    i2c_master_transmit_receive(cam, addr_buf, 2, &val, 1, 1000);
    ESP_LOGI(TAG, "  With MCLK: Reg 0x0017 = 0x%02X", val);
    
    // Test 3: Try SW reset then read
    ESP_LOGI(TAG, "--- Test 3: SW Reset then read ---");
    uint8_t reset_buf[3] = {0x01, 0x03, 0x01};  // reg 0x0103 = 0x01
    i2c_master_transmit(cam, reset_buf, 3, 1000);
    ESP_LOGI(TAG, "  SW reset sent, waiting 1s...");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    addr_buf[0] = 0x00; addr_buf[1] = 0x16;
    val = 0xFF;
    i2c_master_transmit_receive(cam, addr_buf, 2, &val, 1, 1000);
    ESP_LOGI(TAG, "  After reset: Reg 0x0016 = 0x%02X", val);
    
    addr_buf[0] = 0x00; addr_buf[1] = 0x17;
    val = 0xFF;
    i2c_master_transmit_receive(cam, addr_buf, 2, &val, 1, 1000);
    ESP_LOGI(TAG, "  After reset: Reg 0x0017 = 0x%02X", val);

    // Test 4: Try different I2C speed (400kHz)
    ESP_LOGI(TAG, "--- Test 4: Try 400kHz I2C ---");
    i2c_master_bus_rm_device(cam);
    dev_cfg.scl_speed_hz = 400000;
    i2c_master_bus_add_device(bus, &dev_cfg, &cam);
    
    addr_buf[0] = 0x00; addr_buf[1] = 0x16;
    val = 0xFF;
    i2c_master_transmit_receive(cam, addr_buf, 2, &val, 1, 1000);
    ESP_LOGI(TAG, "  400kHz: Reg 0x0016 = 0x%02X", val);
    
    addr_buf[0] = 0x00; addr_buf[1] = 0x17;
    val = 0xFF;
    i2c_master_transmit_receive(cam, addr_buf, 2, &val, 1, 1000);
    ESP_LOGI(TAG, "  400kHz: Reg 0x0017 = 0x%02X", val);

    // Test 5: Read EEPROM at 0x50 to verify I2C bus works
    ESP_LOGI(TAG, "--- Test 5: Read EEPROM at 0x50 ---");
    i2c_master_dev_handle_t eeprom = NULL;
    i2c_device_config_t eep_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x50,
        .scl_speed_hz = 100000,
    };
    i2c_master_bus_add_device(bus, &eep_cfg, &eeprom);
    
    // EEPROM usually uses 8-bit or 16-bit address
    // Try 8-bit address: read first 4 bytes
    uint8_t eep_addr = 0x00;
    uint8_t eep_data[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    esp_err_t eep_ret = i2c_master_transmit_receive(eeprom, &eep_addr, 1, eep_data, 4, 1000);
    ESP_LOGI(TAG, "  EEPROM 0x50: %s → %02X %02X %02X %02X",
             esp_err_to_name(eep_ret), eep_data[0], eep_data[1], eep_data[2], eep_data[3]);

    // Test 6: Try reading device at 0x0C
    ESP_LOGI(TAG, "--- Test 6: Read device at 0x0C ---");
    i2c_master_dev_handle_t dev0c = NULL;
    i2c_device_config_t dev0c_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x0C,
        .scl_speed_hz = 100000,
    };
    i2c_master_bus_add_device(bus, &dev0c_cfg, &dev0c);
    uint8_t d0c_data[2] = {0xFF, 0xFF};
    uint8_t d0c_addr = 0x00;
    esp_err_t d0c_ret = i2c_master_transmit_receive(dev0c, &d0c_addr, 1, d0c_data, 2, 1000);
    ESP_LOGI(TAG, "  Dev 0x0C: %s → %02X %02X",
             esp_err_to_name(d0c_ret), d0c_data[0], d0c_data[1]);

    // Summary
    ESP_LOGI(TAG, "=== TEST COMPLETE ===");
    ESP_LOGI(TAG, "If all IMX708 reads return 0x02:");
    ESP_LOGI(TAG, "  → Camera needs USB power cycle (unplug & replug)");
    ESP_LOGI(TAG, "  → Or GPIO47 is not connected to camera MCLK");
    
    // Blink to show done
    while (1) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(800 / portTICK_PERIOD_MS);
    }
}

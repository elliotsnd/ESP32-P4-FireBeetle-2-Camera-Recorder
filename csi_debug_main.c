/*
 * ESP32-P4 v1.0 CSI Debug - Step by step with delays
 * Find exactly which step crashes ECO1
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_heap_caps.h"

// CSI + ISP headers
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "driver/isp.h"
#include "esp_ldo_regulator.h"

static const char *TAG = "DBG";

#define LED_PIN     15
#define IMX708_ADDR 0x1A

static void blink(int n, int ms) {
    for (int i = 0; i < n; i++) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(ms / portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(ms / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    esp_err_t ret;

    // NVS
    nvs_flash_init();

    // LED
    gpio_config_t io = { .pin_bit_mask = (1ULL << LED_PIN), .mode = GPIO_MODE_OUTPUT };
    gpio_config(&io);

    // Wait 3 seconds so serial can connect
    ESP_LOGI(TAG, "=== CSI Debug - waiting 3s ===");
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Step 0: System info");
    ESP_LOGI(TAG, "  Heap: %ld bytes", esp_get_free_heap_size());
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Step 1: LDO
    ESP_LOGI(TAG, "Step 1: LDO init...");
    esp_ldo_channel_handle_t ldo = NULL;
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = 3,
        .voltage_mv = 2500,
    };
    ret = esp_ldo_acquire_channel(&ldo_cfg, &ldo);
    ESP_LOGI(TAG, "  LDO result: %s", esp_err_to_name(ret));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "  LDO FAILED - stopping");
        blink(20, 100);
        return;
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Step 2: I2C
    ESP_LOGI(TAG, "Step 2: I2C init...");
    i2c_master_bus_handle_t bus = NULL;
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = 7,
        .scl_io_num = 8,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ret = i2c_new_master_bus(&bus_cfg, &bus);
    ESP_LOGI(TAG, "  I2C bus: %s", esp_err_to_name(ret));

    i2c_master_dev_handle_t cam = NULL;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMX708_ADDR,
        .scl_speed_hz = 100000,
    };
    ret = i2c_master_bus_add_device(bus, &dev_cfg, &cam);
    ESP_LOGI(TAG, "  I2C device: %s", esp_err_to_name(ret));

    // Read chip ID
    uint8_t addr_buf[2] = {0x00, 0x16};
    uint8_t id = 0;
    ret = i2c_master_transmit_receive(cam, addr_buf, 2, &id, 1, 1000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "  Chip ID high: 0x%02X (ret=%s)", id, esp_err_to_name(ret));
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Step 3: Frame buffer
    ESP_LOGI(TAG, "Step 3: Alloc 153600 bytes (320x240x2)...");
    uint8_t *fb = heap_caps_aligned_calloc(64, 1, 153600, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "  Buffer: %p, heap left: %ld", fb, esp_get_free_heap_size());
    if (!fb) {
        ESP_LOGE(TAG, "  ALLOC FAILED");
        blink(20, 100);
        return;
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Step 4: ISP processor (before CSI per some examples)
    ESP_LOGI(TAG, "Step 4: ISP init...");
    isp_proc_handle_t isp = NULL;
    esp_isp_processor_cfg_t isp_cfg = {
        .clk_hz = 80 * 1000 * 1000,
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8,
        .output_data_color_type = ISP_COLOR_RGB565,
        .has_line_start_packet = false,
        .has_line_end_packet = false,
        .h_res = 320,
        .v_res = 240,
    };
    ret = esp_isp_new_processor(&isp_cfg, &isp);
    ESP_LOGI(TAG, "  ISP new: %s", esp_err_to_name(ret));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "  ISP FAILED - stopping");
        blink(20, 100);
        return;
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Step 4b: ISP enable...");
    ret = esp_isp_enable(isp);
    ESP_LOGI(TAG, "  ISP enable: %s", esp_err_to_name(ret));
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Step 5: CSI controller
    ESP_LOGI(TAG, "Step 5: CSI controller...");
    esp_cam_ctlr_handle_t cam_h = NULL;
    esp_cam_ctlr_csi_config_t csi_cfg = {
        .ctlr_id = 0,
        .h_res = 320,
        .v_res = 240,
        .lane_bit_rate_mbps = 200,
        .input_data_color_type = CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = CAM_CTLR_COLOR_RGB565,
        .data_lane_num = 2,
        .byte_swap_en = false,
        .queue_items = 1,
    };
    ret = esp_cam_new_csi_ctlr(&csi_cfg, &cam_h);
    ESP_LOGI(TAG, "  CSI new: %s", esp_err_to_name(ret));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "  CSI FAILED - stopping");
        blink(20, 100);
        return;
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Step 6: CSI enable...");
    ret = esp_cam_ctlr_enable(cam_h);
    ESP_LOGI(TAG, "  CSI enable: %s", esp_err_to_name(ret));
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Success!
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== ALL STEPS PASSED ===");
    ESP_LOGI(TAG, "CSI + ISP initialized on ESP32-P4 v1.0!");
    ESP_LOGI(TAG, "");

    // Keep printing status forever
    int n = 0;
    while (1) {
        n++;
        ESP_LOGI(TAG, "alive [%d] heap=%ld", n, esp_get_free_heap_size());
        blink(1, 200);
        vTaskDelay(800 / portTICK_PERIOD_MS);
    }
}

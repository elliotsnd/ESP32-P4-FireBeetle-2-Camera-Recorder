/*
 * ESP32-P4 v1.0 CSI Debug - LED blink tells us which step fails
 * 
 * LED PATTERNS (watch the LED on GPIO15):
 *   Boot:  LED ON solid for 2s = firmware started
 *   
 *   Before each step: N slow blinks (1s on, 0.5s off)
 *     1 blink  = Step 1: LDO init
 *     2 blinks = Step 2: I2C + IMX708
 *     3 blinks = Step 3: Buffer alloc
 *     4 blinks = Step 4: ISP new_processor
 *     5 blinks = Step 5: ISP enable  
 *     6 blinks = Step 6: CSI new controller
 *     7 blinks = Step 7: CSI enable
 *
 *   FAILURE: Step N failed → rapid blink N times, 2s pause, repeat forever
 *   SUCCESS: All passed → continuous fast heartbeat (0.2s on/off)
 *   CRASH:   If chip panics mid-step, LED freezes (on or off)
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

static const char *TAG = "CSI_DBG";

#define LED_PIN     15
#define IMX708_ADDR 0x1A

// Slow blink: visible step indicator
static void slow_blink(int n) {
    for (int i = 0; i < n; i++) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(800 / portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(400 / portTICK_PERIOD_MS);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
}

// Error pattern: blink N times fast, pause, repeat forever
static void error_blink_forever(int step_num) {
    while (1) {
        for (int i = 0; i < step_num; i++) {
            gpio_set_level(LED_PIN, 1);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            gpio_set_level(LED_PIN, 0);
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);  // 2s pause between groups
    }
}

// Success pattern: fast continuous heartbeat
static void success_blink_forever(void) {
    while (1) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(150 / portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(150 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    esp_err_t ret;

    // NVS init
    nvs_flash_init();

    // LED setup
    gpio_config_t io = { .pin_bit_mask = (1ULL << LED_PIN), .mode = GPIO_MODE_OUTPUT };
    gpio_config(&io);

    // ====== BOOT INDICATOR: solid LED for 2s ======
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    gpio_set_level(LED_PIN, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " CSI LED Debug - ESP32-P4 v1.0 (ECO1)");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Watch LED on GPIO15 for step indicators");
    ESP_LOGI(TAG, "Heap: %lu bytes free", (unsigned long)esp_get_free_heap_size());
    ESP_LOGI(TAG, "");

    // 15 second delay for serial connection
    for (int i = 15; i > 0; i--) {
        ESP_LOGI(TAG, "Starting in %d seconds...", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "");

    // ====== STEP 1: LDO (1 blink) ======
    ESP_LOGI(TAG, ">>> Step 1: LDO init (1 blink)");
    slow_blink(1);
    
    esp_ldo_channel_handle_t ldo = NULL;
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = 3,
        .voltage_mv = 2500,
    };
    ret = esp_ldo_acquire_channel(&ldo_cfg, &ldo);
    ESP_LOGI(TAG, "    Result: %s (0x%x)", esp_err_to_name(ret), ret);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "*** STEP 1 FAILED: LDO ***");
        error_blink_forever(1);
        // never returns
    }
    ESP_LOGI(TAG, "    Step 1 OK!");
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // ====== STEP 2: I2C + IMX708 (2 blinks) ======
    ESP_LOGI(TAG, ">>> Step 2: I2C + IMX708 (2 blinks)");
    slow_blink(2);
    
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
    ESP_LOGI(TAG, "    I2C bus: %s", esp_err_to_name(ret));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "*** STEP 2 FAILED: I2C bus ***");
        error_blink_forever(2);
    }

    i2c_master_dev_handle_t cam_dev = NULL;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMX708_ADDR,
        .scl_speed_hz = 100000,
    };
    ret = i2c_master_bus_add_device(bus, &dev_cfg, &cam_dev);
    ESP_LOGI(TAG, "    I2C device: %s", esp_err_to_name(ret));

    // Read chip ID (best-effort, don't fail on this)
    uint8_t addr_buf[2] = {0x00, 0x16};
    uint8_t id_hi = 0, id_lo = 0;
    ret = i2c_master_transmit_receive(cam_dev, addr_buf, 2, &id_hi, 1, 1000);
    addr_buf[1] = 0x17;
    i2c_master_transmit_receive(cam_dev, addr_buf, 2, &id_lo, 1, 1000);
    ESP_LOGI(TAG, "    IMX708 Chip ID: 0x%02X%02X", id_hi, id_lo);
    ESP_LOGI(TAG, "    Step 2 OK!");
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // ====== STEP 3: Frame buffer (3 blinks) ======
    ESP_LOGI(TAG, ">>> Step 3: Buffer alloc (3 blinks)");
    slow_blink(3);
    
    size_t buf_size = 320 * 240 * 2;  // 153600 bytes
    ESP_LOGI(TAG, "    Allocating %u bytes (internal RAM)...", (unsigned)buf_size);
    uint8_t *fb = heap_caps_aligned_calloc(64, 1, buf_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "    Buffer: %p, heap left: %lu", fb, (unsigned long)esp_get_free_heap_size());
    if (!fb) {
        ESP_LOGE(TAG, "*** STEP 3 FAILED: Buffer alloc ***");
        error_blink_forever(3);
    }
    ESP_LOGI(TAG, "    Step 3 OK!");
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // ====== STEP 4: ISP new processor (4 blinks) ======
    ESP_LOGI(TAG, ">>> Step 4: ISP new_processor (4 blinks)");
    slow_blink(4);
    
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
    ESP_LOGI(TAG, "    Result: %s (0x%x)", esp_err_to_name(ret), ret);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "*** STEP 4 FAILED: ISP new_processor ***");
        error_blink_forever(4);
    }
    ESP_LOGI(TAG, "    Step 4 OK!");
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // ====== STEP 5: ISP enable (5 blinks) ======
    ESP_LOGI(TAG, ">>> Step 5: ISP enable (5 blinks)");
    slow_blink(5);
    
    ret = esp_isp_enable(isp);
    ESP_LOGI(TAG, "    Result: %s (0x%x)", esp_err_to_name(ret), ret);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "*** STEP 5 FAILED: ISP enable ***");
        error_blink_forever(5);
    }
    ESP_LOGI(TAG, "    Step 5 OK!");
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // ====== STEP 6: CSI new controller (6 blinks) ======
    ESP_LOGI(TAG, ">>> Step 6: CSI new_controller (6 blinks)");
    slow_blink(6);
    
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
    ESP_LOGI(TAG, "    Result: %s (0x%x)", esp_err_to_name(ret), ret);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "*** STEP 6 FAILED: CSI new_controller ***");
        error_blink_forever(6);
    }
    ESP_LOGI(TAG, "    Step 6 OK!");
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // ====== STEP 7: CSI enable (7 blinks) ======
    ESP_LOGI(TAG, ">>> Step 7: CSI enable (7 blinks)");
    slow_blink(7);
    
    ret = esp_cam_ctlr_enable(cam_h);
    ESP_LOGI(TAG, "    Result: %s (0x%x)", esp_err_to_name(ret), ret);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "*** STEP 7 FAILED: CSI enable ***");
        error_blink_forever(7);
    }
    ESP_LOGI(TAG, "    Step 7 OK!");
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // ====== ALL PASSED! ======
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, " ALL 7 STEPS PASSED on ESP32-P4 v1.0!!!");
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Heap remaining: %lu bytes", (unsigned long)esp_get_free_heap_size());
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "LED will now fast-blink = SUCCESS");
    ESP_LOGI(TAG, "");

    // Keep alive with fast heartbeat + serial output
    int n = 0;
    while (1) {
        n++;
        if (n % 10 == 0) {
            ESP_LOGI(TAG, "alive [%d] heap=%lu", n, (unsigned long)esp_get_free_heap_size());
        }
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(150 / portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(150 / portTICK_PERIOD_MS);
    }
}

/*
 * ESP32-P4 v1.0 CSI Capture - No PSRAM, Test Pattern Mode
 * 
 * BREAKTHROUGH RESULTS SO FAR:
 * - ALL CSI/ISP init steps PASS with bk_buffer_dis=true
 * - Got 1 frame from IMX708 (real data!)
 * - Sensor stops after 1 frame (needs PLL/MCLK config for continuous)
 * 
 * This version enables IMX708 test pattern to verify continuous CSI capture
 * independent of proper sensor timing configuration.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

// CSI + ISP headers
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "driver/isp.h"
#include "esp_ldo_regulator.h"

// LEDC for MCLK generation
#include "driver/ledc.h"

static const char *TAG = "CAM";

#define LED_PIN     15
#define IMX708_ADDR 0x1A

// Camera MCLK - try GPIO47 (common camera clock pin on ESP32-P4)
// The IMX708 needs an external clock input (INCK), typically 24MHz
#define CAM_MCLK_PIN  47
#define CAM_MCLK_FREQ 24000000  // 24 MHz

// Frame dimensions
#define FRAME_W     320
#define FRAME_H     240
#define FRAME_BPP   2
#define FRAME_SIZE  (FRAME_W * FRAME_H * FRAME_BPP)

// Double-buffer
static uint8_t *frame_buf[2] = {NULL, NULL};
static volatile int write_buf_idx = 0;
static volatile int frame_count = 0;
static volatile int64_t last_frame_time_us = 0;
static SemaphoreHandle_t frame_done_sem = NULL;

static bool IRAM_ATTR on_get_new_trans(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    trans->buffer = frame_buf[write_buf_idx];
    trans->buflen = FRAME_SIZE;
    return false;
}

static bool IRAM_ATTR on_trans_finished(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    frame_count++;
    last_frame_time_us = esp_timer_get_time();
    write_buf_idx = 1 - write_buf_idx;
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(frame_done_sem, &xHigherPriorityTaskWoken);
    return (xHigherPriorityTaskWoken == pdTRUE);
}

static void blink(int n, int ms) {
    for (int i = 0; i < n; i++) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(ms / portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(ms / portTICK_PERIOD_MS);
    }
}

// IMX708 register access
static i2c_master_dev_handle_t cam_dev = NULL;

static esp_err_t imx708_write_reg(uint16_t reg, uint8_t val)
{
    uint8_t buf[3] = { (reg >> 8) & 0xFF, reg & 0xFF, val };
    return i2c_master_transmit(cam_dev, buf, 3, 1000);
}

static esp_err_t imx708_read_reg(uint16_t reg, uint8_t *val)
{
    uint8_t addr_buf[2] = { (reg >> 8) & 0xFF, reg & 0xFF };
    return i2c_master_transmit_receive(cam_dev, addr_buf, 2, val, 1, 1000);
}

static esp_err_t imx708_write_reg16(uint16_t reg, uint16_t val)
{
    imx708_write_reg(reg, (val >> 8) & 0xFF);
    return imx708_write_reg(reg + 1, val & 0xFF);
}

// Initialize MCLK output via LEDC timer
static esp_err_t init_camera_mclk(void)
{
    ESP_LOGI(TAG, "  Generating %d Hz MCLK on GPIO%d", CAM_MCLK_FREQ, CAM_MCLK_PIN);
    
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_1_BIT,  // Minimal resolution for square wave
        .timer_num = LEDC_TIMER_0,
        .freq_hz = CAM_MCLK_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK) return ret;
    
    ledc_channel_config_t chan_cfg = {
        .gpio_num = CAM_MCLK_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 1,  // 50% duty for 1-bit resolution
        .hpoint = 0,
    };
    return ledc_channel_config(&chan_cfg);
}

// IMX708 minimal init for 320x240 RAW8 output
// Based on libcamera/Raspberry Pi Camera Module 3 register tables
static void imx708_init_320x240(void)
{
    ESP_LOGI(TAG, "Configuring IMX708...");
    
    // Software reset
    imx708_write_reg(0x0103, 0x01);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    
    // External clock frequency (24 MHz = 0x1800 in units of 1/256 MHz? or direct)
    // EXCK_FREQ[15:8] and EXCK_FREQ[7:0]  
    // For 24 MHz: 0x18 = 24
    imx708_write_reg16(0x0136, 0x1800);  // 24.00 MHz
    
    // MIPI output: 2 lanes
    imx708_write_reg(0x0114, 0x01);  // CSI_LANE_MODE: 0x01 = 2 lanes
    
    // CSI data format: RAW8
    imx708_write_reg16(0x0112, 0x0808);  // CSI_DT_FMT: RAW8
    
    // PLL settings for 24MHz input, ~200Mbps MIPI output
    // Video timing PLL (VTPXCK)
    imx708_write_reg(0x0301, 0x08);  // VT_PIX_CLK_DIV
    imx708_write_reg(0x0303, 0x01);  // VT_SYS_CLK_DIV
    imx708_write_reg16(0x0305, 0x0032);  // PRE_PLL_CLK_DIV = 50? Try different values
    imx708_write_reg16(0x0307, 0x0096);  // PLL_MULTIPLIER = 150
    
    // Output timing PLL
    imx708_write_reg(0x0309, 0x08);  // OP_PIX_CLK_DIV
    imx708_write_reg(0x030B, 0x02);  // OP_SYS_CLK_DIV
    imx708_write_reg16(0x030D, 0x0004);  // PRE_PLL_CLK_DIV2
    imx708_write_reg16(0x030F, 0x0058);  // PLL_MULTIPLIER2 = 88
    
    // Timing
    imx708_write_reg16(0x0340, 0x0120);  // FRM_LENGTH_LINES = 288
    imx708_write_reg16(0x0342, 0x0500);  // LINE_LENGTH_PCK = 1280
    
    // Crop/window for 320x240
    // Full sensor is 4608x2592, we need heavy binning/cropping
    // X start/end
    imx708_write_reg16(0x0344, 0x0000);  // X_ADDR_START = 0
    imx708_write_reg16(0x0348, 0x0FFF);  // X_ADDR_END = 4095
    // Y start/end
    imx708_write_reg16(0x0346, 0x0000);  // Y_ADDR_START = 0  
    imx708_write_reg16(0x034A, 0x0BFF);  // Y_ADDR_END = 3071
    
    // Output size
    imx708_write_reg16(0x034C, FRAME_W);  // X_OUTPUT_SIZE = 320
    imx708_write_reg16(0x034E, FRAME_H);  // Y_OUTPUT_SIZE = 240
    
    // Binning mode (to downsample from full res to 320x240)
    // 4x4 binning: 4608/4 = 1152, 2592/4 = 648 → still need crop/digital zoom
    // Or use subsampling
    imx708_write_reg(0x0900, 0x01);  // BINNING_MODE: enable
    imx708_write_reg(0x0901, 0x44);  // BINNING_TYPE: 4x4

    // Digital crop (after binning: 1152x648 → 320x240)
    imx708_write_reg16(0x0408, 0x0000);  // DIG_CROP_X_OFFSET
    imx708_write_reg16(0x040A, 0x0000);  // DIG_CROP_Y_OFFSET
    imx708_write_reg16(0x040C, FRAME_W);  // DIG_CROP_IMAGE_WIDTH
    imx708_write_reg16(0x040E, FRAME_H);  // DIG_CROP_IMAGE_HEIGHT

    // Scaling
    imx708_write_reg16(0x0404, 0x0010);  // SCALE_M = 16 (1x)
    
    // Exposure
    imx708_write_reg16(0x0202, 0x0100);  // COARSE_INTEG_TIME = 256 lines
    
    // Gain
    imx708_write_reg16(0x0204, 0x0040);  // ANA_GAIN = 64 (1x)
    
    // Test pattern (uncomment for debug)
    // 0=off, 1=solid color, 2=color bars, 3=gray ramp, 4=PN9
    imx708_write_reg16(0x0600, 0x0002);  // TEST_PATTERN_MODE = Color Bars
    
    // Start streaming
    imx708_write_reg(0x0100, 0x01);  // MODE_SELECT = streaming
    
    ESP_LOGI(TAG, "IMX708 configured + streaming (test pattern = color bars)");
}

void app_main(void)
{
    esp_err_t ret;

    nvs_flash_init();

    // LED
    gpio_config_t io = { .pin_bit_mask = (1ULL << LED_PIN), .mode = GPIO_MODE_OUTPUT };
    gpio_config(&io);

    gpio_set_level(LED_PIN, 1);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    gpio_set_level(LED_PIN, 0);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " ESP32-P4 CSI Capture v2 (No PSRAM)");
    ESP_LOGI(TAG, " Test Pattern + MCLK Generation");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Heap: %lu bytes free", (unsigned long)esp_get_free_heap_size());

    // 10 second delay for serial
    for (int i = 10; i > 0; i--) {
        ESP_LOGI(TAG, "Starting in %d...", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // ====== Step 1: LDO ======
    ESP_LOGI(TAG, "[1/8] LDO init");
    esp_ldo_channel_handle_t ldo = NULL;
    esp_ldo_channel_config_t ldo_cfg = { .chan_id = 3, .voltage_mv = 2500 };
    ret = esp_ldo_acquire_channel(&ldo_cfg, &ldo);
    ESP_LOGI(TAG, "  → %s", esp_err_to_name(ret));
    if (ret != ESP_OK) { blink(1, 1000); while(1) vTaskDelay(1000 / portTICK_PERIOD_MS); }

    // ====== Step 2: MCLK for camera ======
    ESP_LOGI(TAG, "[2/8] Camera MCLK init (24MHz on GPIO%d)", CAM_MCLK_PIN);
    ret = init_camera_mclk();
    ESP_LOGI(TAG, "  → %s", esp_err_to_name(ret));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "  MCLK failed - sensor may not stream continuously");
        // Don't abort - sensor might have internal oscillator
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);  // Let MCLK stabilize

    // ====== Step 3: I2C + IMX708 ======
    ESP_LOGI(TAG, "[3/8] I2C + IMX708");
    i2c_master_bus_handle_t bus = NULL;
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = 7, .scl_io_num = 8,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ret = i2c_new_master_bus(&bus_cfg, &bus);
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMX708_ADDR,
        .scl_speed_hz = 100000,
    };
    ret = i2c_master_bus_add_device(bus, &dev_cfg, &cam_dev);

    uint8_t id_hi = 0, id_lo = 0;
    imx708_read_reg(0x0016, &id_hi);
    imx708_read_reg(0x0017, &id_lo);
    ESP_LOGI(TAG, "  Chip ID: 0x%02X%02X %s", id_hi, id_lo,
             (id_hi == 0x07 && id_lo == 0x08) ? "OK" : "FAIL");

    // ====== Step 4: Frame buffers ======
    ESP_LOGI(TAG, "[4/8] Frame buffers (2x %d bytes)", FRAME_SIZE);
    frame_buf[0] = heap_caps_aligned_calloc(64, 1, FRAME_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    frame_buf[1] = heap_caps_aligned_calloc(64, 1, FRAME_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "  buf[0]=%p buf[1]=%p heap=%lu", frame_buf[0], frame_buf[1],
             (unsigned long)esp_get_free_heap_size());
    if (!frame_buf[0] || !frame_buf[1]) { blink(4, 1000); while(1) vTaskDelay(1000 / portTICK_PERIOD_MS); }
    frame_done_sem = xSemaphoreCreateBinary();

    // ====== Step 5: ISP ======
    ESP_LOGI(TAG, "[5/8] ISP processor");
    isp_proc_handle_t isp = NULL;
    esp_isp_processor_cfg_t isp_cfg = {
        .clk_hz = 80000000,
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8,
        .output_data_color_type = ISP_COLOR_RGB565,
        .has_line_start_packet = false,
        .has_line_end_packet = false,
        .h_res = FRAME_W,
        .v_res = FRAME_H,
    };
    ret = esp_isp_new_processor(&isp_cfg, &isp);
    ESP_LOGI(TAG, "  new: %s", esp_err_to_name(ret));
    if (ret != ESP_OK) { blink(5, 1000); while(1) vTaskDelay(1000 / portTICK_PERIOD_MS); }
    ret = esp_isp_enable(isp);
    ESP_LOGI(TAG, "  enable: %s", esp_err_to_name(ret));

    // ====== Step 6: CSI controller ======
    ESP_LOGI(TAG, "[6/8] CSI controller (bk_buffer_dis=true)");
    esp_cam_ctlr_handle_t cam_h = NULL;
    esp_cam_ctlr_csi_config_t csi_cfg = {
        .ctlr_id = 0,
        .h_res = FRAME_W,
        .v_res = FRAME_H,
        .lane_bit_rate_mbps = 200,
        .input_data_color_type = CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = CAM_CTLR_COLOR_RGB565,
        .data_lane_num = 2,
        .byte_swap_en = false,
        .queue_items = 1,
        .bk_buffer_dis = true,
    };
    ret = esp_cam_new_csi_ctlr(&csi_cfg, &cam_h);
    ESP_LOGI(TAG, "  → %s", esp_err_to_name(ret));
    if (ret != ESP_OK) { blink(6, 1000); while(1) vTaskDelay(1000 / portTICK_PERIOD_MS); }

    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans = on_get_new_trans,
        .on_trans_finished = on_trans_finished,
    };
    esp_cam_ctlr_register_event_callbacks(cam_h, &cbs, NULL);

    // ====== Step 7: CSI enable ======
    ESP_LOGI(TAG, "[7/8] CSI enable");
    ret = esp_cam_ctlr_enable(cam_h);
    ESP_LOGI(TAG, "  → %s", esp_err_to_name(ret));
    if (ret != ESP_OK) { blink(7, 1000); while(1) vTaskDelay(1000 / portTICK_PERIOD_MS); }

    // ====== Step 8: Start CSI + configure sensor ======
    ESP_LOGI(TAG, "[8/8] CSI start + IMX708 config");
    ret = esp_cam_ctlr_start(cam_h);
    ESP_LOGI(TAG, "  CSI start: %s", esp_err_to_name(ret));

    // Configure IMX708 with test pattern
    imx708_init_320x240();
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== Monitoring frames ===");

    // ====== Main loop ======
    int prev_count = 0;
    int seconds = 0;
    int64_t fps_start = esp_timer_get_time();
    
    while (1) {
        if (xSemaphoreTake(frame_done_sem, 2000 / portTICK_PERIOD_MS) == pdTRUE) {
            int64_t now = esp_timer_get_time();
            int elapsed_ms = (int)((now - fps_start) / 1000);
            int delta = frame_count - prev_count;
            
            if (elapsed_ms >= 1000 || delta >= 1) {
                float fps = (delta * 1000000.0f) / (float)(now - fps_start);
                fps_start = now;
                prev_count = frame_count;
                seconds++;
                
                int read_idx = 1 - write_buf_idx;
                uint8_t *fb = frame_buf[read_idx];
                
                // Check if frame has actual data (not all zeros)
                int nonzero = 0;
                for (int i = 0; i < 100; i++) nonzero += (fb[i] != 0);
                
                ESP_LOGI(TAG, "[%ds] frames=%d fps=%.1f nonzero=%d data=%02X%02X%02X%02X %02X%02X%02X%02X",
                         seconds, frame_count, fps, nonzero,
                         fb[0], fb[1], fb[2], fb[3], fb[4], fb[5], fb[6], fb[7]);
                
                gpio_set_level(LED_PIN, 1);
                vTaskDelay(30 / portTICK_PERIOD_MS);
                gpio_set_level(LED_PIN, 0);
            }
        } else {
            seconds += 2;
            ESP_LOGW(TAG, "[%ds] No frame (timeout) total=%d", seconds, frame_count);
            
            // Read some IMX708 status registers for debugging
            uint8_t mode = 0, streaming = 0;
            imx708_read_reg(0x0100, &mode);
            imx708_read_reg(0x0005, &streaming);
            ESP_LOGW(TAG, "  MODE_SELECT=0x%02X FRAME_COUNT=0x%02X", mode, streaming);
            
            blink(2, 100);
        }
    }
}

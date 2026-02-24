/*
 * ESP32-P4 v1.0 CSI Capture - No PSRAM mode
 * Uses bk_buffer_dis=true to skip PSRAM backup buffer
 * Provides frame buffer via on_get_new_trans callback
 * 
 * Fixed init order following official example:
 * LDO → I2C → ISP new → ISP enable → CSI new → CSI enable → CSI start → Receive
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

// CSI + ISP headers
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "driver/isp.h"
#include "esp_ldo_regulator.h"

static const char *TAG = "CAM";

#define LED_PIN     15
#define IMX708_ADDR 0x1A

// Frame dimensions - small to fit in internal RAM
#define FRAME_W     320
#define FRAME_H     240
#define FRAME_BPP   2       // RGB565 = 2 bytes per pixel
#define FRAME_SIZE  (FRAME_W * FRAME_H * FRAME_BPP)  // 153600 bytes

// Double-buffer for capture
static uint8_t *frame_buf[2] = {NULL, NULL};
static volatile int write_buf_idx = 0;
static volatile int frame_count = 0;
static SemaphoreHandle_t frame_done_sem = NULL;

// Callback: CSI needs a new buffer to write into
static bool IRAM_ATTR on_get_new_trans(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    // Give it the current write buffer
    trans->buffer = frame_buf[write_buf_idx];
    trans->buflen = FRAME_SIZE;
    return false;  // no need to yield
}

// Callback: CSI finished writing a frame
static bool IRAM_ATTR on_trans_finished(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    frame_count++;
    // Swap buffers
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

// Write IMX708 register (16-bit address, 8-bit value)
static esp_err_t imx708_write_reg(i2c_master_dev_handle_t dev, uint16_t reg, uint8_t val)
{
    uint8_t buf[3] = { (reg >> 8) & 0xFF, reg & 0xFF, val };
    return i2c_master_transmit(dev, buf, 3, 1000);
}

// Read IMX708 register (16-bit address, 8-bit value)
static esp_err_t imx708_read_reg(i2c_master_dev_handle_t dev, uint16_t reg, uint8_t *val)
{
    uint8_t addr_buf[2] = { (reg >> 8) & 0xFF, reg & 0xFF };
    return i2c_master_transmit_receive(dev, addr_buf, 2, val, 1, 1000);
}

void app_main(void)
{
    esp_err_t ret;

    nvs_flash_init();

    // LED
    gpio_config_t io = { .pin_bit_mask = (1ULL << LED_PIN), .mode = GPIO_MODE_OUTPUT };
    gpio_config(&io);

    // Boot indicator
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    gpio_set_level(LED_PIN, 0);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " ESP32-P4 CSI Capture (No PSRAM Mode)");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Resolution: %dx%d RGB565", FRAME_W, FRAME_H);
    ESP_LOGI(TAG, "Frame size: %d bytes", FRAME_SIZE);
    ESP_LOGI(TAG, "Heap: %lu bytes free", (unsigned long)esp_get_free_heap_size());
    ESP_LOGI(TAG, "");

    // 10 second delay for serial
    for (int i = 10; i > 0; i--) {
        ESP_LOGI(TAG, "Starting in %d...", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // ====== Step 1: LDO for MIPI PHY ======
    ESP_LOGI(TAG, "[1/7] LDO init (chan=3, 2500mV)");
    esp_ldo_channel_handle_t ldo = NULL;
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = 3,
        .voltage_mv = 2500,
    };
    ret = esp_ldo_acquire_channel(&ldo_cfg, &ldo);
    ESP_LOGI(TAG, "  → %s", esp_err_to_name(ret));
    if (ret != ESP_OK) { blink(1, 1000); while(1) vTaskDelay(1000 / portTICK_PERIOD_MS); }

    // ====== Step 2: I2C + IMX708 verify ======
    ESP_LOGI(TAG, "[2/7] I2C + IMX708 init");
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

    i2c_master_dev_handle_t cam_dev = NULL;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMX708_ADDR,
        .scl_speed_hz = 100000,
    };
    ret = i2c_master_bus_add_device(bus, &dev_cfg, &cam_dev);

    uint8_t id_hi = 0, id_lo = 0;
    imx708_read_reg(cam_dev, 0x0016, &id_hi);
    imx708_read_reg(cam_dev, 0x0017, &id_lo);
    ESP_LOGI(TAG, "  IMX708 Chip ID: 0x%02X%02X %s", id_hi, id_lo,
             (id_hi == 0x07 && id_lo == 0x08) ? "✓" : "MISMATCH!");
    if (id_hi != 0x07 || id_lo != 0x08) { blink(2, 1000); while(1) vTaskDelay(1000 / portTICK_PERIOD_MS); }

    // ====== Step 3: Allocate frame buffers ======
    ESP_LOGI(TAG, "[3/7] Allocate frame buffers (2x %d bytes)", FRAME_SIZE);
    frame_buf[0] = heap_caps_aligned_calloc(64, 1, FRAME_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    frame_buf[1] = heap_caps_aligned_calloc(64, 1, FRAME_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "  buf[0]=%p buf[1]=%p heap_left=%lu", frame_buf[0], frame_buf[1],
             (unsigned long)esp_get_free_heap_size());
    if (!frame_buf[0] || !frame_buf[1]) {
        ESP_LOGE(TAG, "  Buffer alloc failed! Need 2x %d = %d bytes", FRAME_SIZE, FRAME_SIZE * 2);
        blink(3, 1000); while(1) vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    frame_done_sem = xSemaphoreCreateBinary();

    // ====== Step 4: ISP processor ======
    ESP_LOGI(TAG, "[4/7] ISP new_processor");
    isp_proc_handle_t isp = NULL;
    esp_isp_processor_cfg_t isp_cfg = {
        .clk_hz = 80 * 1000 * 1000,
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8,
        .output_data_color_type = ISP_COLOR_RGB565,
        .has_line_start_packet = false,
        .has_line_end_packet = false,
        .h_res = FRAME_W,
        .v_res = FRAME_H,
    };
    ret = esp_isp_new_processor(&isp_cfg, &isp);
    ESP_LOGI(TAG, "  → %s", esp_err_to_name(ret));
    if (ret != ESP_OK) { blink(4, 1000); while(1) vTaskDelay(1000 / portTICK_PERIOD_MS); }

    // ====== Step 5: ISP enable ======
    ESP_LOGI(TAG, "[5/7] ISP enable");
    ret = esp_isp_enable(isp);
    ESP_LOGI(TAG, "  → %s", esp_err_to_name(ret));
    if (ret != ESP_OK) { blink(5, 1000); while(1) vTaskDelay(1000 / portTICK_PERIOD_MS); }

    // ====== Step 6: CSI controller (with bk_buffer_dis!) ======
    ESP_LOGI(TAG, "[6/7] CSI new_controller (bk_buffer_dis=true, NO PSRAM needed)");
    ESP_LOGI(TAG, "  Heap before CSI: %lu", (unsigned long)esp_get_free_heap_size());
    
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
        .bk_buffer_dis = true,  // ← KEY: Skip PSRAM backup buffer!
    };
    ret = esp_cam_new_csi_ctlr(&csi_cfg, &cam_h);
    ESP_LOGI(TAG, "  → %s", esp_err_to_name(ret));
    ESP_LOGI(TAG, "  Heap after CSI: %lu", (unsigned long)esp_get_free_heap_size());
    if (ret != ESP_OK) { blink(6, 1000); while(1) vTaskDelay(1000 / portTICK_PERIOD_MS); }

    // Register callbacks BEFORE enable
    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans = on_get_new_trans,
        .on_trans_finished = on_trans_finished,
    };
    ret = esp_cam_ctlr_register_event_callbacks(cam_h, &cbs, NULL);
    ESP_LOGI(TAG, "  Callbacks registered: %s", esp_err_to_name(ret));

    // ====== Step 7: CSI enable ======
    ESP_LOGI(TAG, "[7/7] CSI enable");
    ret = esp_cam_ctlr_enable(cam_h);
    ESP_LOGI(TAG, "  → %s", esp_err_to_name(ret));
    if (ret != ESP_OK) { blink(7, 1000); while(1) vTaskDelay(1000 / portTICK_PERIOD_MS); }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, " ALL INIT STEPS PASSED!");
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Heap remaining: %lu bytes", (unsigned long)esp_get_free_heap_size());
    ESP_LOGI(TAG, "");

    // ====== Configure IMX708 for streaming ======
    ESP_LOGI(TAG, "Configuring IMX708 for 320x240 RAW8...");
    
    // Software reset
    imx708_write_reg(cam_dev, 0x0103, 0x01);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Basic IMX708 init registers for 320x240 output
    // Set 2-lane MIPI CSI
    imx708_write_reg(cam_dev, 0x0114, 0x01);  // CSI_LANE_MODE: 2 lanes
    
    // Analog gain
    imx708_write_reg(cam_dev, 0x0204, 0x00);  // ANA_GAIN_GLOBAL[15:8]
    imx708_write_reg(cam_dev, 0x0205, 0x40);  // ANA_GAIN_GLOBAL[7:0] = 64 (1x)

    // Frame length (vertical timing)
    imx708_write_reg(cam_dev, 0x0340, 0x01);  // FRM_LENGTH_LINES[15:8]
    imx708_write_reg(cam_dev, 0x0341, 0x20);  // FRM_LENGTH_LINES[7:0] = 288

    // Line length (horizontal timing)
    imx708_write_reg(cam_dev, 0x0342, 0x05);  // LINE_LENGTH_PCK[15:8]
    imx708_write_reg(cam_dev, 0x0343, 0x00);  // LINE_LENGTH_PCK[7:0] = 1280

    // Output size
    imx708_write_reg(cam_dev, 0x034C, (FRAME_W >> 8) & 0xFF);  // X_OUTPUT_SIZE[15:8]
    imx708_write_reg(cam_dev, 0x034D, FRAME_W & 0xFF);          // X_OUTPUT_SIZE[7:0]
    imx708_write_reg(cam_dev, 0x034E, (FRAME_H >> 8) & 0xFF);  // Y_OUTPUT_SIZE[15:8]
    imx708_write_reg(cam_dev, 0x034F, FRAME_H & 0xFF);          // Y_OUTPUT_SIZE[7:0]

    // RAW8 output format
    imx708_write_reg(cam_dev, 0x0112, 0x08);  // CSI_DT_FMT[15:8] = RAW8
    imx708_write_reg(cam_dev, 0x0113, 0x08);  // CSI_DT_FMT[7:0]

    // Integration time (exposure)
    imx708_write_reg(cam_dev, 0x0202, 0x01);  // COARSE_INTEG_TIME[15:8]
    imx708_write_reg(cam_dev, 0x0203, 0x00);  // COARSE_INTEG_TIME[7:0] = 256

    ESP_LOGI(TAG, "IMX708 registers written");

    // ====== Start CSI capture ======
    ESP_LOGI(TAG, "Starting CSI capture...");
    ret = esp_cam_ctlr_start(cam_h);
    ESP_LOGI(TAG, "CSI start: %s", esp_err_to_name(ret));

    // Start IMX708 streaming
    imx708_write_reg(cam_dev, 0x0100, 0x01);  // MODE_SELECT: streaming
    ESP_LOGI(TAG, "IMX708 streaming started!");
    ESP_LOGI(TAG, "");

    // ====== Main loop: monitor frames ======
    int last_count = 0;
    int seconds = 0;
    
    while (1) {
        // Wait up to 2 seconds for a frame
        if (xSemaphoreTake(frame_done_sem, 2000 / portTICK_PERIOD_MS) == pdTRUE) {
            seconds++;
            int fps = frame_count - last_count;
            last_count = frame_count;
            
            // Sample first few bytes of the latest frame
            int read_idx = 1 - write_buf_idx;  // read from the completed buffer
            uint8_t *fb = frame_buf[read_idx];
            
            ESP_LOGI(TAG, "[%ds] Frames: %d | FPS: ~%d | Bytes: %02X %02X %02X %02X %02X %02X %02X %02X",
                     seconds, frame_count, fps,
                     fb[0], fb[1], fb[2], fb[3], fb[4], fb[5], fb[6], fb[7]);
            
            // Blink LED on each report
            gpio_set_level(LED_PIN, 1);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            gpio_set_level(LED_PIN, 0);
        } else {
            seconds += 2;
            ESP_LOGW(TAG, "[%ds] No frame received (timeout) | Total frames: %d", seconds, frame_count);
            blink(2, 100);
        }
    }
}

// ===== V8: FRAME HEX DUMP FOR IMAGE VERIFICATION =====
// Gain: 8x analog (0x0380), 1x digital (0x0100)
// Exposure: 1314 lines (MAX)
// After 500 frames (~10s warmup): hex dump one full 320x240 RGB565 frame
// Format: ===FRAME_START=== / hex rows / ===FRAME_END===
// Then: continuous streaming with periodic analysis
// ======================================================

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "driver/isp.h"
#include "esp_ldo_regulator.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"

static const char *TAG = "CAM";

// ===== IMX708 REGISTER TABLES (Linux kernel reference) =====

static const uint8_t mode_common_regs[][3] = {
    {0x01, 0x00, 0x00}, {0x01, 0x36, 0x27}, {0x02, 0x70, 0x00}, {0x02, 0x71, 0x10},
    {0x02, 0x72, 0x16}, {0x02, 0x73, 0x00}, {0x02, 0x74, 0x1F}, {0x02, 0x75, 0x00},
    {0x00, 0x01, 0x00}, {0x03, 0x06, 0x00}, {0x03, 0x07, 0x00}, {0x46, 0x15, 0x01},
    {0x46, 0x16, 0x01}, {0x46, 0x17, 0x02}, {0x46, 0x1B, 0x01}, {0x09, 0x0C, 0x01},
    {0x09, 0x0D, 0x02}, {0x09, 0x0E, 0x03}, {0x09, 0x0F, 0x0A}, {0x09, 0x10, 0x0A},
    {0x09, 0x11, 0x0A}, {0x09, 0x12, 0x0A}, {0x09, 0x13, 0x0A}, {0x09, 0x14, 0x0A},
    {0x09, 0x15, 0x0A}, {0x09, 0x16, 0x0A}, {0x09, 0x17, 0x0A}, {0x09, 0x18, 0x0A},
    {0x09, 0x19, 0x0A}, {0x09, 0x1A, 0x0A}, {0x09, 0x1B, 0x0A}, {0x09, 0x1C, 0x0A},
    {0x09, 0x1D, 0x0A}, {0x09, 0x1E, 0x0A}, {0x31, 0x00, 0x00}, {0x31, 0x16, 0x01},
    {0x31, 0x17, 0x00}, {0x31, 0x18, 0xC8}, {0x31, 0x19, 0x00}, {0x31, 0x1A, 0xE7},
    {0x34, 0x0C, 0x00}, {0x34, 0x0D, 0x04}, {0x35, 0x01, 0x02}, {0x34, 0x0D, 0x04},
    {0x34, 0x0E, 0x3C}, {0x4E, 0x42, 0x05}, {0xD2, 0x1A, 0x00}, {0xD2, 0x1B, 0xC8},
    {0xD2, 0x1C, 0x01}, {0xD2, 0x1D, 0x5E},
    {0xFF, 0xFF, 0xFF}
};

static const uint8_t mode_2x2binned_regs[][3] = {
    {0x01, 0x12, 0x0A}, {0x01, 0x13, 0x0A}, {0x01, 0x14, 0x01},
    {0x03, 0x44, 0x00}, {0x03, 0x45, 0x00}, {0x03, 0x46, 0x00}, {0x03, 0x47, 0x00},
    {0x03, 0x48, 0x11}, {0x03, 0x49, 0xFF}, {0x03, 0x4A, 0x0A}, {0x03, 0x4B, 0x1F},
    {0x03, 0x81, 0x00}, {0x03, 0x83, 0x01}, {0x04, 0x00, 0x00}, {0x04, 0x01, 0x00},
    {0x04, 0x04, 0x12}, {0x04, 0x05, 0x00}, {0x04, 0x08, 0x0A}, {0x04, 0x09, 0x20},
    {0x04, 0x0C, 0x00}, {0x04, 0x0D, 0x00}, {0x04, 0x0E, 0x00}, {0x04, 0x0F, 0x00},
    {0x30, 0x20, 0x10}, {0x30, 0x21, 0x40}, {0x30, 0x22, 0x04}, {0x30, 0x23, 0x20},
    {0x30, 0x24, 0x07}, {0x30, 0x25, 0x00}, {0x30, 0x28, 0x28}, {0x30, 0x29, 0x42},
    {0x30, 0x2A, 0x00}, {0x30, 0x2B, 0x04}, {0x30, 0x2C, 0x02}, {0x30, 0x2D, 0x80},
    {0x30, 0x30, 0x14}, {0x30, 0x31, 0x42}, {0x30, 0x32, 0x00}, {0x30, 0x33, 0x08},
    {0x30, 0x34, 0x02}, {0x30, 0x35, 0x80}, {0x30, 0x50, 0x40}, {0x30, 0x56, 0x00},
    {0x30, 0x57, 0x9C}, {0x30, 0x58, 0x44}, {0x31, 0x02, 0x01},
    {0x09, 0x00, 0x01}, {0x09, 0x01, 0x22},
    {0xFF, 0xFF, 0xFF}
};

static const uint8_t link_450Mhz_regs[][3] = {
    {0x08, 0x20, 0x00}, {0x08, 0x21, 0x00}, {0x08, 0x22, 0x00}, {0x08, 0x23, 0x84},
    {0x08, 0x24, 0x00}, {0x08, 0x25, 0x00}, {0x08, 0x26, 0x01}, {0x08, 0x27, 0x07},
    {0xFF, 0xFF, 0xFF}
};

// 320x240 RAW8 with 8x analog gain, 1x digital gain, max exposure
static const uint8_t override_320x240_raw8[][3] = {
    {0x03, 0x40, 0x05}, {0x03, 0x41, 0x42},   // Frame length = 1346
    {0x03, 0x42, 0x0A}, {0x03, 0x43, 0x20},   // Line length = 2592
    {0x03, 0x4C, 0x01}, {0x03, 0x4D, 0x40},   // Width = 320
    {0x03, 0x4E, 0x00}, {0x03, 0x4F, 0xF0},   // Height = 240
    {0x30, 0x28, 0x00}, {0x30, 0x29, 0x00},   // 2x2 binning adjustments
    {0x01, 0x12, 0x0A}, {0x01, 0x13, 0x0A},   // CSI data type RAW8
    {0x01, 0x14, 0x01},                         // Data type selector
    // Exposure = 1314 lines (0x0522) - MAX
    {0x02, 0x02, 0x05}, {0x02, 0x03, 0x22},
    // Analog gain = 0x0380 = 8x (formula: 1024/(1024-0x380) = 1024/128 = 8)
    {0x02, 0x04, 0x03}, {0x02, 0x05, 0x80},
    // Digital gains all 1x (0x0100)
    {0x02, 0x0E, 0x01}, {0x02, 0x0F, 0x00},   // GR digital gain
    {0x02, 0x10, 0x01}, {0x02, 0x11, 0x00},   // R digital gain
    {0x02, 0x12, 0x01}, {0x02, 0x13, 0x00},   // B digital gain
    {0x02, 0x14, 0x01}, {0x02, 0x15, 0x00},   // GB digital gain
    {0xFF, 0xFF, 0xFF}
};

// ===== GLOBALS =====
#define IMG_WIDTH  320
#define IMG_HEIGHT 240
#define IMG_BPP    2
#define FRAME_SIZE (IMG_WIDTH * IMG_HEIGHT * IMG_BPP)

static uint8_t *frame_buf[2] = {NULL, NULL};
static int frame_count = 0;
static volatile bool dump_requested = false;
static volatile bool dump_done = false;
static volatile uint8_t *dump_buf_ptr = NULL;
static esp_cam_ctlr_handle_t cam_handle = NULL;
static SemaphoreHandle_t dump_sem = NULL;
static QueueHandle_t trans_queue = NULL; // Our transaction queue

// ===== I2C FUNCTIONS =====
static i2c_master_dev_handle_t sensor_dev = NULL;

static esp_err_t imx708_write_reg(uint16_t reg, uint8_t val) {
    uint8_t buf[3] = { reg >> 8, reg & 0xFF, val };
    return i2c_master_transmit(sensor_dev, buf, 3, 100);
}

static uint8_t imx708_read_reg(uint16_t reg) {
    uint8_t addr[2] = { reg >> 8, reg & 0xFF };
    uint8_t val = 0;
    i2c_master_transmit_receive(sensor_dev, addr, 2, &val, 1, 100);
    return val;
}

static void imx708_write_table(const uint8_t table[][3]) {
    for (int i = 0; table[i][0] != 0xFF || table[i][1] != 0xFF; i++) {
        uint16_t reg = (table[i][0] << 8) | table[i][1];
        imx708_write_reg(reg, table[i][2]);
    }
}

// ===== HEX DUMP FUNCTION =====
static const char hx[] = "0123456789ABCDEF";

static void dump_frame_hex(const uint8_t *buf, int width, int height) {
    static char hex_line[IMG_WIDTH * 4 + 16];
    ESP_LOGW(TAG, "");
    ESP_LOGW(TAG, "===FRAME_START===");
    // Use printf for data lines (no ESP_LOG overhead)
    printf("W=%d H=%d FMT=RGB565_LE\n", width, height);

    for (int y = 0; y < height; y++) {
        const uint8_t *row = buf + y * width * 2;
        char *p = hex_line;
        for (int x = 0; x < width * 2; x++) {
            *p++ = hx[row[x] >> 4];
            *p++ = hx[row[x] & 0x0F];
        }
        *p++ = '\n';
        *p = '\0';
        fputs(hex_line, stdout);
        fflush(stdout);
        if (y % 20 == 0) vTaskDelay(1); // yield periodically
    }

    ESP_LOGW(TAG, "===FRAME_END===");
    ESP_LOGW(TAG, "");
}

// ===== CSI CALLBACKS =====
// Get new transaction callback (called by CSI driver when it needs a new buffer)
static IRAM_ATTR bool csi_get_new_trans_cb(esp_cam_ctlr_handle_t handle,
    esp_cam_ctlr_trans_t *trans, void *user_data)
{
    BaseType_t high_task_woken = pdFALSE;
    if (xQueueReceiveFromISR(trans_queue, trans, &high_task_woken) == pdTRUE) {
        return high_task_woken == pdTRUE;
    }
    // No transaction available — clear buffer pointer so driver knows
    trans->buffer = NULL;
    trans->buflen = 0;
    return false;
}

// Frame done callback
static bool csi_frame_done_cb(esp_cam_ctlr_handle_t handle,
    esp_cam_ctlr_trans_t *trans, void *user_data)
{
    frame_count++;

    // Log every frame for debugging
    if (frame_count <= 15) {
        ESP_DRAM_LOGI(TAG, "Frame #%d received, buf=%p", frame_count, trans->buffer);
    }

    // Check if we should dump this frame
    if (frame_count == 10 && !dump_done) {
        dump_buf_ptr = (uint8_t *)trans->buffer;
        dump_requested = true;
        // DON'T submit next receive — pause CSI during dump
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(dump_sem, &xHigherPriorityTaskWoken);
        return xHigherPriorityTaskWoken == pdTRUE;
    }

    // Normal: submit same buffer back to our queue for reuse
    esp_cam_ctlr_trans_t new_trans = {
        .buffer = trans->buffer,
        .buflen = FRAME_SIZE,
    };
    BaseType_t high_task_woken = pdFALSE;
    xQueueSendFromISR(trans_queue, &new_trans, &high_task_woken);

    // Periodic status (every 1000 frames after dump)
    if (dump_done && (frame_count % 1000 == 0)) {
        uint16_t *pix = (uint16_t *)trans->buffer;
        ESP_DRAM_LOGI(TAG, "[F#%d] %04X %04X %04X %04X",
            frame_count, pix[0], pix[1000], pix[38400], pix[76799]);
    }

    return high_task_woken == pdTRUE;
}

// ===== MAIN =====
void app_main(void)
{
    ESP_LOGI(TAG, "====== V8: FRAME HEX DUMP ======");
    ESP_LOGI(TAG, "Gain: 8x analog, 1x digital");
    ESP_LOGI(TAG, "After 500 frames: hex dump 320x240 RGB565");

    // LED setup
    gpio_config_t led_conf = { .pin_bit_mask = (1ULL << 15), .mode = GPIO_MODE_OUTPUT };
    gpio_config(&led_conf);
    gpio_set_level(15, 0);

    // Semaphore for dump synchronization
    dump_sem = xSemaphoreCreateBinary();

    // LDO for MIPI CSI
    esp_ldo_channel_handle_t ldo_handle = NULL;
    esp_ldo_channel_config_t ldo_cfg = { .chan_id = 3, .voltage_mv = 2500 };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_cfg, &ldo_handle));
    ESP_LOGI(TAG, "LDO OK");

    // I2C bus
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = 7,
        .scl_io_num = 8,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x1A,
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg, &sensor_dev));
    ESP_LOGI(TAG, "I2C OK");

    // Read chip ID
    vTaskDelay(pdMS_TO_TICKS(100));
    uint8_t id_hi = imx708_read_reg(0x0016);
    uint8_t id_lo = imx708_read_reg(0x0017);
    ESP_LOGI(TAG, "Chip ID: 0x%02X%02X", id_hi, id_lo);
    if (id_hi != 0x07 || id_lo != 0x08) {
        ESP_LOGE(TAG, "NOT IMX708! Abort.");
        return;
    }

    // Software reset
    imx708_write_reg(0x0103, 0x01);
    vTaskDelay(pdMS_TO_TICKS(20));

    // Write register tables
    ESP_LOGI(TAG, "Writing mode_common_regs...");
    imx708_write_table(mode_common_regs);
    ESP_LOGI(TAG, "Writing mode_2x2binned_regs...");
    imx708_write_table(mode_2x2binned_regs);
    ESP_LOGI(TAG, "Writing link_450Mhz_regs...");
    imx708_write_table(link_450Mhz_regs);
    ESP_LOGI(TAG, "Writing override_320x240_raw8...");
    imx708_write_table(override_320x240_raw8);

    // Verify key registers
    uint8_t g_hi = imx708_read_reg(0x0204);
    uint8_t g_lo = imx708_read_reg(0x0205);
    uint8_t e_hi = imx708_read_reg(0x0202);
    uint8_t e_lo = imx708_read_reg(0x0203);
    ESP_LOGI(TAG, "Readback: gain=0x%02X%02X exp=%d", g_hi, g_lo, (e_hi<<8)|e_lo);

    // ISP
    isp_proc_handle_t isp_handle = NULL;
    esp_isp_processor_cfg_t isp_config = {
        .clk_hz = 80000000,
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8,
        .output_data_color_type = ISP_COLOR_RGB565,
        .has_line_start_packet = false,
        .has_line_end_packet = false,
        .h_res = IMG_WIDTH,
        .v_res = IMG_HEIGHT,
    };
    ESP_ERROR_CHECK(esp_isp_new_processor(&isp_config, &isp_handle));
    ESP_ERROR_CHECK(esp_isp_enable(isp_handle));
    ESP_LOGI(TAG, "ISP OK");

    // CSI
    esp_cam_ctlr_csi_config_t csi_config = {
        .ctlr_id = 0,
        .h_res = IMG_WIDTH,
        .v_res = IMG_HEIGHT,
        .lane_bit_rate_mbps = 900,
        .input_data_color_type = CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = CAM_CTLR_COLOR_RGB565,
        .data_lane_num = 2,
        .byte_swap_en = false,
        .queue_items = 1,
        .bk_buffer_dis = true,
    };

    // Allocate frame buffers dynamically BEFORE CSI (needs DMA-capable memory)
    ESP_LOGI(TAG, "Free heap: %lu, DMA: %lu",
        (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DEFAULT),
        (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    frame_buf[0] = heap_caps_aligned_alloc(64, FRAME_SIZE, MALLOC_CAP_INTERNAL);
    frame_buf[1] = heap_caps_aligned_alloc(64, FRAME_SIZE, MALLOC_CAP_INTERNAL);
    if (!frame_buf[0] || !frame_buf[1]) {
        ESP_LOGE(TAG, "Failed to allocate frame buffers! buf0=%p buf1=%p", frame_buf[0], frame_buf[1]);
        return;
    }
    ESP_LOGI(TAG, "Frame buffers allocated: %p, %p (%d bytes each)", frame_buf[0], frame_buf[1], FRAME_SIZE);
    ESP_LOGI(TAG, "Free heap after alloc: %lu", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DEFAULT));

    ESP_ERROR_CHECK(esp_cam_new_csi_ctlr(&csi_config, &cam_handle));
    ESP_LOGI(TAG, "CSI controller created, handle=%p", cam_handle);

    // Create our transaction queue (in IRAM for ISR access)
    trans_queue = xQueueCreateWithCaps(2, sizeof(esp_cam_ctlr_trans_t), MALLOC_CAP_INTERNAL);
    ESP_ERROR_CHECK(trans_queue ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_LOGI(TAG, "Transaction queue created");

    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans = csi_get_new_trans_cb,
        .on_trans_finished = csi_frame_done_cb,
    };
    esp_err_t cb_err = esp_cam_ctlr_register_event_callbacks(cam_handle, &cbs, NULL);
    ESP_LOGI(TAG, "Callback registration result: 0x%x (%s)", cb_err, esp_err_to_name(cb_err));
    ESP_ERROR_CHECK(cb_err);
    ESP_ERROR_CHECK(esp_cam_ctlr_enable(cam_handle));
    ESP_LOGI(TAG, "CSI OK");

    // Start streaming - sensor on
    imx708_write_reg(0x0100, 0x01);
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Sensor streaming ON");

    // Submit initial frame buffer to our queue (start will call on_get_new_trans to retrieve it)
    esp_cam_ctlr_trans_t trans0 = { .buffer = frame_buf[0], .buflen = FRAME_SIZE };
    xQueueSend(trans_queue, &trans0, 0);
    ESP_LOGI(TAG, "Initial transaction queued");
    ESP_ERROR_CHECK(esp_cam_ctlr_start(cam_handle));
    ESP_LOGI(TAG, "Streaming started! Warming up 500 frames...");

    // ===== MAIN LOOP =====
    while (1) {
        // Wait for dump request
        if (xSemaphoreTake(dump_sem, pdMS_TO_TICKS(5000)) == pdTRUE) {
            if (dump_requested && !dump_done) {
                ESP_LOGW(TAG, "Frame #%d selected for hex dump", frame_count);

                // Quick analysis before dump
                uint16_t *pix = (uint16_t *)dump_buf_ptr;
                uint16_t mn = 0xFFFF, mx = 0;
                uint32_t r_sum = 0, g_sum = 0, b_sum = 0;
                for (int i = 0; i < IMG_WIDTH * IMG_HEIGHT; i++) {
                    uint16_t v = pix[i];
                    if (v < mn) mn = v;
                    if (v > mx) mx = v;
                    r_sum += (v >> 11) & 0x1F;
                    g_sum += (v >> 5) & 0x3F;
                    b_sum += v & 0x1F;
                }
                int n = IMG_WIDTH * IMG_HEIGHT;
                ESP_LOGI(TAG, "Pre-dump analysis: min=0x%04X max=0x%04X R=%lu G=%lu B=%lu",
                    mn, mx, r_sum/n, g_sum/n, b_sum/n);

                // LED on during dump
                gpio_set_level(15, 1);

                // HEX DUMP
                dump_frame_hex((const uint8_t *)dump_buf_ptr, IMG_WIDTH, IMG_HEIGHT);

                gpio_set_level(15, 0);
                dump_done = true;
                dump_requested = false;

                ESP_LOGI(TAG, "Hex dump complete! Resuming streaming...");

                // Resume CSI by submitting the buffer back to our queue
                esp_cam_ctlr_trans_t resume_trans = {
                    .buffer = (void *)dump_buf_ptr,
                    .buflen = FRAME_SIZE,
                };
                xQueueSend(trans_queue, &resume_trans, pdMS_TO_TICKS(500));
            }
        }

        // Periodic heartbeat
        if (dump_done && frame_count % 5000 == 0) {
            ESP_LOGI(TAG, "Heartbeat: %d frames captured", frame_count);
        }
    }
}

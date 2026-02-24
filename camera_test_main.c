/*
 * ESP32-P4 v1.0 + Pi Camera 3 (IMX708) CSI Camera Test
 * Based on DFRobot CSI camera example
 * 
 * Tests:
 * 1. IMX708 sensor initialization via I2C
 * 2. CSI controller setup (2-lane MIPI)
 * 3. ISP processor (RAW10 to RGB565 conversion)
 * 4. Frame capture to memory
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_chip_info.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

// Camera driver headers (ESP-IDF v5.4.1)
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "esp_sccb_intf.h"
#include "esp_sccb_i2c.h"
#include "esp_ldo_regulator.h"
#include "esp_cache.h"

// ISP processor
#include "esp_isp.h"
#include "esp_isp_isp.h"

static const char *TAG = "CAM_TEST";

// Pi Camera 3 (IMX708) Configuration
#define IMX708_I2C_ADDR         0x1A
#define CAM_I2C_SCL             8
#define CAM_I2C_SDA             7

// CSI Configuration (2-lane MIPI)
#define CSI_LANE_BITRATE_MBPS   200
#define CSI_DATA_LANES          2

// Initial test resolution: 800x640 @ 50fps (RAW8)
// Later we'll upgrade to 1920x1080 @ 30fps (RAW10)
#define FRAME_WIDTH             800
#define FRAME_HEIGHT            640
#define FRAME_FORMAT            "MIPI_2lane_24Minput_RAW8_800x640_50fps"

// LDO for MIPI PHY
#define MIPI_LDO_CHAN_ID        3
#define MIPI_LDO_VOLTAGE_MV     2500

// Frame buffers
#define NUM_FRAME_BUFFERS       2
static uint8_t *frame_buffers[NUM_FRAME_BUFFERS];
static size_t frame_buffer_size;
static int frame_count = 0;

// Handles
static esp_cam_ctlr_handle_t cam_handle = NULL;
static isp_proc_handle_t isp_proc = NULL;
static esp_ldo_channel_handle_t ldo_mipi_phy = NULL;

/**
 * IMX708 register definitions (basic initialization)
 * These are placeholder values - full IMX708 initialization 
 * requires datasheet which we don't have yet
 */
typedef struct {
    uint16_t reg;
    uint8_t val;
} sensor_reg_t;

// Minimal IMX708 initialization sequence
static const sensor_reg_t imx708_init_regs[] = {
    // Software reset
    {0x0103, 0x01},
    // Mode select: streaming
    {0x0100, 0x01},
};

/**
 * Write to IMX708 register via I2C
 */
static esp_err_t imx708_write_reg(i2c_master_bus_handle_t bus, uint16_t reg, uint8_t val)
{
    uint8_t write_buf[3] = {
        (reg >> 8) & 0xFF,  // Register high byte
        reg & 0xFF,          // Register low byte
        val                  // Value
    };
    
    return i2c_master_transmit(bus, IMX708_I2C_ADDR, write_buf, 3, 1000);
}

/**
 * Initialize IMX708 sensor via I2C
 */
static esp_err_t init_imx708_sensor(void)
{
    ESP_LOGI(TAG, "=== Initializing IMX708 Sensor ===");
    
    // Configure I2C bus
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
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C bus initialized on SCL=%d SDA=%d", CAM_I2C_SCL, CAM_I2C_SDA);
    
    // Verify IMX708 is present
    ret = i2c_master_probe(bus_handle, IMX708_I2C_ADDR, 100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMX708 not found at 0x%02X", IMX708_I2C_ADDR);
        i2c_del_master_bus(bus_handle);
        return ret;
    }
    ESP_LOGI(TAG, "IMX708 detected at I2C address 0x%02X", IMX708_I2C_ADDR);
    
    // Initialize sensor registers
    ESP_LOGI(TAG, "Writing IMX708 initialization registers...");
    for (int i = 0; i < sizeof(imx708_init_regs) / sizeof(sensor_reg_t); i++) {
        ret = imx708_write_reg(bus_handle, imx708_init_regs[i].reg, imx708_init_regs[i].val);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to write reg 0x%04X", imx708_init_regs[i].reg);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGI(TAG, "IMX708 basic initialization complete");
    
    // Keep I2C bus open for now
    // i2c_del_master_bus(bus_handle);
    
    return ESP_OK;
}

/**
 * Camera frame received callback (called from ISR context)
 */
static bool IRAM_ATTR on_camera_frame_received(esp_cam_ctlr_handle_t handle, 
                                                 esp_cam_ctlr_trans_t *trans, 
                                                 void *user_data)
{
    BaseType_t high_task_woken = pdFALSE;
    
    // Count frames
    frame_count++;
    
    // For now, just cycle through buffers
    static int buffer_index = 0;
    trans->buffer = frame_buffers[buffer_index];
    trans->buflen = frame_buffer_size;
    buffer_index = (buffer_index + 1) % NUM_FRAME_BUFFERS;
    
    return high_task_woken == pdTRUE;
}

/**
 * Camera frame finished callback (called from ISR context)
 */
static bool IRAM_ATTR on_camera_frame_finished(esp_cam_ctlr_handle_t handle,
                                                 esp_cam_ctlr_trans_t *trans,
                                                 void *user_data)
{
    // Frame received successfully
    return false;
}

/**
 * Initialize CSI camera controller
 */
static esp_err_t init_csi_camera(void)
{
    ESP_LOGI(TAG, "=== Initializing CSI Camera Controller ===");
    
    // Allocate frame buffers in PSRAM (if available) or internal RAM
    frame_buffer_size = FRAME_WIDTH * FRAME_HEIGHT * 2; // RGB565: 2 bytes per pixel
    
    size_t alignment = 0;
    ESP_ERROR_CHECK(esp_cache_get_alignment(0, &alignment));
    
    ESP_LOGI(TAG, "Allocating %d frame buffers (%zu bytes each)", 
             NUM_FRAME_BUFFERS, frame_buffer_size);
    
    for (int i = 0; i < NUM_FRAME_BUFFERS; i++) {
        frame_buffers[i] = heap_caps_aligned_calloc(alignment, 1, frame_buffer_size,
                                                      MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (frame_buffers[i] == NULL) {
            ESP_LOGE(TAG, "Failed to allocate frame buffer %d", i);
            return ESP_ERR_NO_MEM;
        }
        ESP_LOGI(TAG, "Frame buffer %d allocated at %p", i, frame_buffers[i]);
    }
    
    // Initialize MIPI LDO (power for CSI PHY)
    ESP_LOGI(TAG, "Initializing MIPI LDO (channel %d, %d mV)", 
             MIPI_LDO_CHAN_ID, MIPI_LDO_VOLTAGE_MV);
    
    esp_ldo_channel_config_t ldo_config = {
        .chan_id = MIPI_LDO_CHAN_ID,
        .voltage_mv = MIPI_LDO_VOLTAGE_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_config, &ldo_mipi_phy));
    
    // Configure CSI controller
    ESP_LOGI(TAG, "Configuring CSI controller (%dx%d, %d-lane, %d Mbps)", 
             FRAME_WIDTH, FRAME_HEIGHT, CSI_DATA_LANES, CSI_LANE_BITRATE_MBPS);
    
    esp_cam_ctlr_csi_config_t csi_config = {
        .ctlr_id = 0,
        .h_res = FRAME_WIDTH,
        .v_res = FRAME_HEIGHT,
        .lane_bit_rate_mbps = CSI_LANE_BITRATE_MBPS,
        .input_data_color_type = CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = CAM_CTLR_COLOR_RGB565,
        .data_lane_num = CSI_DATA_LANES,
        .byte_swap_en = false,
        .queue_items = 1,
    };
    
    esp_err_t ret = esp_cam_new_csi_ctlr(&csi_config, &cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create CSI controller: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "CSI controller created");
    
    // Register callbacks
    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans = on_camera_frame_received,
        .on_trans_finished = on_camera_frame_finished,
    };
    ESP_ERROR_CHECK(esp_cam_ctlr_register_event_callbacks(cam_handle, &cbs, NULL));
    ESP_LOGI(TAG, "Camera callbacks registered");
    
    // Enable CSI controller
    ESP_ERROR_CHECK(esp_cam_ctlr_enable(cam_handle));
    ESP_LOGI(TAG, "CSI controller enabled");
    
    return ESP_OK;
}

/**
 * Initialize ISP processor for RAW8 to RGB565 conversion
 */
static esp_err_t init_isp_processor(void)
{
    ESP_LOGI(TAG, "=== Initializing ISP Processor ===");
    
    esp_isp_processor_cfg_t isp_config = {
        .clk_hz = 80 * 1000 * 1000,  // 80 MHz
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8,
        .output_data_color_type = ISP_COLOR_RGB565,
        .has_line_start_packet = true,
        .has_line_end_packet = true,
        .h_res = FRAME_WIDTH,
        .v_res = FRAME_HEIGHT,
    };
    
    esp_err_t ret = esp_isp_new_processor(&isp_config, &isp_proc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ISP processor: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_ERROR_CHECK(esp_isp_enable(isp_proc));
    ESP_LOGI(TAG, "ISP processor enabled (RAW8 -> RGB565)");
    
    return ESP_OK;
}

/**
 * Start camera capture
 */
static esp_err_t start_camera_capture(void)
{
    ESP_LOGI(TAG, "=== Starting Camera Capture ===");
    
    esp_err_t ret = esp_cam_ctlr_start(cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start camera: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Camera capture started!");
    ESP_LOGI(TAG, "Resolution: %dx%d", FRAME_WIDTH, FRAME_HEIGHT);
    ESP_LOGI(TAG, "Format: %s", FRAME_FORMAT);
    
    return ESP_OK;
}

void app_main(void)
{
    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "ESP32-P4 v1.0 + Pi Camera 3 CSI Test");
    ESP_LOGI(TAG, "======================================");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // System info
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "ESP32-P4 revision v%d.%d", chip_info.revision / 100, chip_info.revision % 100);
    ESP_LOGI(TAG, "ESP-IDF version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
    
    // Initialize IMX708 sensor
    ret = init_imx708_sensor();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMX708 initialization failed!");
        return;
    }
    
    // Initialize ISP processor
    ret = init_isp_processor();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ISP initialization failed!");
        return;
    }
    
    // Initialize CSI camera
    ret = init_csi_camera();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CSI camera initialization failed!");
        return;
    }
    
    // Start capture
    ret = start_camera_capture();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start capture!");
        return;
    }
    
    // Monitor frame capture
    int last_count = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        int current_count = frame_count;
        int fps = current_count - last_count;
        last_count = current_count;
        
        ESP_LOGI(TAG, "Frames: %d (FPS: %d), Free heap: %lu bytes", 
                 current_count, fps, esp_get_free_heap_size());
    }
}

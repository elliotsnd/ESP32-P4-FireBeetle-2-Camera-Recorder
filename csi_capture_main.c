/*
 * ESP32-P4 v1.0 + Pi Camera 3 (IMX708) CSI Frame Capture
 * ESP-IDF v5.4.1
 * 
 * Flow: LDO → ISP → CSI → Enable → Start → Receive frames
 * IMX708 initialized via direct I2C register writes
 * ISP converts RAW8 → RGB565
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_chip_info.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"

// Camera CSI driver
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"

// ISP processor
#include "driver/isp.h"

// LDO regulator for MIPI PHY
#include "esp_ldo_regulator.h"

static const char *TAG = "CSI_CAM";

// ============ Hardware Configuration ============
// Pi Camera 3 (IMX708) I2C
#define IMX708_I2C_ADDR         0x1A
#define CAM_I2C_SCL             8
#define CAM_I2C_SDA             7
#define I2C_FREQ_HZ             100000

// LED for status
#define LED_PIN                 15

// MIPI CSI Configuration
#define CSI_LANE_BITRATE_MBPS   200
#define CSI_DATA_LANES          2

// Frame dimensions - very small to fit in internal RAM (~598KB heap)
#define FRAME_WIDTH             320
#define FRAME_HEIGHT            240
#define BYTES_PER_PIXEL_OUT     2  // RGB565

// LDO for MIPI PHY power
#define MIPI_LDO_CHAN           3
#define MIPI_LDO_VOLTAGE_MV     2500

// Frame buffer
#define FRAME_BUF_SIZE          (FRAME_WIDTH * FRAME_HEIGHT * BYTES_PER_PIXEL_OUT)

// ============ Global State ============
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t imx708_handle = NULL;
static esp_cam_ctlr_handle_t cam_handle = NULL;
static isp_proc_handle_t isp_proc = NULL;
static uint8_t *frame_buffer = NULL;
static volatile uint32_t frame_count = 0;
static volatile uint32_t last_frame_size = 0;

// ============ IMX708 Register Definitions ============
// From IMX708 datasheet
#define IMX708_REG_CHIP_ID_H     0x0016
#define IMX708_REG_CHIP_ID_L     0x0017
#define IMX708_REG_MODE_SELECT   0x0100  // 0=standby, 1=streaming
#define IMX708_REG_SW_RESET      0x0103  // Software reset

// Mode 0: 800x640 RAW8 - analog crop from full sensor
// These are minimal registers to get streaming started
// Full init tables would come from libcamera/Pi Camera driver
#define IMX708_REG_X_OUTPUT_SIZE_H  0x034C
#define IMX708_REG_X_OUTPUT_SIZE_L  0x034D
#define IMX708_REG_Y_OUTPUT_SIZE_H  0x034E
#define IMX708_REG_Y_OUTPUT_SIZE_L  0x034F
#define IMX708_REG_CSI_DT_FMT_H    0x0112  // CSI data format
#define IMX708_REG_CSI_DT_FMT_L    0x0113
#define IMX708_REG_CSI_LANE_MODE    0x0114  // 1=2-lane, 3=4-lane
#define IMX708_REG_FRM_LENGTH_H     0x0340
#define IMX708_REG_FRM_LENGTH_L     0x0341
#define IMX708_REG_LINE_LENGTH_H    0x0342
#define IMX708_REG_LINE_LENGTH_L    0x0343
#define IMX708_REG_COARSE_INT_H     0x0202  // Exposure
#define IMX708_REG_COARSE_INT_L     0x0203
#define IMX708_REG_ANA_GAIN_H       0x0204  // Analog gain
#define IMX708_REG_ANA_GAIN_L       0x0205

// ============ I2C Functions ============

static esp_err_t imx708_write_reg(uint16_t reg, uint8_t val)
{
    uint8_t buf[3] = { (reg >> 8) & 0xFF, reg & 0xFF, val };
    return i2c_master_transmit(imx708_handle, buf, 3, 1000 / portTICK_PERIOD_MS);
}

static esp_err_t imx708_read_reg(uint16_t reg, uint8_t *val)
{
    uint8_t addr[2] = { (reg >> 8) & 0xFF, reg & 0xFF };
    return i2c_master_transmit_receive(imx708_handle, addr, 2, val, 1, 1000 / portTICK_PERIOD_MS);
}

static esp_err_t imx708_write_reg16(uint16_t reg, uint16_t val)
{
    esp_err_t ret = imx708_write_reg(reg, (val >> 8) & 0xFF);
    if (ret == ESP_OK) {
        ret = imx708_write_reg(reg + 1, val & 0xFF);
    }
    return ret;
}

// ============ IMX708 Init ============

static esp_err_t init_i2c(void)
{
    ESP_LOGI(TAG, "I2C init (SCL=%d, SDA=%d, addr=0x%02X)", CAM_I2C_SCL, CAM_I2C_SDA, IMX708_I2C_ADDR);

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CAM_I2C_SDA,
        .scl_io_num = CAM_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMX708_I2C_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &imx708_handle));

    // Read chip ID
    uint8_t id_h = 0, id_l = 0;
    ESP_ERROR_CHECK(imx708_read_reg(IMX708_REG_CHIP_ID_H, &id_h));
    ESP_ERROR_CHECK(imx708_read_reg(IMX708_REG_CHIP_ID_L, &id_l));
    ESP_LOGI(TAG, "IMX708 Chip ID: 0x%02X%02X", id_h, id_l);

    return ESP_OK;
}

static esp_err_t init_imx708_streaming(void)
{
    ESP_LOGI(TAG, "Configuring IMX708 for %dx%d RAW8...", FRAME_WIDTH, FRAME_HEIGHT);

    // Software reset
    ESP_ERROR_CHECK(imx708_write_reg(IMX708_REG_SW_RESET, 0x01));
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // Stop streaming while configuring
    ESP_ERROR_CHECK(imx708_write_reg(IMX708_REG_MODE_SELECT, 0x00));
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // CSI-2 lane mode: 2-lane
    ESP_ERROR_CHECK(imx708_write_reg(IMX708_REG_CSI_LANE_MODE, 0x01));  // 01 = 2-lane

    // CSI data format: RAW8 (0x0808)
    ESP_ERROR_CHECK(imx708_write_reg(IMX708_REG_CSI_DT_FMT_H, 0x08));
    ESP_ERROR_CHECK(imx708_write_reg(IMX708_REG_CSI_DT_FMT_L, 0x08));

    // Output size
    ESP_ERROR_CHECK(imx708_write_reg16(IMX708_REG_X_OUTPUT_SIZE_H, FRAME_WIDTH));
    ESP_ERROR_CHECK(imx708_write_reg16(IMX708_REG_Y_OUTPUT_SIZE_H, FRAME_HEIGHT));

    // Frame/line length (generous timing)
    ESP_ERROR_CHECK(imx708_write_reg16(IMX708_REG_LINE_LENGTH_H, 0x1000));  // 4096
    ESP_ERROR_CHECK(imx708_write_reg16(IMX708_REG_FRM_LENGTH_H, 0x0300));   // 768

    // Exposure: moderate
    ESP_ERROR_CHECK(imx708_write_reg16(IMX708_REG_COARSE_INT_H, 0x0100));

    // Analog gain: low gain
    ESP_ERROR_CHECK(imx708_write_reg16(IMX708_REG_ANA_GAIN_H, 0x0100));

    ESP_LOGI(TAG, "IMX708 registers configured");

    // Start streaming
    ESP_ERROR_CHECK(imx708_write_reg(IMX708_REG_MODE_SELECT, 0x01));
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "IMX708 streaming started");
    return ESP_OK;
}

// ============ CSI Callbacks ============

static bool IRAM_ATTR on_get_new_trans(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    // Provide the same buffer for next frame
    esp_cam_ctlr_trans_t *new_trans = (esp_cam_ctlr_trans_t *)user_data;
    trans->buffer = new_trans->buffer;
    trans->buflen = new_trans->buflen;
    return false;
}

static bool IRAM_ATTR on_trans_finished(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    frame_count++;
    last_frame_size = trans->received_size;
    return false;
}

// ============ LED ============

static void init_led(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&cfg);
    gpio_set_level(LED_PIN, 0);
}

// ============ Main ============

void app_main(void)
{
    esp_err_t ret;

    // NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_led();

    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "  ESP32-P4 CSI Camera Capture");
    ESP_LOGI(TAG, "  IMX708 %dx%d RAW8 -> RGB565", FRAME_WIDTH, FRAME_HEIGHT);
    ESP_LOGI(TAG, "====================================");

    // Print memory info
    ESP_LOGI(TAG, "Free heap: %ld bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "Frame buffer needed: %d bytes", FRAME_BUF_SIZE);

    // --- Step 1: MIPI LDO ---
    ESP_LOGI(TAG, "[1/6] MIPI LDO init (chan=%d, %dmV)...", MIPI_LDO_CHAN, MIPI_LDO_VOLTAGE_MV);
    esp_ldo_channel_handle_t ldo_mipi = NULL;
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = MIPI_LDO_CHAN,
        .voltage_mv = MIPI_LDO_VOLTAGE_MV,
    };
    ret = esp_ldo_acquire_channel(&ldo_cfg, &ldo_mipi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LDO init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "  LDO OK");

    // --- Step 2: I2C + IMX708 ---
    ESP_LOGI(TAG, "[2/6] IMX708 I2C init...");
    ret = init_i2c();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed!");
        return;
    }
    ESP_LOGI(TAG, "  I2C OK");

    // --- Step 3: Allocate frame buffer ---
    ESP_LOGI(TAG, "[3/6] Allocating frame buffer (%d bytes)...", FRAME_BUF_SIZE);
    // Try DMA-capable internal memory first (no PSRAM unless enabled in sdkconfig)
    frame_buffer = heap_caps_aligned_calloc(64, 1, FRAME_BUF_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!frame_buffer) {
        ESP_LOGE(TAG, "  Frame buffer alloc FAILED! Need %d bytes, have %ld",
                 FRAME_BUF_SIZE, esp_get_free_heap_size());
        return;
    }
    ESP_LOGI(TAG, "  Buffer at %p, remaining heap: %ld", frame_buffer, esp_get_free_heap_size());

    // Transaction for frame receive
    esp_cam_ctlr_trans_t cam_trans = {
        .buffer = frame_buffer,
        .buflen = FRAME_BUF_SIZE,
    };

    // --- Step 4: CSI controller ---
    ESP_LOGI(TAG, "[4/6] CSI controller init (%d-lane, %d Mbps)...", CSI_DATA_LANES, CSI_LANE_BITRATE_MBPS);
    esp_cam_ctlr_csi_config_t csi_cfg = {
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
    ret = esp_cam_new_csi_ctlr(&csi_cfg, &cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "  CSI init FAILED: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "  CSI controller created");

    // Register callbacks
    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans = on_get_new_trans,
        .on_trans_finished = on_trans_finished,
    };
    ESP_ERROR_CHECK(esp_cam_ctlr_register_event_callbacks(cam_handle, &cbs, &cam_trans));
    ESP_LOGI(TAG, "  Callbacks registered");

    // Enable CSI (before ISP, per example)
    ESP_ERROR_CHECK(esp_cam_ctlr_enable(cam_handle));
    ESP_LOGI(TAG, "  CSI enabled");

    // --- Step 5: ISP processor ---
    ESP_LOGI(TAG, "[5/6] ISP processor init (RAW8 -> RGB565)...");
    esp_isp_processor_cfg_t isp_cfg = {
        .clk_hz = 80 * 1000 * 1000,
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8,
        .output_data_color_type = ISP_COLOR_RGB565,
        .has_line_start_packet = false,
        .has_line_end_packet = false,
        .h_res = FRAME_WIDTH,
        .v_res = FRAME_HEIGHT,
    };
    ret = esp_isp_new_processor(&isp_cfg, &isp_proc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "  ISP init FAILED: %s", esp_err_to_name(ret));
        return;
    }
    ESP_ERROR_CHECK(esp_isp_enable(isp_proc));
    ESP_LOGI(TAG, "  ISP enabled");

    // --- Step 6: Configure IMX708 and start streaming ---
    ESP_LOGI(TAG, "[6/6] Starting IMX708 streaming...");
    ret = init_imx708_streaming();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "  IMX708 streaming start FAILED!");
        return;
    }

    // Start CSI reception
    ESP_ERROR_CHECK(esp_cam_ctlr_start(cam_handle));
    ESP_LOGI(TAG, "  CSI started - receiving frames!");

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== CAPTURE RUNNING ===");
    ESP_LOGI(TAG, "Monitoring frame rate...");
    ESP_LOGI(TAG, "");

    // LED on = running
    gpio_set_level(LED_PIN, 1);

    // Monitor loop - prints every second so serial capture can catch it
    uint32_t prev_count = 0;
    int loop_num = 0;
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        loop_num++;

        uint32_t cur = frame_count;
        uint32_t delta = cur - prev_count;
        prev_count = cur;

        ESP_LOGI(TAG, "[%d] Frames:%lu FPS:%lu Size:%lu Heap:%ld",
                 loop_num, (unsigned long)cur, (unsigned long)delta,
                 (unsigned long)last_frame_size, esp_get_free_heap_size());

        // Toggle LED on each report
        static int led_state = 1;
        led_state = !led_state;
        gpio_set_level(LED_PIN, led_state);

        // If no frames after 10 seconds, report issue
        if (cur == 0 && prev_count == 0) {
            ESP_LOGW(TAG, "No frames received yet - check CSI connection");

            // Dump some sensor registers for debug
            uint8_t mode = 0, lane = 0, fmt_h = 0, fmt_l = 0;
            imx708_read_reg(IMX708_REG_MODE_SELECT, &mode);
            imx708_read_reg(IMX708_REG_CSI_LANE_MODE, &lane);
            imx708_read_reg(IMX708_REG_CSI_DT_FMT_H, &fmt_h);
            imx708_read_reg(IMX708_REG_CSI_DT_FMT_L, &fmt_l);
            ESP_LOGI(TAG, "  Sensor: mode=0x%02X lane=0x%02X fmt=0x%02X%02X",
                     mode, lane, fmt_h, fmt_l);
        }
    }
}

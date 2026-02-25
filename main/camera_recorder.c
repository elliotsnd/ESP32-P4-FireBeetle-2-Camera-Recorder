/*
 * ESP32-P4 Camera Recorder
 *
 * Follows esp-video-components repo patterns:
 *   - esp_video_init() for camera/ISP initialization
 *   - V4L2 M2M H.264 device for hardware encoding
 *   - V4L2 capture for frame acquisition
 *
 * Pipeline: Capture+Encode (Core 0) -> Queue -> SD Write (Core 1)
 *
 * Hardware: FireBeetle 2 ESP32-P4 + RPi Camera 3 (IMX708)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <inttypes.h>
#include <math.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_ldo_regulator.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

#include "driver/i2s_pdm.h"
#include "driver/ppa.h"

#include "linux/videodev2.h"
#include "esp_video_init.h"
#include "esp_video_device.h"
#include "esp_video_isp_ioctl.h"
#include "esp_video_ioctl.h"

/* CSI/ISP/DMA hardware register access for diagnostics */
#include "soc/mipi_csi_bridge_struct.h"
#include "soc/mipi_csi_host_struct.h"
#include "soc/isp_struct.h"
#include "soc/dw_gdma_struct.h"
#include "soc/hp_sys_clkrst_struct.h"

/* SDMMC DMA tuning registers */
#include "soc/sdmmc_reg.h"
#include "soc/hp_system_reg.h"

static const char *TAG = "CAM_REC";

/* ISR diagnostic counters defined in esp_video_csi_device.c */
extern volatile uint32_t csi_isr_trans_finished_cnt;
extern volatile uint32_t csi_isr_get_new_trans_cnt;
extern volatile uint32_t csi_isr_frame_dropped_cnt;
extern volatile uint32_t csi_isr_queue_empty_cnt;

/* Forward-declare for use in dump_csi_diagnostics (defined later) */
static i2c_master_bus_handle_t i2c_bus_handle;

static void dump_csi_diagnostics(void)
{
    ESP_LOGW(TAG, "=== CSI DIAGNOSTIC DUMP ===");

    // CSI Bridge errors
    uint32_t brg_raw = MIPI_CSI_BRIDGE.int_raw.val;
    ESP_LOGW(TAG, "BRG int_raw=0x%08lx [vadr_gt=%d vadr_lt=%d discard=%d overrun=%d fifo_ovf=%d dma_upd=%d]",
             (unsigned long)brg_raw,
             (int)MIPI_CSI_BRIDGE.int_raw.vadr_num_gt_int_raw,
             (int)MIPI_CSI_BRIDGE.int_raw.vadr_num_lt_int_raw,
             (int)MIPI_CSI_BRIDGE.int_raw.discard_int_raw,
             (int)MIPI_CSI_BRIDGE.int_raw.csi_buf_overrun_int_raw,
             (int)MIPI_CSI_BRIDGE.int_raw.csi_async_fifo_ovf_int_raw,
             (int)MIPI_CSI_BRIDGE.int_raw.dma_cfg_has_updated_int_raw);
    // Bridge buffer depth (current fill level)
    ESP_LOGW(TAG, "BRG buf_depth=%lu afull_thrd=%lu",
             (unsigned long)MIPI_CSI_BRIDGE.buf_flow_ctl.csi_buf_depth,
             (unsigned long)MIPI_CSI_BRIDGE.buf_flow_ctl.csi_buf_afull_thrd);

    // CSI Host PHY state
    ESP_LOGW(TAG, "HOST phy_rx=0x%08lx [clk_hs_active=%d ulp_esc0=%d ulp_esc1=%d ulp_clk_not=%d]",
             (unsigned long)MIPI_CSI_HOST.phy_rx.val,
             (int)MIPI_CSI_HOST.phy_rx.phy_rxclkactivehs,
             (int)MIPI_CSI_HOST.phy_rx.phy_rxulpsesc_0,
             (int)MIPI_CSI_HOST.phy_rx.phy_rxulpsesc_1,
             (int)MIPI_CSI_HOST.phy_rx.phy_rxulpsclknot);
    ESP_LOGW(TAG, "HOST phy_stop=0x%08lx [data0=%d data1=%d clk=%d]",
             (unsigned long)MIPI_CSI_HOST.phy_stopstate.val,
             (int)MIPI_CSI_HOST.phy_stopstate.phy_stopstatedata_0,
             (int)MIPI_CSI_HOST.phy_stopstate.phy_stopstatedata_1,
             (int)MIPI_CSI_HOST.phy_stopstate.phy_stopstateclk);

    // CSI Host error summary (captured once to avoid read-clear issues)
    uint32_t main_int = MIPI_CSI_HOST.int_st_main.val;
    ESP_LOGW(TAG, "HOST int_st_main=0x%08lx [phy_fatal=%d pkt_fatal=%d bndry=%d seq=%d crc=%d pld_crc=%d data_id=%d ecc=%d phy=%d]",
             (unsigned long)main_int,
             (int)((main_int >> 0) & 1),
             (int)((main_int >> 1) & 1),
             (int)((main_int >> 2) & 1),
             (int)((main_int >> 3) & 1),
             (int)((main_int >> 4) & 1),
             (int)((main_int >> 5) & 1),
             (int)((main_int >> 6) & 1),
             (int)((main_int >> 7) & 1),
             (int)((main_int >> 8) & 1));
    // Always dump PHY sub-interrupt registers (they're also read-clear)
    uint32_t phy_fatal = MIPI_CSI_HOST.int_st_phy_fatal.val;
    uint32_t pkt_fatal = MIPI_CSI_HOST.int_st_pkt_fatal.val;
    uint32_t phy_nonfatal = MIPI_CSI_HOST.int_st_phy.val;
    ESP_LOGW(TAG, "  phy_fatal=0x%08lx pkt_fatal=0x%08lx phy=0x%08lx",
             (unsigned long)phy_fatal, (unsigned long)pkt_fatal,
             (unsigned long)phy_nonfatal);
    if (phy_fatal) {
        ESP_LOGW(TAG, "  phy_fatal bits: dl0_errcontrol=%d dl1_errcontrol=%d erresc_dl0=%d erresc_dl1=%d errsyncesc_dl0=%d errsyncesc_dl1=%d",
                 (int)((phy_fatal >> 0) & 1), (int)((phy_fatal >> 1) & 1),
                 (int)((phy_fatal >> 8) & 1), (int)((phy_fatal >> 9) & 1),
                 (int)((phy_fatal >> 16) & 1), (int)((phy_fatal >> 17) & 1));
    }
    if (phy_nonfatal) {
        ESP_LOGW(TAG, "  phy bits: errsoths_dl0=%d errsoths_dl1=%d erresc_dl0=%d erresc_dl1=%d",
                 (int)((phy_nonfatal >> 0) & 1), (int)((phy_nonfatal >> 1) & 1),
                 (int)((phy_nonfatal >> 16) & 1), (int)((phy_nonfatal >> 17) & 1));
    }

    // D-PHY test register scan — read key registers via test interface
    // Protocol: set testen=1 + testdin=addr → pulse testclk → read testdout
    ESP_LOGW(TAG, "DPHY test regs:");
    for (uint8_t addr = 0; addr <= 0x50; addr++) {
        // Select register address
        MIPI_CSI_HOST.phy_test_ctrl0.val = 0x00;               // testclk=0, testclr=0
        MIPI_CSI_HOST.phy_test_ctrl1.val = (1 << 16) | addr;   // testen=1, testdin=addr
        MIPI_CSI_HOST.phy_test_ctrl0.val = 0x02;               // testclk=1
        MIPI_CSI_HOST.phy_test_ctrl0.val = 0x00;               // testclk=0
        // Read testdout [15:8]
        uint8_t val = (MIPI_CSI_HOST.phy_test_ctrl1.val >> 8) & 0xFF;
        // Deselect test mode
        MIPI_CSI_HOST.phy_test_ctrl1.val = 0x00;               // testen=0
        if (val != 0) {
            ESP_LOGW(TAG, "  reg[0x%02x]=0x%02x", addr, val);
        }
    }

    // ISP errors + frame status
    uint32_t isp_raw = ISP.int_raw.val;
    ESP_LOGW(TAG, "ISP int_raw=0x%08lx [type_err=%d fifo_ovf=%d buf_full=%d hvnum_err=%d type_set_err=%d hnum_unmatch=%d frame_done=%d hdr_idi_frame=%d]",
             (unsigned long)isp_raw,
             (int)((isp_raw >> 0) & 1),
             (int)((isp_raw >> 1) & 1),
             (int)((isp_raw >> 2) & 1),
             (int)((isp_raw >> 3) & 1),
             (int)((isp_raw >> 4) & 1),
             (int)((isp_raw >> 5) & 1),
             (int)((isp_raw >> 14) & 1),   // frame_int_raw
             (int)((isp_raw >> 28) & 1));  // header_idi_frame_int_raw

    // ISP frame dimensions and clock diagnostics
    ESP_LOGW(TAG, "ISP frame_cfg: hadr_num=%lu vadr_num=%lu",
             (unsigned long)ISP.frame_cfg.hadr_num,
             (unsigned long)ISP.frame_cfg.vadr_num);
    uint32_t isp_clk_div = HP_SYS_CLKRST.peri_clk_ctrl26.reg_isp_clk_div_num;
    ESP_LOGW(TAG, "ISP clk_div_num=%lu (actual ISP clk ~%lu MHz)",
             (unsigned long)isp_clk_div, (unsigned long)(160 / (isp_clk_div + 1)));

    // DW GDMA channel status - scan all 4 channels to find CSI DMA
    uint32_t chen = DW_GDMA.chen0.val;
    ESP_LOGW(TAG, "DMA chen0=0x%08lx [ch1_en=%d ch2_en=%d ch3_en=%d ch4_en=%d]",
             (unsigned long)chen,
             (int)(chen & 1), (int)((chen >> 1) & 1),
             (int)((chen >> 2) & 1), (int)((chen >> 3) & 1));
    for (int i = 0; i < 4; i++) {
        uint32_t ch_int = DW_GDMA.ch[i].int_st0.val;
        uint32_t ch_int_ena = DW_GDMA.ch[i].int_st_ena0.val;
        if (ch_int || (chen & (1 << i))) {
            uint32_t blk_done = DW_GDMA.ch[i].status0.cmpltd_blk_tfr_size;
            uint32_t fifo_left = DW_GDMA.ch[i].status1.data_left_in_fifo;
            ESP_LOGW(TAG, "DMA ch%d: int_st=0x%08lx int_ena=0x%08lx blk_done=%lu fifo_left=%lu en=%d",
                     i + 1, (unsigned long)ch_int, (unsigned long)ch_int_ena,
                     (unsigned long)blk_done, (unsigned long)fifo_left,
                     (int)((chen >> i) & 1));
            // Decode error bits if any
            if (ch_int & 0x3FE0) { // bits 5-13: various errors
                ESP_LOGE(TAG, "DMA ch%d ERRORS: src_dec=%d dst_dec=%d src_slv=%d dst_slv=%d shadow_invalid=%d",
                         i + 1,
                         (int)((ch_int >> 5) & 1), (int)((ch_int >> 6) & 1),
                         (int)((ch_int >> 7) & 1), (int)((ch_int >> 8) & 1),
                         (int)((ch_int >> 13) & 1));
            }
        }
    }

    // ISR counters
    ESP_LOGW(TAG, "ISR: finished=%lu get_new=%lu dropped=%lu q_empty=%lu",
             (unsigned long)csi_isr_trans_finished_cnt,
             (unsigned long)csi_isr_get_new_trans_cnt,
             (unsigned long)csi_isr_frame_dropped_cnt,
             (unsigned long)csi_isr_queue_empty_cnt);

    // Read IMX708 stream register 0x0100
    i2c_device_config_t cam_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = 0x1A,
        .scl_speed_hz    = 100000,
    };
    i2c_master_dev_handle_t cam_dev = NULL;
    if (i2c_master_bus_add_device(i2c_bus_handle, &cam_cfg, &cam_dev) == ESP_OK) {
        uint8_t reg_addr[] = {0x01, 0x00};
        uint8_t reg_val = 0xFF;
        if (i2c_master_transmit_receive(cam_dev, reg_addr, 2, &reg_val, 1, 100) == ESP_OK) {
            ESP_LOGW(TAG, "IMX708 reg 0x0100 (stream) = 0x%02x %s",
                     reg_val, reg_val ? "(STREAMING)" : "(STOPPED!)");
        } else {
            ESP_LOGE(TAG, "IMX708 reg 0x0100 read FAILED (I2C error)");
        }
        i2c_master_bus_rm_device(cam_dev);
    }
    ESP_LOGW(TAG, "=== END DIAGNOSTIC DUMP ===");
}

// ============================================================================
// DIAGNOSTIC ISOLATION TOGGLES — flip to 1 to test individually
// ============================================================================
#define DIAG_USE_720P       0   // 1 = override resolution to 720p (less memory, less MIPI bandwidth)
#define DIAG_LOW_BITRATE    0   // 1 = drop H.264 bitrate to 6 Mbps
#define DIAG_NO_AUDIO       0   // 1 = skip PDM mic init and audio capture entirely
#define DIAG_USE_1080P15    1   // 1 = 1280×720@30fps (56% FOV, high quality)  0 = 1536×864@30fps (67% FOV)

// ============================================================================
// CONFIGURATION
// ============================================================================

#if DIAG_LOW_BITRATE
#define H264_BITRATE        4000000   // 4 Mbps (reduced for diagnostics)
#else
#define H264_BITRATE        8000000   // 8 Mbps — NOTE: esp_video bug sets encoder fps=GOP, so
#endif                                //   bits_per_frame = bitrate/GOP = 8M/30 = 33KB target
#define H264_GOP            15        // GOP=15. Halved from 30 to limit P-frame corruption duration.
                                      //   esp_video H264 driver uses GOP as encoder FPS for RC budget.
                                      //   I-frame every 0.5s means artifacts heal in ≤0.5s.
#define H264_MIN_QP         22        // Min QP — floor quality (lower=better, encoder starts at (min+max)/2=31)
#define H264_MAX_QP         40        // Max QP — wide range lets RC converge without uint32_t overflow
#define RECORD_DURATION_SEC 300       // 5 minutes
#define AVI_FPS             30        // Max encode rate (actual FPS patched into AVI header at finalization)

#define NUM_CAP_BUFFERS     6       // DMA pipeline needs 3+ (writing/ISP/encode), 6 gives headroom
#define NUM_M2M_CAP_BUFS    1       // V4L2 M2M capture buffer (single; driver limitation)
#define NUM_JPEG_BUFS       60      // Rotating PSRAM staging buffers - 2s buffer at 30fps absorbs SD stalls
#if DIAG_USE_1080P15
#define JPEG_BUF_SIZE       (64 * 1024)   // 64KB per buffer (H.264 avg ~33KB, I-frames ~50-60KB)
#else
#define JPEG_BUF_SIZE       (64 * 1024)   // 64KB per buffer (H.264 frames ~33KB avg)
#endif
#define SKIP_STARTUP_FRAMES 30      // Skip startup frames for AEC/AGC settling (~1s at 30fps)

// Audio (onboard PDM mic)
#define AUDIO_SAMPLE_RATE   16000   // 16kHz PCM
#define AUDIO_BITS          16
#define AUDIO_CHANNELS      1
#define PDM_CLK_PIN         12      // FireBeetle 2 onboard mic CLK
#define PDM_DATA_PIN        9       // FireBeetle 2 onboard mic DATA

// SD Card pins (FireBeetle 2)
#define SD_CMD_PIN          44
#define SD_CLK_PIN          43
#define SD_D0_PIN           39
#define SD_D1_PIN           40
#define SD_D2_PIN           41
#define SD_D3_PIN           42

// Camera I2C
#define CAM_I2C_PORT        I2C_NUM_0
#define CAM_I2C_SDA         7
#define CAM_I2C_SCL         8
#define CAM_I2C_FREQ        400000

// Camera XCLK - NOT NEEDED
// RPi Camera Module 3 has an onboard 24MHz oscillator.
// FireBeetle 2's 15-pin FPC connector has NO XCLK pin.

#define LED_PIN             3

// ============================================================================
// TYPES
// ============================================================================

typedef struct {
    // Camera capture
    int cap_fd;
    uint32_t width;           // sensor output width (e.g. 2304)
    uint32_t height;          // sensor output height (e.g. 1296)
    uint32_t enc_width;       // encoder input width (e.g. 1280 after PPA scale)
    uint32_t enc_height;      // encoder input height (e.g. 720 after PPA scale)
    uint32_t capture_fmt;     // pixel format for raw capture
    uint32_t cap_buf_len[NUM_CAP_BUFFERS]; // buffer lengths for munmap
    uint8_t *cap_buffer[NUM_CAP_BUFFERS];

    // H.264 M2M encoder
    int m2m_fd;
    uint8_t *m2m_cap_buffers[NUM_M2M_CAP_BUFS]; // mmap'd H.264 output buffers
    uint32_t m2m_cap_buf_lens[NUM_M2M_CAP_BUFS];

    // Async staging buffers: memcpy from M2M → staging, then write task reads staging
    uint8_t *jpeg_out_buf[NUM_JPEG_BUFS];
    int jpeg_buf_write_idx;  // next buffer for capture task to write into
} recorder_t;

// Queued from capture task -> write task
typedef struct {
    uint8_t *jpeg_data;   // encoded frame data (points to staging buffer)
    uint32_t jpeg_size;
    int seq;
    bool is_keyframe;
} frame_msg_t;

// ============================================================================
// GLOBALS
// ============================================================================

static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static esp_ldo_channel_handle_t ldo_mipi_handle = NULL;
static esp_ldo_channel_handle_t ldo_vdd_io5_handle = NULL;
static sdmmc_card_t *sd_card = NULL;
static recorder_t *recorder = NULL;

// Pipeline: memcpy to staging buffers, async write from staging
static QueueHandle_t write_queue = NULL;
static SemaphoreHandle_t jpeg_buf_sem = NULL;      // counts available staging buffers
static SemaphoreHandle_t write_task_ready = NULL;  // signals write task is initialized

static volatile bool recording = false;
static volatile bool capture_running = false;
static volatile int total_captured = 0;
static volatile int total_written = 0;
static volatile int total_dropped = 0;
static bool sd_card_available = false;

// Task handles for stack watermark monitoring
static TaskHandle_t sd_write_task_handle = NULL;
static TaskHandle_t capture_task_handle = NULL;
static TaskHandle_t audio_task_handle = NULL;

// Audio globals
static i2s_chan_handle_t i2s_rx_handle = NULL;
#define AUDIO_DMA_BUF_SIZE  1024     // DMA buffer size in samples
#define AUDIO_DMA_BUF_COUNT 4        // number of DMA buffers
// Audio ring buffer: holds PCM samples between audio_capture_task and sd_write_task
#define AUDIO_RING_SIZE     (AUDIO_SAMPLE_RATE * 2 * 2)  // 2 seconds of 16-bit mono
static uint8_t *audio_ring = NULL;
static volatile uint32_t audio_ring_wr = 0;   // write position (audio task)
static volatile uint32_t audio_ring_rd = 0;   // read position (write task)
static volatile bool audio_running = false;

// PPA hardware scaler (1920x1080 → 1280x720)
static ppa_client_handle_t ppa_srm_handle = NULL;
static uint8_t *ppa_scaled_buf = NULL;
static uint32_t ppa_scaled_buf_size = 0;

// ============================================================================
// CUSTOM WIDE FOV SENSOR MODE
// Strategy: Use the EXACT working 720p register set but widen the digital crop.
// This keeps all proven-working PLL, binning, and scaling registers.
// The crop is genuine (offset from 2304x1296 binned) so 0x3200=0x43 stays valid.
//
// 180° rotation via reg 0x0101 (IMAGE_ORIENTATION): bit0=H_MIRROR, bit1=V_FLIP.
// Confirmed from: imx708_regs.h (#define IMX708_REG_ORIENTATION 0x0101),
//   imx708.c (imx708_set_hmirror/vflip both use 0x0101),
//   RPi ref driver (#define IMX708_REG_ORIENTATION 0x101).
// Bayer order changes with rotation: RGGB → BGGR (per RPi driver codes[] table).
//
// Two modes available via DIAG_USE_1080P15 toggle:
//   0 (default): 1536×864 @ 30fps — 67% sensor FOV, 1.9 MB/frame, proven stable
//   1:           1280×720 @ 30fps — 56% sensor FOV, ~1.4 MB/frame, high quality motion
//                Lower pixel count = more bits per pixel at same bitrate = no block artifacts.
// ============================================================================
// DIAG_USE_1080P15 defined above (before JPEG_BUF_SIZE which depends on it)

typedef struct { uint16_t reg; uint8_t val; } custom_reginfo_t;

// --- Shared register block: PLL, binning, analog readout, exposure (identical to 720p) ---
// Only FRM_LENGTH, digital crop, output size, and orientation differ between modes.

#if DIAG_USE_1080P15
// 1280×720 @ 30fps: 56% sensor FOV, ~1.4 MB/frame
// Scale-only mode (0x41) — proven stable on ESP32-P4 CSI bridge.
// Digital crop: center 1280×720 in 2304×1296 binned (offset 512,288)
static const custom_reginfo_t custom_imx708_regs[] = {
    // --- Timing ---
    {0x0342, 0x31},  // LINE_LENGTH_PCK (12740)
    {0x0343, 0xC4},
    {0x0340, 0x06},  // FRM_LENGTH_LINES (1558 = 0x0616) — 30fps
    {0x0341, 0x16},  //   was 3500 → only 13fps! Min VTS=1336 (height+40)
    // --- Analog readout: full sensor ---
    {0x0344, 0x00}, {0x0345, 0x00},  // X_ADDR_START (0)
    {0x0346, 0x00}, {0x0347, 0x00},  // Y_ADDR_START (0)
    {0x0348, 0x11}, {0x0349, 0xFF},  // X_ADDR_END (4607)
    {0x034A, 0x0A}, {0x034B, 0x1F},  // Y_ADDR_END (2591)
    // --- Exposure/misc ---
    {0x0220, 0x62}, {0x0222, 0x01},
    // --- Binning: 2×2, averaged weighting ---
    {0x0900, 0x01}, {0x0901, 0x22}, {0x0902, 0x08},
    // --- Scaling: scale-only mode (crop mode 0x43 stalls on ESP32-P4) ---
    {0x3200, 0x41}, {0x3201, 0x41}, {0x32D5, 0x00}, {0x32D6, 0x00},
    {0x32DB, 0x01}, {0x32DF, 0x00}, {0x350C, 0x00}, {0x350D, 0x00},
    // --- Digital crop: 1280×720 centered in 2304×1296 ---
    {0x0408, 0x02},  // DIG_CROP_X_OFFSET = (2304-1280)/2 = 512 = 0x0200
    {0x0409, 0x00},
    {0x040A, 0x01},  // DIG_CROP_Y_OFFSET = (1296-720)/2 = 288 = 0x0120
    {0x040B, 0x20},
    {0x040C, 0x05},  // DIG_CROP_IMAGE_WIDTH = 1280 = 0x0500
    {0x040D, 0x00},
    {0x040E, 0x02},  // DIG_CROP_IMAGE_HEIGHT = 720 = 0x02D0
    {0x040F, 0xD0},
    // --- Output size = 1280×720 ---
    {0x034C, 0x05}, {0x034D, 0x00},  // X_OUTPUT_SIZE = 1280
    {0x034E, 0x02}, {0x034F, 0xD0},  // Y_OUTPUT_SIZE = 720
    // --- PLL: identical to 720p ---
    {0x0301, 0x05}, {0x0303, 0x02}, {0x0305, 0x02},
    {0x0306, 0x00}, {0x0307, 0x7C}, {0x030B, 0x02},
    {0x030D, 0x04}, {0x0310, 0x01},
    // --- MIPI DPHY timing (from built-in 720p / RPi mode_2x2binned_regs) ---
    {0x3CA0, 0x00}, {0x3CA1, 0x3C}, {0x3CA4, 0x00}, {0x3CA5, 0x3C},
    {0x3CA6, 0x00}, {0x3CA7, 0x00}, {0x3CAA, 0x00}, {0x3CAB, 0x00},
    {0x3CB8, 0x00}, {0x3CB9, 0x1C}, {0x3CBA, 0x00}, {0x3CBB, 0x08},
    {0x3CBC, 0x00}, {0x3CBD, 0x1E}, {0x3CBE, 0x00}, {0x3CBF, 0x0A},
    // --- Exposure/gain ---
    {0x0202, 0x05}, {0x0203, 0xAC},
    {0x0204, 0x00}, {0x0205, 0x00},
    {0x020E, 0x01}, {0x020F, 0x00},
    // --- HDR integration/gain defaults ---
    {0x0224, 0x01}, {0x0225, 0xF4},  // SHORT_INTEGRATION_TIME
    {0x3116, 0x01}, {0x3117, 0xF4},  // MID_INTEGRATION_TIME
    {0x0216, 0x00}, {0x0217, 0x00},  // SHORT_ANALOG_GAIN
    {0x0218, 0x01}, {0x0219, 0x00},
    {0x3118, 0x00}, {0x3119, 0x00},  // MID_ANALOG_GAIN
    {0x311A, 0x01}, {0x311B, 0x00},
    // --- QBC / blanking ---
    {0x341A, 0x00}, {0x341B, 0x00}, {0x341C, 0x00}, {0x341D, 0x00},
    {0x341E, 0x00}, {0x341F, 0x50},  // QBC width = 1280/16 = 80 = 0x50
    {0x3420, 0x00}, {0x3421, 0x3C},  // QBC height = 720/12 = 60 = 0x3C
    {0x3366, 0x00}, {0x3367, 0x00}, {0x3368, 0x00}, {0x3369, 0x00},
    // --- 180° rotation (IMX708_REG_ORIENTATION = 0x0101) ---
    {0x0101, 0x03},  // bit0=H_MIRROR, bit1=V_FLIP → 0x03 = 180°
    {0xFFFF, 0x00}
};

static const esp_cam_sensor_isp_info_t custom_isp_info = {
    .isp_v1_info = {
        .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
        .pclk = 182400000,
        .vts = 1558,
        .hts = 12740,
        .bayer_type = ESP_CAM_SENSOR_BAYER_BGGR,
    }
};

static const esp_cam_sensor_format_t custom_wide_format = {
    .name = "MIPI_2lane_24Minput_RAW10_1280x720",
    .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
    .port = ESP_CAM_SENSOR_MIPI_CSI,
    .xclk = 24000000,
    .width = 1280,
    .height = 720,
    .regs = custom_imx708_regs,
    .regs_size = ARRAY_SIZE(custom_imx708_regs),
    .fps = 30,
    .isp_info = &custom_isp_info,
    .mipi_info = {
        .mipi_clk = 450000000,
        .lane_num = 2,
        .line_sync_en = false,
    },
    .reserved = NULL,
};

#else /* Default: 1536×864 @ 30fps */

static const custom_reginfo_t custom_imx708_regs[] = {
    // --- Timing: LINE_LENGTH from 720p, FRM_LENGTH for 30fps ---
    {0x0342, 0x31},  // LINE_LENGTH_PCK (12740, same as 720p)
    {0x0343, 0xC4},
    {0x0340, 0x0E},  // FRM_LENGTH_LINES (3722 = 0x0E8A) — 30fps
    {0x0341, 0x8A},
    // --- Analog readout: full sensor ---
    {0x0344, 0x00}, {0x0345, 0x00},  // X_ADDR_START (0)
    {0x0346, 0x00}, {0x0347, 0x00},  // Y_ADDR_START (0)
    {0x0348, 0x11}, {0x0349, 0xFF},  // X_ADDR_END (4607)
    {0x034A, 0x0A}, {0x034B, 0x1F},  // Y_ADDR_END (2591)
    // --- Exposure/misc ---
    {0x0220, 0x62}, {0x0222, 0x01},
    // --- Binning: 2×2, averaged weighting (matches 1920×1080 built-in) ---
    {0x0900, 0x01}, {0x0901, 0x22}, {0x0902, 0x08},
    // --- Scaling: scale-only mode (matches 1920×1080 built-in; crop mode 0x43 stalls) ---
    {0x3200, 0x41}, {0x3201, 0x41}, {0x32D5, 0x00}, {0x32D6, 0x00},
    {0x32DB, 0x01}, {0x32DF, 0x00}, {0x350C, 0x00}, {0x350D, 0x00},
    // --- Digital crop: 1536×864 centered in 2304×1296 ---
    {0x0408, 0x01},  // DIG_CROP_X_OFFSET = (2304-1536)/2 = 384 = 0x0180
    {0x0409, 0x80},
    {0x040A, 0x00},  // DIG_CROP_Y_OFFSET = (1296-864)/2 = 216 = 0x00D8
    {0x040B, 0xD8},
    {0x040C, 0x06},  // DIG_CROP_IMAGE_WIDTH = 1536 = 0x0600
    {0x040D, 0x00},
    {0x040E, 0x03},  // DIG_CROP_IMAGE_HEIGHT = 864 = 0x0360
    {0x040F, 0x60},
    // --- Output size = 1536×864 ---
    {0x034C, 0x06}, {0x034D, 0x00},  // X_OUTPUT_SIZE = 1536
    {0x034E, 0x03}, {0x034F, 0x60},  // Y_OUTPUT_SIZE = 864
    // --- PLL: identical to 720p ---
    {0x0301, 0x05}, {0x0303, 0x02}, {0x0305, 0x02},
    {0x0306, 0x00}, {0x0307, 0x7C}, {0x030B, 0x02},
    {0x030D, 0x04}, {0x0310, 0x01},
    // --- Exposure/gain ---
    {0x0202, 0x05}, {0x0203, 0xAC},
    {0x0204, 0x00}, {0x0205, 0x00},
    {0x020E, 0x01}, {0x020F, 0x00},
    // --- 180° rotation (IMX708_REG_ORIENTATION = 0x0101) ---
    {0x0101, 0x03},  // bit0=H_MIRROR, bit1=V_FLIP → 0x03 = 180°
    {0xFFFF, 0x00}
};

static const esp_cam_sensor_isp_info_t custom_isp_info = {
    .isp_v1_info = {
        .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
        .pclk = 182400000,
        .vts = 3722,
        .hts = 12740,
        .bayer_type = ESP_CAM_SENSOR_BAYER_BGGR,  // RGGB → BGGR with 180° rotation
    }
};

static const esp_cam_sensor_format_t custom_wide_format = {
    .name = "MIPI_2lane_24Minput_RAW10_1536x864_wide",
    .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
    .port = ESP_CAM_SENSOR_MIPI_CSI,
    .xclk = 24000000,
    .width = 1536,
    .height = 864,
    .regs = custom_imx708_regs,
    .regs_size = ARRAY_SIZE(custom_imx708_regs),
    .fps = 30,
    .isp_info = &custom_isp_info,
    .mipi_info = {
        .mipi_clk = 450000000,
        .lane_num = 2,
        .line_sync_en = false,
    },
    .reserved = NULL,
};
#endif /* DIAG_USE_1080P15 */

// ============================================================================
// LED
// ============================================================================

static void init_led(void)
{
    gpio_config_t io = { .pin_bit_mask = (1ULL << LED_PIN), .mode = GPIO_MODE_OUTPUT };
    gpio_config(&io);
    gpio_set_level(LED_PIN, 0);
}

static void led_blink(int count, int delay_ms)
{
    for (int i = 0; i < count; i++) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

// ============================================================================
// LDO
// ============================================================================

static esp_err_t init_ldo(void)
{
    // Power-cycle the MIPI LDO to ensure camera gets a clean reset.
    // On software reset, LDOs stay active so camera may be in bad state.
    // We acquire the LDO, release it (which disables the HW output),
    // wait for the camera to fully power down, then re-acquire.
    esp_ldo_channel_config_t ldo3 = { .chan_id = 3, .voltage_mv = 2500 };
    ESP_RETURN_ON_ERROR(esp_ldo_acquire_channel(&ldo3, &ldo_mipi_handle), TAG, "LDO3 fail");

    // VDD_IO5 (3.3V) powers GPIO 38-49 including SD card pins
    esp_ldo_channel_config_t ldo4 = { .chan_id = 4, .voltage_mv = 3300 };
    ESP_RETURN_ON_ERROR(esp_ldo_acquire_channel(&ldo4, &ldo_vdd_io5_handle), TAG, "LDO4 fail");

    // Power-cycle: release LDO3 (disables HW output), wait, re-acquire
    ESP_LOGI(TAG, "Power-cycling camera LDO...");
    esp_ldo_release_channel(ldo_mipi_handle);
    ldo_mipi_handle = NULL;
    vTaskDelay(pdMS_TO_TICKS(2000));  // 2s off: IMX708 needs full power-down for MIPI PHY + PLL reset
    ESP_RETURN_ON_ERROR(esp_ldo_acquire_channel(&ldo3, &ldo_mipi_handle), TAG, "LDO3 re-acquire fail");
    vTaskDelay(pdMS_TO_TICKS(1000));  // 1s stabilization: wait for IMX708 internal oscillator + PLL lock

    ESP_LOGI(TAG, "LDOs OK (MIPI 2.5V, IO5 3.3V)");
    return ESP_OK;
}

// ============================================================================
// XCLK - NOT NEEDED
// RPi Camera Module 3 has an onboard 24MHz oscillator.
// FireBeetle 2's 15-pin FPC connector has no XCLK pin.
// ============================================================================

// ============================================================================
// I2C
// ============================================================================

static esp_err_t init_i2c(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port = CAM_I2C_PORT,
        .sda_io_num = CAM_I2C_SDA,
        .scl_io_num = CAM_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&cfg, &i2c_bus_handle), TAG, "I2C fail");
    ESP_LOGI(TAG, "I2C OK (SDA=%d, SCL=%d)", CAM_I2C_SDA, CAM_I2C_SCL);
    return ESP_OK;
}

// ============================================================================
// ESP VIDEO FRAMEWORK (follows example_init_video.c pattern)
// ============================================================================

static esp_err_t init_esp_video(void)
{
    esp_video_init_csi_config_t csi_cfg = {
        .sccb_config = {
            .init_sccb = false,           // We init I2C ourselves
            .i2c_handle = i2c_bus_handle,
            .freq = CAM_I2C_FREQ,
        },
        .reset_pin = -1,
        .pwdn_pin = -1,
    };
#if CONFIG_ESP_VIDEO_ENABLE_CAMERA_MOTOR_CONTROLLER
    esp_video_init_cam_motor_config_t motor_cfg = {
        .sccb_config = {
            .init_sccb = false,           // Reuse same I2C bus
            .i2c_handle = i2c_bus_handle,
            .freq = CAM_I2C_FREQ,
        },
        .reset_pin = -1,
        .pwdn_pin = -1,
        .signal_pin = -1,
    };
#endif
    esp_video_init_config_t vcfg = {
        .csi = &csi_cfg,
#if CONFIG_ESP_VIDEO_ENABLE_CAMERA_MOTOR_CONTROLLER
        .cam_motor = &motor_cfg,
#endif
    };
    ESP_RETURN_ON_ERROR(esp_video_init(&vcfg), TAG, "esp_video_init fail");
    ESP_LOGI(TAG, "esp_video OK");
    return ESP_OK;
}

// ============================================================================
// VIDEO PIPELINE INIT (follows sd_card example's example_video_start pattern)
// ============================================================================

static void print_video_caps(int fd)
{
    struct v4l2_capability cap;
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
        ESP_LOGI(TAG, "  driver: %s, card: %s", cap.driver, cap.card);
        if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) ESP_LOGI(TAG, "  cap: VIDEO_CAPTURE");
        if (cap.capabilities & V4L2_CAP_STREAMING) ESP_LOGI(TAG, "  cap: STREAMING");
    }
}

static esp_err_t init_video_pipeline(void)
{
    recorder = calloc(1, sizeof(recorder_t));
    if (!recorder) return ESP_ERR_NO_MEM;

    // ---- Open camera capture device ----
    recorder->cap_fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDONLY);
    if (recorder->cap_fd < 0) {
        ESP_LOGE(TAG, "Failed to open %s: %s", ESP_VIDEO_MIPI_CSI_DEVICE_NAME, strerror(errno));
        free(recorder); recorder = NULL;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Camera device: %s (fd=%d)", ESP_VIDEO_MIPI_CSI_DEVICE_NAME, recorder->cap_fd);
    print_video_caps(recorder->cap_fd);

    // Apply custom sensor format with 180° rotation (reg 0x0101=0x03).
    // Uses scale-only mode (0x41) matching 1920×1080 built-in — crop mode (0x43) stalls.
    if (ioctl(recorder->cap_fd, VIDIOC_S_SENSOR_FMT, &custom_wide_format) != 0) {
        ESP_LOGW(TAG, "Custom sensor format failed (%s), falling back to default",
                 strerror(errno));
    } else {
#if DIAG_USE_1080P15
        ESP_LOGI(TAG, "Custom format applied: 1920x1080@30fps scale-only mode (83%% FOV, 180° rot)");
#else
        ESP_LOGI(TAG, "Custom format applied: 1536x864@30fps (67%% sensor FOV, 180° rotated)");
#endif
    }

    // Verify: read back sensor format to confirm custom regs took effect
    {
        esp_cam_sensor_format_t readback_fmt;
        memset(&readback_fmt, 0, sizeof(readback_fmt));
        if (ioctl(recorder->cap_fd, VIDIOC_G_SENSOR_FMT, &readback_fmt) == 0) {
            ESP_LOGI(TAG, "[DIAG] Sensor readback: %dx%d @ %dfps, name='%s'",
                     readback_fmt.width, readback_fmt.height, readback_fmt.fps,
                     readback_fmt.name ? readback_fmt.name : "(null)");
            if (readback_fmt.width != custom_wide_format.width ||
                readback_fmt.height != custom_wide_format.height) {
                ESP_LOGW(TAG, "[DIAG] FORMAT MISMATCH! Expected %dx%d, got %dx%d",
                         custom_wide_format.width, custom_wide_format.height,
                         readback_fmt.width, readback_fmt.height);
            }
        } else {
            ESP_LOGW(TAG, "[DIAG] G_SENSOR_FMT not supported (%s)", strerror(errno));
        }
    }

    // Get sensor's format (custom if applied, or default from menuconfig)
    struct v4l2_format init_fmt = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
    if (ioctl(recorder->cap_fd, VIDIOC_G_FMT, &init_fmt) != 0) {
        ESP_LOGE(TAG, "G_FMT failed: %s", strerror(errno));
        close(recorder->cap_fd); free(recorder); recorder = NULL;
        return ESP_FAIL;
    }
    recorder->width = init_fmt.fmt.pix.width;
    recorder->height = init_fmt.fmt.pix.height;
    ESP_LOGI(TAG, "Pipeline format: %" PRIu32 "x%" PRIu32 " (should match custom format)",
             recorder->width, recorder->height);

#if DIAG_USE_720P
    recorder->width = 1280;
    recorder->height = 720;
    ESP_LOGW(TAG, "DIAG: Overriding to 720p (%" PRIu32 "x%" PRIu32 ")", recorder->width, recorder->height);
#endif

    // Determine encoder dimensions.
    // Skip PPA scaling — encode at native sensor resolution.
    // H.264 HW encoder supports up to 1920x2032; 1536x864 is macroblock-aligned (16x16).
    // PPA YUV420 fractional scaling hangs on ESP32-P4, so we bypass it entirely.
    recorder->enc_width = recorder->width;
    recorder->enc_height = recorder->height;
    ESP_LOGI(TAG, "Encoding at native sensor resolution: %" PRIu32 "x%" PRIu32,
             recorder->enc_width, recorder->enc_height);

    // Find YUV420 format (required by H.264 hardware encoder)
    recorder->capture_fmt = 0;

    int fmt_index = 0;
    while (!recorder->capture_fmt) {
        struct v4l2_fmtdesc fmtdesc = {
            .index = fmt_index++,
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        };
        if (ioctl(recorder->cap_fd, VIDIOC_ENUM_FMT, &fmtdesc) != 0) break;
        ESP_LOGI(TAG, "  Available format[%d]: %.4s (0x%08" PRIx32 ")",
                 fmt_index - 1, (char *)&fmtdesc.pixelformat, fmtdesc.pixelformat);
        if (fmtdesc.pixelformat == V4L2_PIX_FMT_YUV420) {
            recorder->capture_fmt = fmtdesc.pixelformat;
        }
    }

    if (!recorder->capture_fmt) {
        ESP_LOGE(TAG, "No YUV420 format found (required for H.264 encoder)");
        close(recorder->cap_fd); free(recorder); recorder = NULL;
        return ESP_ERR_NOT_SUPPORTED;
    }
    ESP_LOGI(TAG, "Selected capture format: %.4s", (char *)&recorder->capture_fmt);

    // Set camera capture format with limited-range YUV for H.264 compatibility
    struct v4l2_format fmt = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
    fmt.fmt.pix.width = recorder->width;
    fmt.fmt.pix.height = recorder->height;
    fmt.fmt.pix.pixelformat = recorder->capture_fmt;
    fmt.fmt.pix.quantization = V4L2_QUANTIZATION_LIM_RANGE;
    fmt.fmt.pix.ycbcr_enc = V4L2_YCBCR_ENC_601;
    fmt.fmt.pix.colorspace = V4L2_COLORSPACE_SMPTE170M;
    if (ioctl(recorder->cap_fd, VIDIOC_S_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "S_FMT failed: %s", strerror(errno));
        close(recorder->cap_fd); free(recorder); recorder = NULL;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Camera format set: %" PRIu32 "x%" PRIu32 " %.4s",
             fmt.fmt.pix.width, fmt.fmt.pix.height, (char *)&fmt.fmt.pix.pixelformat);

    // Verify effective crop at runtime
    {
        struct v4l2_crop crop = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
        if (ioctl(recorder->cap_fd, VIDIOC_G_CROP, &crop) == 0) {
            ESP_LOGI(TAG, "Effective crop: %" PRIu32 "x%" PRIu32 " at (%" PRId32 ",%" PRId32 ")",
                     (uint32_t)crop.c.width, (uint32_t)crop.c.height,
                     (int32_t)crop.c.left, (int32_t)crop.c.top);
            if ((uint32_t)crop.c.width != recorder->width || (uint32_t)crop.c.height != recorder->height) {
                ESP_LOGW(TAG, "Crop mismatch! Expected %" PRIu32 "x%" PRIu32,
                         recorder->width, recorder->height);
            }
        } else {
            ESP_LOGW(TAG, "VIDIOC_G_CROP not supported: %s", strerror(errno));
        }
    }

    // Request camera capture buffers
    struct v4l2_requestbuffers req = {
        .count = NUM_CAP_BUFFERS,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(recorder->cap_fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "REQBUFS failed: %s", strerror(errno));
        close(recorder->cap_fd); free(recorder); recorder = NULL;
        return ESP_FAIL;
    }

    for (int i = 0; i < NUM_CAP_BUFFERS; i++) {
        struct v4l2_buffer buf = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = i,
        };
        if (ioctl(recorder->cap_fd, VIDIOC_QUERYBUF, &buf) != 0) {
            ESP_LOGE(TAG, "QUERYBUF %d failed", i);
            return ESP_FAIL;
        }
        recorder->cap_buf_len[i] = buf.length;
        recorder->cap_buffer[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                       MAP_SHARED, recorder->cap_fd, buf.m.offset);
        if (recorder->cap_buffer[i] == MAP_FAILED) {
            ESP_LOGE(TAG, "mmap %d failed", i);
            return ESP_FAIL;
        }
        if (ioctl(recorder->cap_fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "QBUF %d failed", i);
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "  cap_buf[%d]: %p %" PRIu32 " bytes", i, recorder->cap_buffer[i], buf.length);
    }

    // ---- Open H.264 M2M encoder device ----
    recorder->m2m_fd = open(ESP_VIDEO_H264_DEVICE_NAME, O_RDONLY);
    if (recorder->m2m_fd < 0) {
        ESP_LOGE(TAG, "Failed to open %s: %s", ESP_VIDEO_H264_DEVICE_NAME, strerror(errno));
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "H.264 device: %s (fd=%d)", ESP_VIDEO_H264_DEVICE_NAME, recorder->m2m_fd);
    print_video_caps(recorder->m2m_fd);

    // Set H.264 encoder controls
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control control[1];
    controls.ctrl_class = V4L2_CID_CODEC_CLASS;
    controls.count = 1;
    controls.controls = control;

    control[0].id = V4L2_CID_MPEG_VIDEO_H264_I_PERIOD;
    control[0].value = H264_GOP;
    if (ioctl(recorder->m2m_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGW(TAG, "Failed to set H.264 GOP");
    }
    control[0].id = V4L2_CID_MPEG_VIDEO_BITRATE;
    control[0].value = H264_BITRATE;
    if (ioctl(recorder->m2m_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGW(TAG, "Failed to set H.264 bitrate");
    }
    control[0].id = V4L2_CID_MPEG_VIDEO_H264_MIN_QP;
    control[0].value = H264_MIN_QP;
    if (ioctl(recorder->m2m_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGW(TAG, "Failed to set H.264 min QP");
    }
    control[0].id = V4L2_CID_MPEG_VIDEO_H264_MAX_QP;
    control[0].value = H264_MAX_QP;
    if (ioctl(recorder->m2m_fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGW(TAG, "Failed to set H.264 max QP");
    }

    // Configure M2M output stream (raw frames IN — uses encoder dims, may differ from sensor)
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    fmt.fmt.pix.width = recorder->enc_width;
    fmt.fmt.pix.height = recorder->enc_height;
    fmt.fmt.pix.pixelformat = recorder->capture_fmt;
    if (ioctl(recorder->m2m_fd, VIDIOC_S_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "M2M S_FMT output failed: %s", strerror(errno));
        return ESP_FAIL;
    }

    memset(&req, 0, sizeof(req));
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    req.memory = V4L2_MEMORY_USERPTR;
    if (ioctl(recorder->m2m_fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "M2M REQBUFS output failed: %s", strerror(errno));
        return ESP_FAIL;
    }

    // Configure M2M capture stream (H.264 frames OUT)
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = recorder->enc_width;
    fmt.fmt.pix.height = recorder->enc_height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
    if (ioctl(recorder->m2m_fd, VIDIOC_S_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "M2M S_FMT capture failed: %s", strerror(errno));
        return ESP_FAIL;
    }

    memset(&req, 0, sizeof(req));
    req.count = NUM_M2M_CAP_BUFS;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(recorder->m2m_fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "M2M REQBUFS capture failed: %s", strerror(errno));
        return ESP_FAIL;
    }

    // mmap all M2M capture buffers for zero-copy pipeline
    for (int i = 0; i < NUM_M2M_CAP_BUFS; i++) {
        struct v4l2_buffer m2m_buf = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = i,
        };
        if (ioctl(recorder->m2m_fd, VIDIOC_QUERYBUF, &m2m_buf) != 0) {
            ESP_LOGE(TAG, "M2M QUERYBUF[%d] failed: %s", i, strerror(errno));
            return ESP_FAIL;
        }
        recorder->m2m_cap_buf_lens[i] = m2m_buf.length;
        recorder->m2m_cap_buffers[i] = mmap(NULL, m2m_buf.length, PROT_READ | PROT_WRITE,
                                            MAP_SHARED, recorder->m2m_fd, m2m_buf.m.offset);
        if (recorder->m2m_cap_buffers[i] == MAP_FAILED) {
            ESP_LOGE(TAG, "M2M mmap[%d] failed", i);
            return ESP_FAIL;
        }
        if (ioctl(recorder->m2m_fd, VIDIOC_QBUF, &m2m_buf) != 0) {
            ESP_LOGE(TAG, "M2M QBUF[%d] failed", i);
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "  m2m_cap[%d]: %p %" PRIu32 " bytes", i, recorder->m2m_cap_buffers[i], m2m_buf.length);
    }

    // Allocate rotating PSRAM staging buffers for async write pipeline
    recorder->jpeg_buf_write_idx = 0;
    for (int i = 0; i < NUM_JPEG_BUFS; i++) {
        recorder->jpeg_out_buf[i] = heap_caps_aligned_alloc(128, JPEG_BUF_SIZE,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!recorder->jpeg_out_buf[i]) {
            ESP_LOGE(TAG, "Staging buffer %d alloc failed", i);
            return ESP_ERR_NO_MEM;
        }
    }
    ESP_LOGI(TAG, "Staging buffers: %d x %dKB PSRAM", NUM_JPEG_BUFS, JPEG_BUF_SIZE / 1024);

    write_queue = xQueueCreate(NUM_JPEG_BUFS, sizeof(frame_msg_t));
    jpeg_buf_sem = xSemaphoreCreateCounting(NUM_JPEG_BUFS, NUM_JPEG_BUFS);

    ESP_LOGI(TAG, "Video pipeline init OK");
    return ESP_OK;
}

static void deinit_video_pipeline(void)
{
    if (!recorder) return;

    // Unmap and close camera capture
    for (int i = 0; i < NUM_CAP_BUFFERS; i++) {
        if (recorder->cap_buffer[i] && recorder->cap_buffer[i] != MAP_FAILED) {
            munmap(recorder->cap_buffer[i], recorder->cap_buf_len[i]);
            recorder->cap_buffer[i] = NULL;
        }
    }
    if (recorder->cap_fd >= 0) {
        close(recorder->cap_fd);
        recorder->cap_fd = -1;
    }

    // Unmap and close H.264 M2M encoder
    for (int i = 0; i < NUM_M2M_CAP_BUFS; i++) {
        if (recorder->m2m_cap_buffers[i] && recorder->m2m_cap_buffers[i] != MAP_FAILED) {
            munmap(recorder->m2m_cap_buffers[i], recorder->m2m_cap_buf_lens[i]);
            recorder->m2m_cap_buffers[i] = NULL;
        }
    }
    if (recorder->m2m_fd >= 0) {
        close(recorder->m2m_fd);
        recorder->m2m_fd = -1;
    }

    // Clean up PPA scaler
    if (ppa_scaled_buf) {
        heap_caps_free(ppa_scaled_buf);
        ppa_scaled_buf = NULL;
        ppa_scaled_buf_size = 0;
    }
    if (ppa_srm_handle) {
        ppa_unregister_client(ppa_srm_handle);
        ppa_srm_handle = NULL;
    }

    // Free staging buffers
    if (recorder) {
        for (int i = 0; i < NUM_JPEG_BUFS; i++) {
            if (recorder->jpeg_out_buf[i]) {
                heap_caps_free(recorder->jpeg_out_buf[i]);
                recorder->jpeg_out_buf[i] = NULL;
            }
        }
    }

    if (write_queue) {
        vQueueDelete(write_queue);
        write_queue = NULL;
    }
    if (jpeg_buf_sem) {
        vSemaphoreDelete(jpeg_buf_sem);
        jpeg_buf_sem = NULL;
    }

    free(recorder);
    recorder = NULL;
    ESP_LOGI(TAG, "Video pipeline deinitialized");
}

// ============================================================================
// ISP WHITE BALANCE
// ============================================================================

static esp_err_t init_isp_white_balance(void)
{
    int isp_fd = open(ESP_VIDEO_ISP1_DEVICE_NAME, O_RDWR);
    if (isp_fd < 0) {
        ESP_LOGE(TAG, "Failed to open ISP device %s: %s", ESP_VIDEO_ISP1_DEVICE_NAME, strerror(errno));
        return ESP_FAIL;
    }

    // Enable manual white balance with gains tuned for daylight/IMX708.
    // Without WB the raw Bayer→YUV conversion produces a strong green cast
    // because silicon sensors are ~2x more sensitive to green.
    esp_video_isp_wb_t wb = {
        .enable = true,
        .red_gain = 1.6,
        .blue_gain = 1.4,
    };

    struct v4l2_ext_control ctrl;
    struct v4l2_ext_controls ctrls;
    memset(&ctrl, 0, sizeof(ctrl));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrl.id = V4L2_CID_USER_ESP_ISP_WB;
    ctrl.size = sizeof(wb);
    ctrl.p_u8 = (uint8_t *)&wb;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_USER;
    ctrls.count = 1;
    ctrls.controls = &ctrl;

    if (ioctl(isp_fd, VIDIOC_S_EXT_CTRLS, &ctrls) != 0) {
        ESP_LOGE(TAG, "Failed to set ISP white balance: %s", strerror(errno));
        close(isp_fd);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ISP WB set: red=%.1f blue=%.1f", wb.red_gain, wb.blue_gain);
    close(isp_fd);
    return ESP_OK;
}

// ============================================================================
// PDM MICROPHONE (I2S)
// ============================================================================

static esp_err_t init_pdm_mic(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = AUDIO_DMA_BUF_COUNT;
    chan_cfg.dma_frame_num = AUDIO_DMA_BUF_SIZE;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, NULL, &i2s_rx_handle), TAG, "I2S new channel fail");

    i2s_pdm_rx_config_t pdm_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = PDM_CLK_PIN,
            .din = PDM_DATA_PIN,
            .invert_flags = { .clk_inv = false },
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_pdm_rx_mode(i2s_rx_handle, &pdm_cfg), TAG, "I2S PDM RX init fail");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(i2s_rx_handle), TAG, "I2S enable fail");

    // Allocate audio ring buffer in PSRAM
    audio_ring = heap_caps_calloc(1, AUDIO_RING_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!audio_ring) {
        ESP_LOGE(TAG, "Audio ring buffer alloc failed");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "PDM mic OK: %dHz %dbit mono (CLK=GPIO%d, DATA=GPIO%d)",
             AUDIO_SAMPLE_RATE, AUDIO_BITS, PDM_CLK_PIN, PDM_DATA_PIN);
    return ESP_OK;
}

// Audio capture task - reads PDM mic into ring buffer
static void audio_capture_task(void *arg)
{
    size_t read_buf_size = AUDIO_DMA_BUF_SIZE * sizeof(int16_t);
    uint8_t *read_buf = heap_caps_malloc(read_buf_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (!read_buf) {
        ESP_LOGE(TAG, "Audio read buffer alloc failed");
        vTaskDelete(NULL);
        return;
    }

    audio_running = true;
    ESP_LOGI(TAG, "Audio capture started");
    int audio_reads = 0;

    while (recording) {
        size_t bytes_read = 0;
        esp_err_t ret = i2s_channel_read(i2s_rx_handle, read_buf, read_buf_size, &bytes_read, pdMS_TO_TICKS(100));
        if (ret != ESP_OK || bytes_read == 0) continue;

        // Write to ring buffer (overwrite oldest if full)
        for (size_t i = 0; i < bytes_read; i++) {
            audio_ring[audio_ring_wr % AUDIO_RING_SIZE] = read_buf[i];
            audio_ring_wr++;
        }

        audio_reads++;
    }

    audio_running = false;
    free(read_buf);
    ESP_LOGI(TAG, "Audio capture stopped");
    audio_task_handle = NULL;
    vTaskDelete(NULL);
}

// Drain available audio from ring buffer into a flat buffer.
// Returns number of bytes copied.
static uint32_t audio_drain(uint8_t *dst, uint32_t max_bytes)
{
    uint32_t wr = audio_ring_wr;
    uint32_t rd = audio_ring_rd;
    uint32_t avail = wr - rd;
    if (avail > AUDIO_RING_SIZE) {
        // Overrun — skip to latest data
        rd = wr - AUDIO_RING_SIZE;
    }
    if (avail > max_bytes) avail = max_bytes;
    if (avail == 0) return 0;
    // Use memcpy instead of byte-by-byte (PSRAM is slow per-byte)
    uint32_t start = rd % AUDIO_RING_SIZE;
    uint32_t first = AUDIO_RING_SIZE - start;
    if (first >= avail) {
        memcpy(dst, audio_ring + start, avail);
    } else {
        memcpy(dst, audio_ring + start, first);
        memcpy(dst + first, audio_ring, avail - first);
    }
    audio_ring_rd = rd + avail;
    return avail;
}

// ============================================================================
// SD CARD
// ============================================================================

static esp_err_t init_sd_card(void)
{
    // Power on SD card (FireBeetle 2 uses GPIO45 active LOW)
    gpio_config_t pwr = { .pin_bit_mask = (1ULL << GPIO_NUM_45), .mode = GPIO_MODE_OUTPUT };
    gpio_config(&pwr);

    // Full power cycle: 1s off, 1s on for clean card initialization
    ESP_LOGI(TAG, "SD: power cycling card (GPIO45)...");
    gpio_set_level(GPIO_NUM_45, 1);  // OFF
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(GPIO_NUM_45, 0);  // ON
    vTaskDelay(pdMS_TO_TICKS(1000));

    // LDO4 (VDD_IO5 3.3V) for SD IO pins is already set up in init_ldo()

    esp_vfs_fat_sdmmc_mount_config_t mount = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 64 * 1024,
    };

    // Try 4-bit 40MHz first (best proven throughput), fall back to slower modes
    struct { int w; int f; const char *n; } modes[] = {
        {4, SDMMC_FREQ_HIGHSPEED, "4-bit 40MHz"},
        {4, SDMMC_FREQ_DEFAULT,   "4-bit 20MHz"},
        {1, SDMMC_FREQ_DEFAULT,   "1-bit 20MHz"},
    };

    for (int i = 0; i < 3; i++) {
        ESP_LOGI(TAG, "SD: trying %s", modes[i].n);
        sdmmc_host_t host = SDMMC_HOST_DEFAULT();
        host.slot = SDMMC_HOST_SLOT_1;
        host.max_freq_khz = modes[i].f;

        sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
        slot.width = modes[i].w;
        slot.clk = SD_CLK_PIN;
        slot.cmd = SD_CMD_PIN;
        slot.d0 = SD_D0_PIN;
        slot.d1 = SD_D1_PIN;
        slot.d2 = SD_D2_PIN;
        slot.d3 = SD_D3_PIN;
        slot.cd = GPIO_NUM_NC;
        slot.wp = GPIO_NUM_NC;
        slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

        esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot, &mount, &sd_card);
        if (ret == ESP_OK) {
            sdmmc_card_print_info(stdout, sd_card);
            ESP_LOGI(TAG, "SD mounted: %s, %llu MB (%s)",
                     sd_card->cid.name,
                     ((uint64_t)sd_card->csd.capacity) * sd_card->csd.sector_size / (1024*1024),
                     modes[i].n);
            mkdir("/sdcard/vid", 0);
            return ESP_OK;
        }
        ESP_LOGW(TAG, "SD %s failed: 0x%x", modes[i].n, ret);
        sdmmc_host_deinit();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGE(TAG, "SD mount failed");
    return ESP_FAIL;
}

// ============================================================================
// AVI H.264 Writer Helpers
// ============================================================================

// Write a 32-bit little-endian value
static void avi_write_u32(int fd, uint32_t val)
{
    uint8_t b[4] = { val & 0xFF, (val >> 8) & 0xFF, (val >> 16) & 0xFF, (val >> 24) & 0xFF };
    write(fd, b, 4);
}

// Write a FourCC string
static void avi_write_4cc(int fd, const char *cc)
{
    write(fd, cc, 4);
}

// Write 16-bit little-endian value
static void avi_write_u16(int fd, uint16_t val)
{
    uint8_t b[2] = { val & 0xFF, (val >> 8) & 0xFF };
    write(fd, b, 2);
}

// Write AVI headers with 2 streams (video + audio), returns offset to 'movi' data start
static uint32_t avi_write_header(int fd, uint32_t width, uint32_t height, uint32_t fps)
{
    uint32_t us_per_frame = 1000000 / fps;
    uint32_t frame_size = width * height; // estimated max
    uint32_t audio_rate = AUDIO_SAMPLE_RATE;
    uint32_t audio_bytes_per_sec = audio_rate * AUDIO_CHANNELS * (AUDIO_BITS / 8);
    uint32_t audio_block_align = AUDIO_CHANNELS * (AUDIO_BITS / 8);

    // RIFF header (file size placeholder = 0, updated at end)
    avi_write_4cc(fd, "RIFF");
    avi_write_u32(fd, 0);        // placeholder: file size - 8
    avi_write_4cc(fd, "AVI ");

    // LIST 'hdrl'
    avi_write_4cc(fd, "LIST");
    uint32_t hdrl_size_pos = lseek(fd, 0, SEEK_CUR);
    avi_write_u32(fd, 0);        // placeholder: hdrl size
    uint32_t hdrl_start = lseek(fd, 0, SEEK_CUR);
    avi_write_4cc(fd, "hdrl");

    // 'avih' - Main AVI Header (56 bytes)
    avi_write_4cc(fd, "avih");
    avi_write_u32(fd, 56);
    avi_write_u32(fd, us_per_frame);  // dwMicroSecPerFrame
    avi_write_u32(fd, frame_size);    // dwMaxBytesPerSec (estimate)
    avi_write_u32(fd, 0);             // dwPaddingGranularity
    avi_write_u32(fd, 0);             // dwFlags: 0 initially (AVIF_HASINDEX set at finalization)
    avi_write_u32(fd, 0);             // dwTotalFrames (placeholder, updated at end)
    avi_write_u32(fd, 0);             // dwInitialFrames
    avi_write_u32(fd, 2);             // dwStreams (video + audio)
    avi_write_u32(fd, frame_size);    // dwSuggestedBufferSize
    avi_write_u32(fd, width);         // dwWidth
    avi_write_u32(fd, height);        // dwHeight
    avi_write_u32(fd, 0);             // dwReserved[0]
    avi_write_u32(fd, 0);             // dwReserved[1]
    avi_write_u32(fd, 0);             // dwReserved[2]
    avi_write_u32(fd, 0);             // dwReserved[3]

    // ---- Stream 0: Video (H.264) ----
    avi_write_4cc(fd, "LIST");
    uint32_t strl0_size_pos = lseek(fd, 0, SEEK_CUR);
    avi_write_u32(fd, 0);
    uint32_t strl0_start = lseek(fd, 0, SEEK_CUR);
    avi_write_4cc(fd, "strl");

    // 'strh' - Video Stream Header (56 bytes)
    avi_write_4cc(fd, "strh");
    avi_write_u32(fd, 56);
    avi_write_4cc(fd, "vids");        // fccType
    avi_write_4cc(fd, "H264");        // fccHandler
    avi_write_u32(fd, 0);             // dwFlags
    avi_write_u32(fd, 0);             // wPriority + wLanguage
    avi_write_u32(fd, 0);             // dwInitialFrames
    avi_write_u32(fd, 1);             // dwScale
    avi_write_u32(fd, fps);           // dwRate
    avi_write_u32(fd, 0);             // dwStart
    avi_write_u32(fd, 0);             // dwLength (placeholder, updated at end)
    avi_write_u32(fd, frame_size);    // dwSuggestedBufferSize
    avi_write_u32(fd, 0);             // dwQuality
    avi_write_u32(fd, 0);             // dwSampleSize
    avi_write_u32(fd, 0);             // rcFrame (left, top)
    avi_write_u32(fd, (height << 16) | width); // rcFrame (right, bottom)

    // 'strf' - Video Stream Format (BITMAPINFOHEADER, 40 bytes)
    avi_write_4cc(fd, "strf");
    avi_write_u32(fd, 40);
    avi_write_u32(fd, 40);            // biSize
    avi_write_u32(fd, width);         // biWidth
    avi_write_u32(fd, height);        // biHeight
    avi_write_u32(fd, (24 << 16) | 1); // biPlanes(1) + biBitCount(24)
    avi_write_4cc(fd, "H264");        // biCompression
    avi_write_u32(fd, frame_size);    // biSizeImage
    avi_write_u32(fd, 0);             // biXPelsPerMeter
    avi_write_u32(fd, 0);             // biYPelsPerMeter
    avi_write_u32(fd, 0);             // biClrUsed
    avi_write_u32(fd, 0);             // biClrImportant

    // Patch strl0 size
    uint32_t strl0_end = lseek(fd, 0, SEEK_CUR);
    lseek(fd, strl0_size_pos, SEEK_SET);
    avi_write_u32(fd, strl0_end - strl0_start);
    lseek(fd, strl0_end, SEEK_SET);

    // ---- Stream 1: Audio (PCM) ----
    avi_write_4cc(fd, "LIST");
    uint32_t strl1_size_pos = lseek(fd, 0, SEEK_CUR);
    avi_write_u32(fd, 0);
    uint32_t strl1_start = lseek(fd, 0, SEEK_CUR);
    avi_write_4cc(fd, "strl");

    // 'strh' - Audio Stream Header (56 bytes)
    avi_write_4cc(fd, "strh");
    avi_write_u32(fd, 56);
    avi_write_4cc(fd, "auds");        // fccType
    avi_write_u32(fd, 0);             // fccHandler (0 for PCM audio)
    avi_write_u32(fd, 0);             // dwFlags
    avi_write_u32(fd, 0);             // wPriority + wLanguage
    avi_write_u32(fd, 0);             // dwInitialFrames
    avi_write_u32(fd, audio_block_align);  // dwScale = block align
    avi_write_u32(fd, audio_bytes_per_sec); // dwRate = bytes/sec
    avi_write_u32(fd, 0);             // dwStart
    avi_write_u32(fd, 0);             // dwLength (placeholder: total audio samples)
    avi_write_u32(fd, audio_bytes_per_sec); // dwSuggestedBufferSize (1 sec)
    avi_write_u32(fd, 0);             // dwQuality
    avi_write_u32(fd, audio_block_align);  // dwSampleSize
    avi_write_u32(fd, 0);             // rcFrame (unused for audio)
    avi_write_u32(fd, 0);             // rcFrame

    // 'strf' - Audio Stream Format (PCMWAVEFORMAT, 16 bytes — no cbSize for PCM)
    avi_write_4cc(fd, "strf");
    avi_write_u32(fd, 16);            // chunk size
    avi_write_u16(fd, 1);             // wFormatTag: WAVE_FORMAT_PCM
    avi_write_u16(fd, AUDIO_CHANNELS); // nChannels
    avi_write_u32(fd, audio_rate);    // nSamplesPerSec
    avi_write_u32(fd, audio_bytes_per_sec); // nAvgBytesPerSec
    avi_write_u16(fd, audio_block_align);   // nBlockAlign
    avi_write_u16(fd, AUDIO_BITS);    // wBitsPerSample

    // Patch strl1 size
    uint32_t strl1_end = lseek(fd, 0, SEEK_CUR);
    lseek(fd, strl1_size_pos, SEEK_SET);
    avi_write_u32(fd, strl1_end - strl1_start);
    lseek(fd, strl1_end, SEEK_SET);

    // Patch hdrl size
    uint32_t hdrl_end = lseek(fd, 0, SEEK_CUR);
    lseek(fd, hdrl_size_pos, SEEK_SET);
    avi_write_u32(fd, hdrl_end - hdrl_start);
    lseek(fd, hdrl_end, SEEK_SET);

    // LIST 'movi'
    avi_write_4cc(fd, "LIST");
    avi_write_u32(fd, 0);        // placeholder: movi size (updated at end)
    uint32_t movi_fourcc_pos = lseek(fd, 0, SEEK_CUR);
    avi_write_4cc(fd, "movi");

    // Return position of 'movi' fourcc — idx1 offsets are relative to this.
    // First data chunk starts at movi_fourcc_pos + 4.
    return movi_fourcc_pos;
}

// Timing accumulators for avi_write_frame breakdown
static int64_t awf_memcpy_total_us = 0;
static int64_t awf_write_total_us = 0;
static int awf_count = 0;

// Write one H.264 frame as AVI chunk via internal DMA staging.
// Merges the 8-byte AVI header (fourcc + size) INTO the first staging chunk
// so the entire frame is written as sector-aligned write() calls from internal RAM.
static uint32_t avi_write_frame(int fd, const uint8_t *jpeg_data, uint32_t jpeg_size,
                                uint8_t *stg, size_t stg_size)
{
    if (stg && stg_size > 8) {
        // Pack AVI chunk header into staging buffer first 8 bytes
        stg[0] = '0'; stg[1] = '0'; stg[2] = 'd'; stg[3] = 'c';
        stg[4] = jpeg_size & 0xFF;
        stg[5] = (jpeg_size >> 8) & 0xFF;
        stg[6] = (jpeg_size >> 16) & 0xFF;
        stg[7] = (jpeg_size >> 24) & 0xFF;

        // Fill rest of first chunk with frame data
        size_t first_data = stg_size - 8;
        if (first_data > jpeg_size) first_data = jpeg_size;
        int64_t t_mc = esp_timer_get_time();
        memcpy(stg + 8, jpeg_data, first_data);
        awf_memcpy_total_us += esp_timer_get_time() - t_mc;
        size_t first_total = 8 + first_data;
        // Pad to even if this is the only chunk
        if (first_data == jpeg_size && (jpeg_size & 1)) {
            stg[first_total] = 0;
            first_total++;
        }
        int64_t t_fw = esp_timer_get_time();
        write(fd, stg, first_total);
        awf_write_total_us += esp_timer_get_time() - t_fw;

        // Remaining data in full chunks
        size_t off = first_data;
        while (off < jpeg_size) {
            size_t chunk = jpeg_size - off;
            if (chunk > stg_size) chunk = stg_size;
            t_mc = esp_timer_get_time();
            memcpy(stg, jpeg_data + off, chunk);
            awf_memcpy_total_us += esp_timer_get_time() - t_mc;
            size_t wr = chunk;
            // Pad last chunk to even boundary
            if (off + chunk >= jpeg_size && (jpeg_size & 1)) {
                stg[wr] = 0;
                wr++;
            }
            t_fw = esp_timer_get_time();
            write(fd, stg, wr);
            awf_write_total_us += esp_timer_get_time() - t_fw;
            off += chunk;
        }
        awf_count++;
    } else {
        // Fallback: separate writes (slow path)
        avi_write_4cc(fd, "00dc");
        avi_write_u32(fd, jpeg_size);
        write(fd, jpeg_data, jpeg_size);
        if (jpeg_size & 1) {
            uint8_t z = 0;
            write(fd, &z, 1);
        }
        awf_count++;
    }
    return jpeg_size;
}

// Write an audio chunk to the AVI movi section (same merged-header approach)
static uint32_t avi_write_audio(int fd, const uint8_t *pcm_data, uint32_t pcm_size,
                                uint8_t *stg, size_t stg_size)
{
    if (stg && stg_size > 8) {
        stg[0] = '0'; stg[1] = '1'; stg[2] = 'w'; stg[3] = 'b';
        stg[4] = pcm_size & 0xFF;
        stg[5] = (pcm_size >> 8) & 0xFF;
        stg[6] = (pcm_size >> 16) & 0xFF;
        stg[7] = (pcm_size >> 24) & 0xFF;

        size_t first_data = stg_size - 8;
        if (first_data > pcm_size) first_data = pcm_size;
        memcpy(stg + 8, pcm_data, first_data);
        size_t first_total = 8 + first_data;
        if (first_data == pcm_size && (pcm_size & 1)) {
            stg[first_total] = 0;
            first_total++;
        }
        write(fd, stg, first_total);

        size_t off = first_data;
        while (off < pcm_size) {
            size_t chunk = pcm_size - off;
            if (chunk > stg_size) chunk = stg_size;
            memcpy(stg, pcm_data + off, chunk);
            size_t wr = chunk;
            if (off + chunk >= pcm_size && (pcm_size & 1)) {
                stg[wr] = 0;
                wr++;
            }
            write(fd, stg, wr);
            off += chunk;
        }
    } else {
        avi_write_4cc(fd, "01wb");
        avi_write_u32(fd, pcm_size);
        write(fd, pcm_data, pcm_size);
        if (pcm_size & 1) {
            uint8_t z = 0;
            write(fd, &z, 1);
        }
    }
    return pcm_size;
}

// Index entry type for combined video+audio idx1
typedef struct {
    char fourcc[4];    // "00dc" or "01wb"
    uint32_t flags;    // AVIIF_KEYFRAME or 0
    uint32_t offset;   // offset from movi start
    uint32_t size;     // chunk data size
} idx1_entry_t;

// Update AVI headers in-place during recording (makes file playable if interrupted)
// Does NOT write idx1 — that's only at finalization
static void avi_update_headers(int fd, uint32_t movi_start, uint32_t current_pos,
                               uint32_t total_video_frames, uint32_t total_audio_samples,
                               uint32_t fps)
{
    off_t saved_pos = lseek(fd, 0, SEEK_CUR);
    uint32_t movi_data_size = current_pos - movi_start; // movi_start is at 'movi' fourcc

    // Patch RIFF size (offset 4): covers everything up to current write pos
    lseek(fd, 4, SEEK_SET);
    avi_write_u32(fd, current_pos - 8);

    // Patch avih dwMicroSecPerFrame (offset 32)
    uint32_t actual_us_per_frame = fps > 0 ? 1000000 / fps : 100000;
    lseek(fd, 32, SEEK_SET);
    avi_write_u32(fd, actual_us_per_frame);

    // Patch avih dwTotalFrames (offset 48)
    lseek(fd, 48, SEEK_SET);
    avi_write_u32(fd, total_video_frames);

    // Patch video strh dwRate (offset 132)
    lseek(fd, 132, SEEK_SET);
    avi_write_u32(fd, fps);

    // Patch video strh dwLength (offset 140)
    lseek(fd, 140, SEEK_SET);
    avi_write_u32(fd, total_video_frames);

    // Patch audio strh dwLength (offset 264 — strh is before strf, so unaffected by strf size)
    lseek(fd, 264, SEEK_SET);
    avi_write_u32(fd, total_audio_samples);

    // Patch movi LIST size (4 bytes before 'movi' fourcc)
    lseek(fd, movi_start - 4, SEEK_SET);
    avi_write_u32(fd, movi_data_size);

    // Restore write position
    lseek(fd, saved_pos, SEEK_SET);
}

// Finalize AVI: update headers with actual frame count, write index
static void avi_finalize(int fd, uint32_t movi_start, uint32_t total_video_frames,
                         uint32_t total_audio_samples,
                         const idx1_entry_t *idx_entries, uint32_t idx_count,
                         uint32_t width, uint32_t height, uint32_t fps)
{
    uint32_t movi_end = lseek(fd, 0, SEEK_CUR);
    uint32_t movi_data_size = movi_end - movi_start;

    // Write idx1 index (video + audio entries)
    avi_write_4cc(fd, "idx1");
    avi_write_u32(fd, idx_count * 16);
    for (uint32_t i = 0; i < idx_count; i++) {
        write(fd, idx_entries[i].fourcc, 4);
        avi_write_u32(fd, idx_entries[i].flags);
        avi_write_u32(fd, idx_entries[i].offset);
        avi_write_u32(fd, idx_entries[i].size);
    }

    uint32_t file_end = lseek(fd, 0, SEEK_CUR);

    // Patch RIFF size (offset 4)
    lseek(fd, 4, SEEK_SET);
    avi_write_u32(fd, file_end - 8);

    // Patch avih dwMicroSecPerFrame (offset 32) with actual FPS
    uint32_t actual_us_per_frame = fps > 0 ? 1000000 / fps : 100000;
    lseek(fd, 32, SEEK_SET);
    avi_write_u32(fd, actual_us_per_frame);

    // Patch avih dwTotalFrames (offset 48)
    lseek(fd, 48, SEEK_SET);
    avi_write_u32(fd, total_video_frames);

    // Patch video strh dwRate (offset 132) with actual FPS
    lseek(fd, 132, SEEK_SET);
    avi_write_u32(fd, fps);

    // Patch video strh dwLength (offset 140)
    lseek(fd, 140, SEEK_SET);
    avi_write_u32(fd, total_video_frames);

    // Patch audio strh dwLength (offset 264)
    lseek(fd, 264, SEEK_SET);
    avi_write_u32(fd, total_audio_samples);

    // Patch avih dwFlags (offset 44): AVIF_HASINDEX | AVIF_ISINTERLEAVED
    lseek(fd, 44, SEEK_SET);
    avi_write_u32(fd, 0x110);

    // Patch movi LIST size (4 bytes before 'movi' fourcc)
    lseek(fd, movi_start - 4, SEEK_SET);
    avi_write_u32(fd, movi_data_size);

    lseek(fd, file_end, SEEK_SET);
}

// ============================================================================
// ============================================================================
// SDMMC DMA TUNING
// Configures hardware registers for maximum SD card DMA throughput.
// Must be called AFTER sd card is mounted (SDMMC host is initialized).
// ============================================================================
static void sdmmc_dma_tuning(void)
{
    // SDMMC FIFOTH: Set DMA burst to 32B (default is 1B)
    uint32_t fifoth = 0;
    fifoth |= (128 & SDHOST_TX_WMARK_V) << SDHOST_TX_WMARK_S;
    fifoth |= (127 & SDHOST_RX_WMARK_V) << SDHOST_RX_WMARK_S;
    fifoth |= (4 & SDHOST_DMA_MULTIPLE_TRANSACTION_SIZE_V) << SDHOST_DMA_MULTIPLE_TRANSACTION_SIZE_S;
    REG_WRITE(SDHOST_FIFOTH_REG, fifoth);

    // TCM arbiter: Boost DMA weight to max (default 2/7)
    uint32_t wrr_cfg = REG_READ(HP_SYSTEM_TCM_RAM_WRR_CONFIG_REG);
    wrr_cfg &= ~(HP_SYSTEM_REG_TCM_RAM_DMA_WT_V << HP_SYSTEM_REG_TCM_RAM_DMA_WT_S);
    wrr_cfg |= (7 << HP_SYSTEM_REG_TCM_RAM_DMA_WT_S);
    REG_WRITE(HP_SYSTEM_TCM_RAM_WRR_CONFIG_REG, wrr_cfg);

    ESP_LOGI(TAG, "SDMMC DMA tuning applied (burst=32B, DMA_WT=7)");
}



// ============================================================================
// SD WRITE TASK - runs on Core 1
// Writes H.264 AVI video file to SD card.
// Pre-allocates file to avoid FAT cluster allocation during recording.
// ============================================================================

static void sd_write_task(void *arg)
{
    frame_msg_t msg;
    int written = 0;
    int audio_chunks_written = 0;
    uint32_t total_audio_samples = 0;
    int64_t start_us = esp_timer_get_time();

    // Allocate combined index array (video + audio entries) in PSRAM
    // Max entries: video frames + audio chunks (one per video frame)
    uint32_t max_frames = RECORD_DURATION_SEC * AVI_FPS * 2;
    uint32_t max_idx = max_frames * 2 + 100; // video + audio entries
    idx1_entry_t *idx_entries = heap_caps_malloc(max_idx * sizeof(idx1_entry_t), MALLOC_CAP_SPIRAM);
    uint32_t idx_count = 0;

    // Audio drain buffer in PSRAM — must handle full ring buffer drain at once
    // (actual FPS may be much lower than AVI_FPS due to SD stalls)
    uint32_t audio_chunk_max = AUDIO_RING_SIZE;
    uint8_t *audio_drain_buf = heap_caps_malloc(audio_chunk_max, MALLOC_CAP_SPIRAM);

    if (!idx_entries || !audio_drain_buf) {
        ESP_LOGE(TAG, "Failed to allocate AVI index/audio arrays");
        if (idx_entries) free(idx_entries);
        if (audio_drain_buf) free(audio_drain_buf);
        if (write_task_ready) xSemaphoreGive(write_task_ready);
        vTaskDelete(NULL);
        return;
    }

    // Find next available filename (don't overwrite previous recordings)
    char filename[64];
    int file_num = 0;
    for (file_num = 0; file_num < 10000; file_num++) {
        snprintf(filename, sizeof(filename), "/sdcard/vid/rec_%04d.avi", file_num);
        struct stat st;
        if (stat(filename, &st) != 0) break;  // file doesn't exist, use this name
    }

    int fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd < 0) {
        ESP_LOGE(TAG, "Failed to create %s: %s", filename, strerror(errno));
        free(idx_entries);
        free(audio_drain_buf);
        if (write_task_ready) xSemaphoreGive(write_task_ready);
        vTaskDelete(NULL);
        return;
    }

    // Pre-allocate file to expected max size.
    // This allocates FAT clusters upfront so writes during recording
    // don't trigger cluster allocation (the main FAT overhead).
    // Cap at 2GB (FAT32 max is 4GB, but stay safe)
    size_t prealloc_bytes = (size_t)RECORD_DURATION_SEC * (H264_BITRATE / 8) * 2; // 2x bitrate headroom
    if (prealloc_bytes > 2000000000UL) prealloc_bytes = 2000000000UL;
    int64_t t_pa = esp_timer_get_time();
    if (lseek(fd, (off_t)prealloc_bytes, SEEK_SET) >= 0) {
        uint8_t zero = 0;
        write(fd, &zero, 1);
        fsync(fd);
        lseek(fd, 0, SEEK_SET);
        ESP_LOGI(TAG, "Pre-allocated %zuMB in %lldms",
                 prealloc_bytes / (1024 * 1024),
                 (esp_timer_get_time() - t_pa) / 1000);
    }

    // Staging buffer: PSRAM->internal bounce for POSIX write().
    // Merges AVI header + frame data so each write() is from internal DMA RAM.
    // With merged header, a 64KB staging buffer fits a full ~33KB H.264 frame
    // in one write() call. Falls back to 32KB (2 write() calls per frame).
    size_t stg_size = 64 * 1024;
    uint8_t *stg_buf = heap_caps_aligned_alloc(128, stg_size,
                           MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!stg_buf) {
        stg_size = 32 * 1024;
        stg_buf = heap_caps_aligned_alloc(128, stg_size,
                       MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    }
    if (!stg_buf) {
        stg_size = 16 * 1024;
        stg_buf = heap_caps_aligned_alloc(128, stg_size,
                       MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    }
    if (!stg_buf) {
        ESP_LOGW(TAG, "No staging buffer - large writes will be slow");
        stg_size = 0;
    } else {
        ESP_LOGI(TAG, "Staging buffer: %zuKB INTERNAL DMA", stg_size / 1024);
    }

    ESP_LOGI(TAG, "AVI writer started: %s", filename);
    ESP_LOGI(TAG, "Internal DMA free: %lu bytes, PSRAM free: %lu bytes",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    // Write AVI header (uses encoder dimensions, not sensor dimensions)
    uint32_t movi_start = avi_write_header(fd, recorder->enc_width, recorder->enc_height, AVI_FPS);
    uint32_t current_pos = movi_start + 4; // first data byte is 4 bytes after 'movi' fourcc
    ESP_LOGI(TAG, "AVI header written");

    // Signal that write task is ready for frames
    if (write_task_ready) xSemaphoreGive(write_task_ready);

    while (1) {
        if (xQueueReceive(write_queue, &msg, pdMS_TO_TICKS(500)) != pdTRUE) {
            if (!capture_running) break;
            // Even when no video frame, drain audio
            if (audio_ring && idx_count < max_idx) {
                uint32_t audio_bytes = audio_drain(audio_drain_buf, audio_chunk_max);
                if (audio_bytes > 0) {
                    if (idx_count < max_idx) {
                        memcpy(idx_entries[idx_count].fourcc, "01wb", 4);
                        idx_entries[idx_count].flags = 0;
                        idx_entries[idx_count].offset = current_pos - movi_start;
                        idx_entries[idx_count].size = audio_bytes;
                        idx_count++;
                    }
                    avi_write_audio(fd, audio_drain_buf, audio_bytes, stg_buf, stg_size);
                    uint32_t audio_samples = audio_bytes / (AUDIO_BITS / 8 * AUDIO_CHANNELS);
                    total_audio_samples += audio_samples;
                    current_pos += 8 + audio_bytes + (audio_bytes & 1 ? 1 : 0);
                    audio_chunks_written++;
                }
            }
            continue;
        }

        if ((uint32_t)written >= max_frames || idx_count >= max_idx - 1) {
            // Release staging buffer back to capture task
            xSemaphoreGive(jpeg_buf_sem);
            continue;
        }

        // Write video frame
        int64_t t_wr_start = esp_timer_get_time();
        if (idx_count < max_idx) {
            memcpy(idx_entries[idx_count].fourcc, "00dc", 4);
            idx_entries[idx_count].flags = msg.is_keyframe ? 0x10 : 0;
            idx_entries[idx_count].offset = current_pos - movi_start;
            idx_entries[idx_count].size = msg.jpeg_size;
            idx_count++;
        }
        avi_write_frame(fd, msg.jpeg_data, msg.jpeg_size, stg_buf, stg_size);
        int64_t t_vidwr_us = esp_timer_get_time() - t_wr_start;

        // Release staging buffer back to capture task for reuse
        xSemaphoreGive(jpeg_buf_sem);

        current_pos += 8 + msg.jpeg_size + (msg.jpeg_size & 1 ? 1 : 0);
        written++;
        total_written = written;

        // Drain audio accumulated since last video frame
        int64_t t_aud_start = esp_timer_get_time();
        int64_t t_audwr_us = 0;
        if (audio_ring && idx_count < max_idx) {
            uint32_t audio_bytes = audio_drain(audio_drain_buf, audio_chunk_max);
            if (audio_bytes > 0) {
                if (idx_count < max_idx) {
                    memcpy(idx_entries[idx_count].fourcc, "01wb", 4);
                    idx_entries[idx_count].flags = 0;
                    idx_entries[idx_count].offset = current_pos - movi_start;
                    idx_entries[idx_count].size = audio_bytes;
                    idx_count++;
                }
                avi_write_audio(fd, audio_drain_buf, audio_bytes, stg_buf, stg_size);
                uint32_t audio_samples = audio_bytes / (AUDIO_BITS / 8 * AUDIO_CHANNELS);
                total_audio_samples += audio_samples;
                current_pos += 8 + audio_bytes + (audio_bytes & 1 ? 1 : 0);
                audio_chunks_written++;
            }
        }
        t_audwr_us = esp_timer_get_time() - t_aud_start;

        // Accumulate write task timing
        static int64_t wt_vid_total = 0, wt_aud_total = 0, wt_sync_total = 0;
        static int wt_count = 0;
        wt_vid_total += t_vidwr_us;
        wt_aud_total += t_audwr_us;
        wt_count++;

        if (written % 150 == 0) {
            int64_t t_sync_start = esp_timer_get_time();

            float elapsed = (esp_timer_get_time() - start_us) / 1e6f;
            uint32_t cur_fps = elapsed > 0 ? (uint32_t)(written / elapsed + 0.5f) : AVI_FPS;
            if (cur_fps < 1) cur_fps = 1;

            // Update AVI headers in-place so file is playable if recording is interrupted
            avi_update_headers(fd, movi_start, current_pos, written, total_audio_samples, cur_fps);
            fsync(fd);  // force to SD card so file survives power loss

            int64_t t_sync_us = esp_timer_get_time() - t_sync_start;
            wt_sync_total += t_sync_us;

            ESP_LOGI(TAG, "WR_TIMING avg(ms): vid=%.1f aud=%.1f sync=%.1f [%d frames] last_sync=%lldms",
                     wt_vid_total / 1000.0f / wt_count,
                     wt_aud_total / 1000.0f / wt_count,
                     wt_sync_total / 1000.0f / (written / 150),
                     wt_count, t_sync_us / 1000);
            if (awf_count > 0) {
                ESP_LOGI(TAG, "  AWF breakdown: memcpy=%.1f write=%.1f (ms avg, %d calls)",
                         awf_memcpy_total_us / 1000.0f / awf_count,
                         awf_write_total_us / 1000.0f / awf_count,
                         awf_count);
            }
            ESP_LOGI(TAG, "Written %d frames (%.1f FPS avg), captured=%d, dropped=%d, audio=%d chunks",
                     written, written / elapsed, total_captured, total_dropped, audio_chunks_written);


        }
    }

    // Drain remaining audio
    if (audio_ring && idx_count < max_idx) {
        uint32_t audio_bytes = audio_drain(audio_drain_buf, audio_chunk_max);
        if (audio_bytes > 0 && idx_count < max_idx) {
            memcpy(idx_entries[idx_count].fourcc, "01wb", 4);
            idx_entries[idx_count].flags = 0;
            idx_entries[idx_count].offset = current_pos - movi_start;
            idx_entries[idx_count].size = audio_bytes;
            idx_count++;
            avi_write_audio(fd, audio_drain_buf, audio_bytes, stg_buf, stg_size);
            uint32_t audio_samples = audio_bytes / (AUDIO_BITS / 8 * AUDIO_CHANNELS);
            total_audio_samples += audio_samples;
            current_pos += 8 + audio_bytes + (audio_bytes & 1 ? 1 : 0);
            audio_chunks_written++;
        }
    }

    // Finalize AVI (update headers with actual FPS, write index)
    float rec_elapsed = (esp_timer_get_time() - start_us) / 1e6f;
    uint32_t actual_fps = rec_elapsed > 0 ? (uint32_t)(written / rec_elapsed + 0.5f) : AVI_FPS;
    if (actual_fps < 1) actual_fps = 1;
    ESP_LOGI(TAG, "Finalizing AVI: %d video frames, %d audio chunks (%lu samples), actual %lu FPS",
             written, audio_chunks_written, (unsigned long)total_audio_samples, (unsigned long)actual_fps);
    avi_finalize(fd, movi_start, written, total_audio_samples,
                 idx_entries, idx_count,
                 recorder->enc_width, recorder->enc_height, actual_fps);
    off_t final_size = lseek(fd, 0, SEEK_CUR);
    ESP_LOGI(TAG, "AVI data size: %ld bytes (%.1f MB)", (long)final_size, final_size / (1024.0f * 1024.0f));
    fsync(fd);
    // Truncate pre-allocated file to actual data size
    if (ftruncate(fd, final_size) != 0) {
        ESP_LOGE(TAG, "ftruncate failed: %s", strerror(errno));
    }
    fsync(fd);  // sync the truncated size
    ESP_LOGI(TAG, "AVI finalized, closing file");
    close(fd);

    // Verify file size on disk
    struct stat st;
    if (stat(filename, &st) == 0) {
        ESP_LOGI(TAG, "File on disk: %ld bytes", (long)st.st_size);
    }
    if (stg_buf) free(stg_buf);
    free(idx_entries);
    free(audio_drain_buf);

    float elapsed = (esp_timer_get_time() - start_us) / 1e6f;
    ESP_LOGI(TAG, "=== RECORDING COMPLETE ===");
    ESP_LOGI(TAG, "  File: %s", filename);
    ESP_LOGI(TAG, "  Video: %d frames in %.1fs = %.1f FPS", written, elapsed,
             elapsed > 0 ? written / elapsed : 0);
    ESP_LOGI(TAG, "  Audio: %d chunks, %lu samples", audio_chunks_written, (unsigned long)total_audio_samples);
    ESP_LOGI(TAG, "  Captured: %d, Dropped: %d", total_captured, total_dropped);
    ESP_LOGI(TAG, "  Heap: %lu free", (unsigned long)esp_get_free_heap_size());

    gpio_set_level(LED_PIN, 0);
    led_blink(5, 150);

    sd_write_task_handle = NULL;
    vTaskDelete(NULL);
}

// ============================================================================
// H.264 KEYFRAME DETECTION
// ============================================================================

// Check if H.264 Annex-B frame contains an IDR NAL unit (keyframe)
static bool h264_is_keyframe(const uint8_t *data, uint32_t size)
{
    uint32_t limit = size < 128 ? size : 128;
    for (uint32_t i = 0; i + 4 < limit; i++) {
        if (data[i] == 0 && data[i+1] == 0 && data[i+2] == 0 && data[i+3] == 1) {
            uint8_t nal_type = data[i + 4] & 0x1F;
            if (nal_type == 5) return true;  // IDR slice
            i += 4;
        }
    }
    return false;
}

// ============================================================================
// CAPTURE + ENCODE TASK - runs on Core 0
// Uses V4L2 M2M H.264 device (follows sd_card example pattern)
// ============================================================================

static void capture_task(void *arg)
{
    int64_t start_us = esp_timer_get_time();
    int64_t end_us = start_us + (int64_t)RECORD_DURATION_SEC * 1000000LL;
    int captured = 0;
    int dropped = 0;
    int pre_enc_drops = 0;    // frames skipped BEFORE encoding (backpressure)
    int post_enc_drops = 0;   // frames dropped AFTER encoding (reference break!)
    int consecutive_dqbuf_fails = 0;
    int64_t last_csi_check_us = 0;

    // Per-stage timing accumulators (microseconds)
    int64_t t_dqbuf_total = 0, t_ppa_total = 0, t_enc_total = 0, t_copy_total = 0, t_sem_total = 0;
    int timing_count = 0;

    ESP_LOGI(TAG, "Capture: starting %d second recording", RECORD_DURATION_SEC);
    capture_running = true;
    gpio_set_level(LED_PIN, 1);

    while (esp_timer_get_time() < end_us) {
        struct v4l2_buffer cap_buf;
        struct v4l2_buffer m2m_out_buf;
        struct v4l2_buffer m2m_cap_buf;

        // Periodic CSI health check: detect pkt_fatal and attempt recovery
        int64_t now_check = esp_timer_get_time();
        if (now_check - last_csi_check_us > 2000000) {  // every 2 seconds
            last_csi_check_us = now_check;
            uint32_t pkt_fatal = MIPI_CSI_HOST.int_st_pkt_fatal.val;
            uint32_t phy_fatal = MIPI_CSI_HOST.int_st_phy_fatal.val;
            if (pkt_fatal || phy_fatal) {
                ESP_LOGW(TAG, "CSI errors detected: pkt_fatal=0x%lx phy_fatal=0x%lx (cap=%d) — attempting STREAMOFF/ON recovery",
                         (unsigned long)pkt_fatal, (unsigned long)phy_fatal, captured);
                // STREAMOFF/STREAMON cycle to reset CSI/DMA pipeline
                int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                ioctl(recorder->cap_fd, VIDIOC_STREAMOFF, &type);
                vTaskDelay(pdMS_TO_TICKS(50));
                // Re-queue all capture buffers
                for (int i = 0; i < NUM_CAP_BUFFERS; i++) {
                    struct v4l2_buffer rebuf = {
                        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
                        .memory = V4L2_MEMORY_MMAP,
                        .index = i,
                    };
                    ioctl(recorder->cap_fd, VIDIOC_QBUF, &rebuf);
                }
                ioctl(recorder->cap_fd, VIDIOC_STREAMON, &type);
                ESP_LOGI(TAG, "CSI recovery: STREAMOFF/ON complete, resuming capture");
            }
        }

        // DQBUF from camera - blocks until frame ready
        int64_t t_stage = esp_timer_get_time();
        memset(&cap_buf, 0, sizeof(cap_buf));
        cap_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        cap_buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(recorder->cap_fd, VIDIOC_DQBUF, &cap_buf) != 0) {
            consecutive_dqbuf_fails++;
            ESP_LOGE(TAG, "DQBUF fail: %s (consecutive=%d)", strerror(errno), consecutive_dqbuf_fails);
            if (consecutive_dqbuf_fails >= 10) {
                ESP_LOGE(TAG, "Too many consecutive DQBUF failures, dumping CSI diagnostics");
                dump_csi_diagnostics();
                consecutive_dqbuf_fails = 0;
            }
            vTaskDelay(1);
            continue;
        }
        consecutive_dqbuf_fails = 0;

        captured++;
        total_captured = captured;
        int64_t t_dqbuf_us = esp_timer_get_time() - t_stage;

        // --- BACKPRESSURE CHECK: skip encoding if staging buffers are exhausted ---
        // Dropping BEFORE encode preserves H.264 reference chain integrity.
        // Dropping AFTER encode causes P-frame corruption (macroblock smearing).
        if (uxSemaphoreGetCount(jpeg_buf_sem) == 0) {
            ioctl(recorder->cap_fd, VIDIOC_QBUF, &cap_buf);
            pre_enc_drops++;
            dropped++;
            total_dropped = dropped;
            if (pre_enc_drops % 10 == 1) {
                ESP_LOGW(TAG, "Backpressure: skipped frame %d before encoding (pre_drops=%d)",
                         captured, pre_enc_drops);
            }
            continue;
        }

        // NOTE: V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME is not supported by esp_video H.264 driver.
        // GOP=15 ensures natural IDR frames every 0.5s to limit P-frame corruption duration.
        // If the driver adds support in the future, uncomment the force_idr block above.

        // Feed raw frame to H.264 M2M encoder via USERPTR
        // If PPA scaling is active, scale 2304x1296 -> 1280x720 first
        uint8_t *enc_input_buf;
        uint32_t enc_input_len;

        int64_t t_ppa_us = 0;
        if (ppa_srm_handle && ppa_scaled_buf) {
            int64_t t_ppa_start = esp_timer_get_time();
            ppa_srm_oper_config_t srm = {
                .in = {
                    .buffer = recorder->cap_buffer[cap_buf.index],
                    .pic_w = recorder->width,
                    .pic_h = recorder->height,
                    .block_w = recorder->width,
                    .block_h = recorder->height,
                    .block_offset_x = 0,
                    .block_offset_y = 0,
                    .srm_cm = PPA_SRM_COLOR_MODE_YUV420,
                    .yuv_range = PPA_COLOR_RANGE_LIMIT,
                    .yuv_std = PPA_COLOR_CONV_STD_RGB_YUV_BT601,
                },
                .out = {
                    .buffer = ppa_scaled_buf,
                    .buffer_size = ppa_scaled_buf_size,
                    .pic_w = recorder->enc_width,
                    .pic_h = recorder->enc_height,
                    .block_offset_x = 0,
                    .block_offset_y = 0,
                    .srm_cm = PPA_SRM_COLOR_MODE_YUV420,
                    .yuv_range = PPA_COLOR_RANGE_LIMIT,
                    .yuv_std = PPA_COLOR_CONV_STD_RGB_YUV_BT601,
                },
                .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
                .scale_x = (float)recorder->enc_width / (float)recorder->width,
                .scale_y = (float)recorder->enc_height / (float)recorder->height,
                .mode = PPA_TRANS_MODE_BLOCKING,
            };
            esp_err_t ppa_ret = ppa_do_scale_rotate_mirror(ppa_srm_handle, &srm);
            if (ppa_ret != ESP_OK) {
                ESP_LOGE(TAG, "PPA scale failed: 0x%x", ppa_ret);
                ioctl(recorder->cap_fd, VIDIOC_QBUF, &cap_buf);
                continue;
            }
            enc_input_buf = ppa_scaled_buf;
            enc_input_len = ppa_scaled_buf_size;
            t_ppa_us = esp_timer_get_time() - t_ppa_start;
        } else {
            enc_input_buf = recorder->cap_buffer[cap_buf.index];
            enc_input_len = cap_buf.bytesused;
        }

        // --- H.264 encode ---
        int64_t t_enc_start = esp_timer_get_time();
        memset(&m2m_out_buf, 0, sizeof(m2m_out_buf));
        m2m_out_buf.index = 0;
        m2m_out_buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        m2m_out_buf.memory = V4L2_MEMORY_USERPTR;
        m2m_out_buf.m.userptr = (unsigned long)enc_input_buf;
        m2m_out_buf.length = enc_input_len;

        if (ioctl(recorder->m2m_fd, VIDIOC_QBUF, &m2m_out_buf) != 0) {
            ESP_LOGE(TAG, "M2M QBUF output fail: %s", strerror(errno));
            ioctl(recorder->cap_fd, VIDIOC_QBUF, &cap_buf);
            continue;
        }
        // DQBUF encoded H.264 from M2M capture
        memset(&m2m_cap_buf, 0, sizeof(m2m_cap_buf));
        m2m_cap_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        m2m_cap_buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(recorder->m2m_fd, VIDIOC_DQBUF, &m2m_cap_buf) != 0) {
            ESP_LOGE(TAG, "M2M DQBUF capture fail: %s", strerror(errno));
            ioctl(recorder->cap_fd, VIDIOC_QBUF, &cap_buf);
            continue;
        }
        int64_t t_enc_us = esp_timer_get_time() - t_enc_start;

        // --- Gap: camera QBUF + M2M output DQBUF ---
        // Release camera buffer ASAP so ISR has buffers
        ioctl(recorder->cap_fd, VIDIOC_QBUF, &cap_buf);

        // Dequeue the M2M output buffer (required to complete the cycle)
        ioctl(recorder->m2m_fd, VIDIOC_DQBUF, &m2m_out_buf);

        // --- Copy to staging buffer ---
        uint32_t jpeg_size = m2m_cap_buf.bytesused;
        int m2m_idx = m2m_cap_buf.index;

        if (jpeg_size > 0 && jpeg_size <= JPEG_BUF_SIZE) {
            // Wait for a free staging buffer (blocks if write task is behind)
            int64_t t_sem_start = esp_timer_get_time();
            if (xSemaphoreTake(jpeg_buf_sem, pdMS_TO_TICKS(200)) == pdTRUE) {
                int64_t t_sem_us = esp_timer_get_time() - t_sem_start;
                int buf_idx = recorder->jpeg_buf_write_idx;
                int64_t t_copy_start = esp_timer_get_time();
                memcpy(recorder->jpeg_out_buf[buf_idx], recorder->m2m_cap_buffers[m2m_idx], jpeg_size);
                int64_t t_copy_us = esp_timer_get_time() - t_copy_start;
                recorder->jpeg_buf_write_idx = (buf_idx + 1) % NUM_JPEG_BUFS;

                // Re-queue M2M buffer immediately (don't wait for SD write)
                m2m_cap_buf.index = m2m_idx;
                m2m_cap_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                m2m_cap_buf.memory = V4L2_MEMORY_MMAP;
                ioctl(recorder->m2m_fd, VIDIOC_QBUF, &m2m_cap_buf);

                frame_msg_t msg = {
                    .jpeg_data = recorder->jpeg_out_buf[buf_idx],
                    .jpeg_size = jpeg_size,
                    .seq = captured,
                    .is_keyframe = h264_is_keyframe(recorder->jpeg_out_buf[buf_idx], jpeg_size),
                };
                xQueueSend(write_queue, &msg, portMAX_DELAY);

                // Accumulate timing stats
                t_dqbuf_total += t_dqbuf_us;
                t_ppa_total += t_ppa_us;
                t_enc_total += t_enc_us;
                t_copy_total += t_copy_us;
                t_sem_total += t_sem_us;
                timing_count++;
                if (timing_count % 50 == 0) {
                    ESP_LOGI(TAG, "TIMING avg(ms): dqbuf=%.1f ppa=%.1f h264=%.1f sem=%.1f copy=%.1f total=%.1f [%d frames]",
                             t_dqbuf_total / 1000.0f / timing_count,
                             t_ppa_total / 1000.0f / timing_count,
                             t_enc_total / 1000.0f / timing_count,
                             t_sem_total / 1000.0f / timing_count,
                             t_copy_total / 1000.0f / timing_count,
                             (t_dqbuf_total + t_ppa_total + t_enc_total + t_sem_total + t_copy_total) / 1000.0f / timing_count,
                             timing_count);
                }

            } else {
                // All staging buffers full AFTER encoding — this breaks H.264 references!
                // Should rarely happen if pre-encode backpressure is working.
                m2m_cap_buf.index = m2m_idx;
                m2m_cap_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                m2m_cap_buf.memory = V4L2_MEMORY_MMAP;
                ioctl(recorder->m2m_fd, VIDIOC_QBUF, &m2m_cap_buf);
                post_enc_drops++;
                dropped++;
                total_dropped = dropped;
                ESP_LOGW(TAG, "POST-ENCODE drop #%d at frame %d — H.264 reference chain broken!",
                         post_enc_drops, captured);
            }
        } else {
            // Empty or oversized frame — re-queue M2M buffer
            m2m_cap_buf.index = m2m_idx;
            m2m_cap_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            m2m_cap_buf.memory = V4L2_MEMORY_MMAP;
            ioctl(recorder->m2m_fd, VIDIOC_QBUF, &m2m_cap_buf);
        }
    }

    capture_running = false;
    recording = false;

    float elapsed = (esp_timer_get_time() - start_us) / 1e6f;
    ESP_LOGI(TAG, "Capture done: %d frames in %.1fs (%.1f FPS), dropped=%d (pre_enc=%d, post_enc=%d)",
             captured, elapsed, elapsed > 0 ? captured / elapsed : 0,
             dropped, pre_enc_drops, post_enc_drops);

    capture_task_handle = NULL;
    vTaskDelete(NULL);
}

// ============================================================================
// FIRST-FRAME TEST with timeout (runs DQBUF in a separate task)
// ============================================================================

typedef struct {
    int fd;
    SemaphoreHandle_t done_sem;
    bool success;
    uint32_t bytesused;
} dqbuf_test_t;

static void dqbuf_test_task(void *arg)
{
    dqbuf_test_t *test = (dqbuf_test_t *)arg;
    struct v4l2_buffer buf = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(test->fd, VIDIOC_DQBUF, &buf) == 0) {
        test->success = true;
        test->bytesused = buf.bytesused;
        // Re-queue the buffer
        ioctl(test->fd, VIDIOC_QBUF, &buf);
    }
    xSemaphoreGive(test->done_sem);
    vTaskDelete(NULL);
}

static bool try_dqbuf_with_timeout(int fd, uint32_t timeout_ms, uint32_t *out_bytesused)
{
    dqbuf_test_t test = {
        .fd = fd,
        .done_sem = xSemaphoreCreateBinary(),
        .success = false,
        .bytesused = 0,
    };

    TaskHandle_t task_h = NULL;
    xTaskCreatePinnedToCore(dqbuf_test_task, "dqtest", 4096, &test, 5, &task_h, 0);

    bool got_frame = (xSemaphoreTake(test.done_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE && test.success);

    if (!got_frame && task_h != NULL) {
        // Task is still blocked in DQBUF - must delete it before cleaning up
        vTaskDelete(task_h);
    }

    vSemaphoreDelete(test.done_sem);

    if (got_frame && out_bytesused) {
        *out_bytesused = test.bytesused;
    }
    return got_frame;
}

// ============================================================================
// STREAMING START with DQBUF timeout diagnostic
// ============================================================================

static void stop_all_streams(void)
{
    int type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(recorder->cap_fd, VIDIOC_STREAMOFF, &type);
    type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    ioctl(recorder->m2m_fd, VIDIOC_STREAMOFF, &type);
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(recorder->m2m_fd, VIDIOC_STREAMOFF, &type);
}

static esp_err_t start_all_streams(void)
{
    int type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(recorder->m2m_fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "M2M STREAMON capture failed: %s", strerror(errno));
        return ESP_FAIL;
    }
    type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    if (ioctl(recorder->m2m_fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "M2M STREAMON output failed: %s", strerror(errno));
        return ESP_FAIL;
    }
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(recorder->cap_fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "Camera STREAMON failed: %s", strerror(errno));
        return ESP_FAIL;
    }
    return ESP_OK;
}

#define MIPI_LINK_MAX_RETRIES 3      // 1 initial (fast-fail) + 2 heavy LDO resets
#define MIPI_LIGHT_RETRIES   0      // Skip light I2C reset — it never recovers MIPI sync
#define IMX708_I2C_ADDR      0x1A
#define IMX708_REG_RESET     0x0103

// Light reset: direct I2C software reset of IMX708 + re-apply format (~250ms)
static esp_err_t light_sensor_reset(void)
{
    ESP_LOGI(TAG, "Light reset: I2C software reset of IMX708...");

    // 1. Stop all streams so ESP32-P4 CSI receiver stops choking
    stop_all_streams();
    vTaskDelay(pdMS_TO_TICKS(50));

    // 2. Direct I2C write: register 0x0103 = 0x01 (IMX708 software reset)
    //    This kills the MIPI transmission from the camera side and resets PLLs
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMX708_I2C_ADDR,
        .scl_speed_hz = CAM_I2C_FREQ,
    };
    i2c_master_dev_handle_t dev_handle = NULL;
    esp_err_t ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &dev_handle);
    if (ret == ESP_OK) {
        uint8_t write_buf[3] = {
            (IMX708_REG_RESET >> 8) & 0xFF,   // reg addr high byte (0x01)
            IMX708_REG_RESET & 0xFF,           // reg addr low byte  (0x03)
            0x01                                // value: trigger reset
        };
        ret = i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), 100);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "I2C reset write failed: 0x%x", ret);
        }
        i2c_master_bus_rm_device(dev_handle);
    } else {
        ESP_LOGW(TAG, "I2C add device failed: 0x%x", ret);
    }

    // 3. Wait for sensor PLLs to settle after reset
    vTaskDelay(pdMS_TO_TICKS(150));

    // 4. Re-apply format (pushes full 1080p register table back into IMX708)
    struct v4l2_format fmt = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
    fmt.fmt.pix.width = recorder->width;
    fmt.fmt.pix.height = recorder->height;
    fmt.fmt.pix.pixelformat = recorder->capture_fmt;
    fmt.fmt.pix.quantization = V4L2_QUANTIZATION_LIM_RANGE;
    fmt.fmt.pix.ycbcr_enc = V4L2_YCBCR_ENC_601;
    fmt.fmt.pix.colorspace = V4L2_COLORSPACE_SMPTE170M;
    if (ioctl(recorder->cap_fd, VIDIOC_S_FMT, &fmt) != 0) {
        ESP_LOGW(TAG, "S_FMT re-apply failed: %s", strerror(errno));
    }

    // 5. Re-apply ISP white balance
    init_isp_white_balance();

    return ESP_OK;
}

// Heavy reset: full teardown + LDO power cycle + reinit from scratch (~2s)
static esp_err_t heavy_pipeline_reset(void)
{
    ESP_LOGI(TAG, "Heavy reset: full teardown + LDO power cycle...");

    // 1. Kill all V4L2 streams
    stop_all_streams();
    vTaskDelay(pdMS_TO_TICKS(100));

    // 2. Tear down the entire video pipeline (close fds, munmap, free buffers)
    deinit_video_pipeline();
    vTaskDelay(pdMS_TO_TICKS(100));

    // 3. Destroy all esp_video devices (CSI, ISP, H264)
    esp_err_t dret = esp_video_deinit();
    if (dret != ESP_OK) {
        ESP_LOGW(TAG, "esp_video_deinit returned 0x%x (continuing anyway)", dret);
    }

    // 4. Hard power-cycle the sensor via LDO3
    ESP_LOGI(TAG, "Power-cycling camera LDO3 (sensor hard reset)...");
    esp_ldo_release_channel(ldo_mipi_handle);
    ldo_mipi_handle = NULL;
    vTaskDelay(pdMS_TO_TICKS(2000));  // 2s off: sensor needs full power-down for MIPI PHY + PLL reset
    esp_ldo_channel_config_t ldo3 = { .chan_id = 3, .voltage_mv = 2500 };
    esp_err_t lret = esp_ldo_acquire_channel(&ldo3, &ldo_mipi_handle);
    if (lret != ESP_OK) {
        ESP_LOGE(TAG, "LDO3 re-acquire failed: 0x%x", lret);
        return lret;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));  // 1s stabilization: wait for IMX708 oscillator + PLL lock

    // 5. Re-initialize esp_video from scratch (re-probes sensor, pushes 1080p regs)
    esp_err_t iret = init_esp_video();
    if (iret != ESP_OK) {
        ESP_LOGE(TAG, "init_esp_video failed on retry: 0x%x", iret);
        return iret;
    }

    // 6. Re-create the full pipeline (open devices, S_FMT, REQBUFS, mmap)
    iret = init_video_pipeline();
    if (iret != ESP_OK) {
        ESP_LOGE(TAG, "init_video_pipeline failed on retry: 0x%x", iret);
        return iret;
    }

    // 7. Re-apply ISP white balance
    if (init_isp_white_balance() != ESP_OK) {
        ESP_LOGW(TAG, "ISP WB re-init failed on retry (colors may be off)");
    }

    return ESP_OK;
}

static esp_err_t start_streaming(void)
{
    uint32_t first_bytes = 0;
    bool got_frame = false;

    for (int attempt = 0; attempt < MIPI_LINK_MAX_RETRIES; attempt++) {
        if (attempt > 0) {
            if (attempt <= MIPI_LIGHT_RETRIES) {
                // Tier 1: Light I2C reset (~250ms) — fast recovery
                ESP_LOGW(TAG, "=== LIGHT RESET %d/%d (I2C sensor reset) ===",
                         attempt, MIPI_LIGHT_RETRIES);
                light_sensor_reset();
            } else {
                // Tier 2: Heavy nuke-and-pave with LDO power cycle (~2s)
                ESP_LOGW(TAG, "=== HEAVY RESET %d/%d (LDO power cycle) ===",
                         attempt - MIPI_LIGHT_RETRIES,
                         MIPI_LINK_MAX_RETRIES - MIPI_LIGHT_RETRIES - 1);
                esp_err_t hret = heavy_pipeline_reset();
                if (hret != ESP_OK) return hret;
            }

            ESP_LOGI(TAG, "Reset complete. Starting streams...");
        }

        // Guarantee LP-11: force sensor stream-off via I2C before ESP CSI STREAMON.
        // init_esp_video() re-probes the sensor which may leave it streaming.
        {
            i2c_device_config_t cam_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address  = IMX708_I2C_ADDR,
                .scl_speed_hz    = CAM_I2C_FREQ,
            };
            i2c_master_dev_handle_t cam_dev = NULL;
            if (i2c_master_bus_add_device(i2c_bus_handle, &cam_cfg, &cam_dev) == ESP_OK) {
                uint8_t stream_off[] = { 0x01, 0x00, 0x00 }; // reg 0x0100 = 0x00
                esp_err_t tx_ret = i2c_master_transmit(cam_dev, stream_off, sizeof(stream_off), 100);
                if (tx_ret != ESP_OK) {
                    ESP_LOGW(TAG, "LP-11 stream-off I2C failed (0x%x), resetting bus...", tx_ret);
                    i2c_master_bus_reset(i2c_bus_handle);
                    vTaskDelay(pdMS_TO_TICKS(50));
                    i2c_master_transmit(cam_dev, stream_off, sizeof(stream_off), 100);
                }
                i2c_master_bus_rm_device(cam_dev);
                ESP_LOGI(TAG, "Sensor stream-off sent, waiting 250ms for LP-11...");
                vTaskDelay(pdMS_TO_TICKS(250));
            }
        }

        esp_err_t ret = start_all_streams();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "start_all_streams failed: 0x%x", ret);
            continue;
        }

        // Override CSI bridge afull_thrd: ESP-IDF hardcodes 960, but at 1920px width
        // half a line = 960, causing backpressure mid-line on every line → pkt_fatal.
        // Register default is 2040 (14-bit field, max 16383).
        {
            uint32_t old_thrd = MIPI_CSI_BRIDGE.buf_flow_ctl.csi_buf_afull_thrd;
            MIPI_CSI_BRIDGE.buf_flow_ctl.csi_buf_afull_thrd = 2040;
            ESP_LOGI(TAG, "CSI bridge afull_thrd: %lu -> 2040", (unsigned long)old_thrd);
        }

        // Log ISP frame dimensions and clock rate
        {
            uint32_t h = ISP.frame_cfg.hadr_num;
            uint32_t v = ISP.frame_cfg.vadr_num;
            uint32_t div = HP_SYS_CLKRST.peri_clk_ctrl26.reg_isp_clk_div_num;
            ESP_LOGI(TAG, "ISP frame_cfg: hadr=%lu vadr=%lu, clk_div=%lu (ISP clk ~%lu MHz)",
                     (unsigned long)h, (unsigned long)v,
                     (unsigned long)div, (unsigned long)(160 / (div + 1)));
        }

        int64_t t_streamon = esp_timer_get_time();

        /* Reset ISR counters BEFORE PHY poll so we don't see stale values */
        csi_isr_trans_finished_cnt = 0;
        csi_isr_get_new_trans_cnt = 0;
        csi_isr_frame_dropped_cnt = 0;
        csi_isr_queue_empty_cnt = 0;

        // ── PHY startup poll: sample PHY/BRG/ISP every 10ms for 100ms ──
        // Tells us if PHY ever transitions LP-11 → HS after sensor start.
        {
            // Clear ISP int_raw WTC bits so we can detect fresh events
            ISP.int_clr.val = (1 << 2) | (1 << 14) | (1 << 28);

            // Log host/bridge enable state at STREAMON
            ESP_LOGI(TAG, "CSI enable state: csi2_resetn=%d dphy_rstz=%d phy_shutdownz=%d"
                     " brg_en=%d enableclk=%d cfg_clk_en=%d"
                     " hadr=%lu vadr=%lu",
                     (int)MIPI_CSI_HOST.csi2_resetn.csi2_resetn,
                     (int)MIPI_CSI_HOST.dphy_rstz.dphy_rstz,
                     (int)MIPI_CSI_HOST.phy_shutdownz.phy_shutdownz,
                     (int)MIPI_CSI_BRIDGE.csi_en.csi_brg_en,
                     (int)MIPI_CSI_BRIDGE.host_ctrl.csi_enableclk,
                     (int)MIPI_CSI_BRIDGE.host_ctrl.csi_cfg_clk_en,
                     (unsigned long)MIPI_CSI_BRIDGE.frame_cfg.hadr_num,
                     (unsigned long)MIPI_CSI_BRIDGE.frame_cfg.vadr_num);

            ESP_LOGI(TAG, "PHY poll (T+ms | stop_d0 stop_d1 stop_clk | clk_hs | buf_depth | isp_raw):");
            for (int p = 0; p <= 10; p++) {
                int64_t t_now = esp_timer_get_time();
                int dt_ms = (int)((t_now - t_streamon) / 1000);
                uint32_t stop_d0  = MIPI_CSI_HOST.phy_stopstate.phy_stopstatedata_0;
                uint32_t stop_d1  = MIPI_CSI_HOST.phy_stopstate.phy_stopstatedata_1;
                uint32_t stop_clk = MIPI_CSI_HOST.phy_stopstate.phy_stopstateclk;
                uint32_t clk_hs   = MIPI_CSI_HOST.phy_rx.phy_rxclkactivehs;
                uint32_t buf_dep  = MIPI_CSI_BRIDGE.buf_flow_ctl.csi_buf_depth;
                uint32_t isp_raw  = ISP.int_raw.val;
                uint32_t isr_new  = csi_isr_get_new_trans_cnt;
                ESP_LOGI(TAG, "  T+%3d | %lu %lu %lu | hs=%lu | dep=%lu | isp=0x%08lx | isr_new=%lu",
                         dt_ms,
                         (unsigned long)stop_d0, (unsigned long)stop_d1, (unsigned long)stop_clk,
                         (unsigned long)clk_hs, (unsigned long)buf_dep,
                         (unsigned long)isp_raw, (unsigned long)isr_new);
                // Stop polling early if we see HS activity or a frame ISR
                if (clk_hs || buf_dep > 0 || isr_new > 0) {
                    ESP_LOGI(TAG, "  PHY entered HS or data arrived at T+%d ms!", dt_ms);
                    break;
                }
                if (p < 10) vTaskDelay(pdMS_TO_TICKS(10));
            }
        }

        // First attempt always fails on cold boot (ESP32-P4 CSI PHY needs one
        // full create/use/delete cycle). Use 500ms timeout to fail fast.
        uint32_t dqbuf_timeout = (attempt == 0) ? 500 : 3000;
        ESP_LOGI(TAG, "Streaming ON (attempt %d) - waiting for first MIPI frame (%lums)...",
                 attempt + 1, (unsigned long)dqbuf_timeout);

        got_frame = try_dqbuf_with_timeout(recorder->cap_fd, dqbuf_timeout, &first_bytes);

        // ── PHY state at timeout ──
        {
            int64_t t_now = esp_timer_get_time();
            int dt_ms = (int)((t_now - t_streamon) / 1000);
            ESP_LOGI(TAG, "PHY at T+%d (after dqbuf): stop=%d/%d/%d hs=%d dep=%lu isp=0x%08lx",
                     dt_ms,
                     (int)MIPI_CSI_HOST.phy_stopstate.phy_stopstatedata_0,
                     (int)MIPI_CSI_HOST.phy_stopstate.phy_stopstatedata_1,
                     (int)MIPI_CSI_HOST.phy_stopstate.phy_stopstateclk,
                     (int)MIPI_CSI_HOST.phy_rx.phy_rxclkactivehs,
                     (unsigned long)MIPI_CSI_BRIDGE.buf_flow_ctl.csi_buf_depth,
                     (unsigned long)ISP.int_raw.val);
            // Log host/bridge enable state at timeout (detect if reset dropped)
            ESP_LOGI(TAG, "CSI at T+%d: resetn=%d dphy=%d shutdn=%d brg=%d eclk=%d cclk=%d",
                     dt_ms,
                     (int)MIPI_CSI_HOST.csi2_resetn.csi2_resetn,
                     (int)MIPI_CSI_HOST.dphy_rstz.dphy_rstz,
                     (int)MIPI_CSI_HOST.phy_shutdownz.phy_shutdownz,
                     (int)MIPI_CSI_BRIDGE.csi_en.csi_brg_en,
                     (int)MIPI_CSI_BRIDGE.host_ctrl.csi_enableclk,
                     (int)MIPI_CSI_BRIDGE.host_ctrl.csi_cfg_clk_en);
        }

        ESP_LOGI(TAG, "ISR counters: finished=%lu get_new=%lu dropped=%lu q_empty=%lu",
                 (unsigned long)csi_isr_trans_finished_cnt,
                 (unsigned long)csi_isr_get_new_trans_cnt,
                 (unsigned long)csi_isr_frame_dropped_cnt,
                 (unsigned long)csi_isr_queue_empty_cnt);

        if (got_frame) {
            break;
        }

        ESP_LOGW(TAG, "No frame received on attempt %d", attempt + 1);
    }

    if (!got_frame) {
        ESP_LOGE(TAG, "==============================================================");
        ESP_LOGE(TAG, "TIMEOUT: No MIPI frame after %d attempts (%d light + %d heavy)!",
                 MIPI_LINK_MAX_RETRIES, MIPI_LIGHT_RETRIES,
                 MIPI_LINK_MAX_RETRIES - MIPI_LIGHT_RETRIES - 1);
        ESP_LOGE(TAG, "==============================================================");

        // CRITICAL: Power off sensor to prevent overheating!
        // The sensor stays powered via LDO3 and generates heat even when
        // the MIPI link isn't established.
        ESP_LOGW(TAG, "Powering off sensor to prevent overheating...");
        stop_all_streams();
        deinit_video_pipeline();
        esp_video_deinit();
        if (ldo_mipi_handle) {
            esp_ldo_release_channel(ldo_mipi_handle);
            ldo_mipi_handle = NULL;
        }
        ESP_LOGW(TAG, "Sensor powered off.");
        return ESP_ERR_TIMEOUT;
    }

    if (first_bytes > 0) {  // suppress log if got_frame from retry path already logged
        ESP_LOGI(TAG, "First frame received! (%" PRIu32 " bytes) - MIPI link OK", first_bytes);
    }

    // Skip startup frames for ISP AEC/AGC/AWB settling.
    // IMX708 has 1.12x minimum analog gain floor - the ISP needs time to converge
    // its histogram-based exposure algorithm. At 30fps, 15 frames ≈ 500ms.
    ESP_LOGI(TAG, "Waiting %d frames for AEC/AGC settling...", SKIP_STARTUP_FRAMES - 1);
    for (int i = 0; i < SKIP_STARTUP_FRAMES - 1; i++) {
        ESP_LOGI(TAG, "AEC skip frame %d/%d - DQBUF... (ISR: fin=%lu new=%lu drop=%lu qe=%lu)",
                 i + 1, SKIP_STARTUP_FRAMES - 1,
                 (unsigned long)csi_isr_trans_finished_cnt,
                 (unsigned long)csi_isr_get_new_trans_cnt,
                 (unsigned long)csi_isr_frame_dropped_cnt,
                 (unsigned long)csi_isr_queue_empty_cnt);

        // Use timeout instead of blocking forever - dump diagnostics if hung
        bool aec_got_frame = try_dqbuf_with_timeout(recorder->cap_fd, 1000, NULL);
        if (!aec_got_frame) {
            ESP_LOGE(TAG, "AEC frame %d HUNG after 1000ms! Dumping diagnostics...", i + 1);
            dump_csi_diagnostics();

            // Extended GDMA/PHY sampling: 20 iterations × 100ms = 2s of continuous monitoring
            ESP_LOGW(TAG, "=== EXTENDED GDMA/PHY SAMPLING (20 × 100ms) ===");
            uint32_t prev_fifo = 0, prev_dep = 0, prev_fin = csi_isr_trans_finished_cnt;
            for (int s = 0; s < 20; s++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                uint32_t dep = MIPI_CSI_BRIDGE.buf_flow_ctl.csi_buf_depth;
                uint32_t fin = csi_isr_trans_finished_cnt;
                uint32_t newt = csi_isr_get_new_trans_cnt;
                uint32_t drop = csi_isr_frame_dropped_cnt;
                uint32_t qe = csi_isr_queue_empty_cnt;
                uint32_t chen = DW_GDMA.chen0.val;
                uint32_t fifo = 0, blk = 0, ch_int = 0;
                for (int c = 0; c < 4; c++) {
                    if (chen & (1 << c)) {
                        fifo = DW_GDMA.ch[c].status1.data_left_in_fifo;
                        blk = DW_GDMA.ch[c].status0.cmpltd_blk_tfr_size;
                        ch_int = DW_GDMA.ch[c].int_st0.val;
                        break;
                    }
                }
                uint32_t pkt_fatal = MIPI_CSI_HOST.int_st_pkt_fatal.val;
                uint32_t phy_fatal = MIPI_CSI_HOST.int_st_phy_fatal.val;
                uint32_t hs = MIPI_CSI_HOST.phy_rx.phy_rxclkactivehs;
                ESP_LOGW(TAG, "  [%2d] dep=%3lu fifo=%3lu(d%+ld) blk=%lu ch_int=0x%04lx fin=%lu new=%lu drop=%lu qe=%lu hs=%lu pkt=%lu phy=%lu",
                         s, (unsigned long)dep, (unsigned long)fifo, (long)(fifo - prev_fifo),
                         (unsigned long)blk, (unsigned long)ch_int,
                         (unsigned long)fin, (unsigned long)newt,
                         (unsigned long)drop, (unsigned long)qe,
                         (unsigned long)hs, (unsigned long)pkt_fatal, (unsigned long)phy_fatal);
                prev_fifo = fifo;
                prev_dep = dep;
                prev_fin = fin;
            }
            ESP_LOGW(TAG, "=== END EXTENDED SAMPLING ===");
            break;
        }
        ESP_LOGI(TAG, "  got AEC frame %d, continuing...", i + 1);
    }
    ESP_LOGI(TAG, "AEC/AGC settling complete");

    return ESP_OK;
}

// ============================================================================
// START RECORDING
// ============================================================================

static void start_recording(void)
{
    total_captured = 0;
    total_written = 0;
    total_dropped = 0;
    recording = true;

    ESP_LOGI(TAG, "Starting recording (%ds)", RECORD_DURATION_SEC);

    if (start_streaming() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start streaming");
        recording = false;
        return;
    }

    // Stop camera stream while SD pre-allocation runs (~1.3s).
    // If left streaming, the 4 DMA buffers fill in ~320ms and
    // the CSI receiver can enter an error state (no DQBUF draining).
    // M2M (H.264) streams stay up; only camera capture is paused.
    {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(recorder->cap_fd, VIDIOC_STREAMOFF, &type);
        ESP_LOGI(TAG, "Camera stream paused for SD pre-alloc");
    }

    // Start SD write task on Core 1 (waits for pre-allocation + init)
    // Priority 14: highest of our tasks — SD writes must not be starved by encode/ISP
    // Stack 12KB: FAT/VFS/SDMMC stack frames are deep (fsync, cluster chain walks)
    write_task_ready = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(sd_write_task, "sd_wr", 16384, NULL, 14, &sd_write_task_handle, 1);
    if (xSemaphoreTake(write_task_ready, pdMS_TO_TICKS(30000)) != pdTRUE) {
        ESP_LOGE(TAG, "Write task init timeout");
        recording = false;
        vSemaphoreDelete(write_task_ready);
        write_task_ready = NULL;
        return;
    }
    vSemaphoreDelete(write_task_ready);
    write_task_ready = NULL;

    // Resume camera stream now that SD is ready and capture task is about to start.
    // STREAMOFF returned all buffers to userspace, so re-queue them first.
    {
        for (int i = 0; i < NUM_CAP_BUFFERS; i++) {
            struct v4l2_buffer buf = {
                .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
                .memory = V4L2_MEMORY_MMAP,
                .index = i,
            };
            if (ioctl(recorder->cap_fd, VIDIOC_QBUF, &buf) != 0) {
                ESP_LOGE(TAG, "Re-QBUF %d failed: %s", i, strerror(errno));
            }
        }
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(recorder->cap_fd, VIDIOC_STREAMON, &type) != 0) {
            ESP_LOGE(TAG, "Camera STREAMON resume failed: %s", strerror(errno));
        } else {
            // Re-apply afull_thrd override (CSI bridge re-init during STREAMON resets it to 960)
            MIPI_CSI_BRIDGE.buf_flow_ctl.csi_buf_afull_thrd = 2040;
            ESP_LOGI(TAG, "Camera stream resumed (afull_thrd=2040)");
        }
    }

    // Start capture+encode on Core 0
    // Priority 15: highest on Core 0 — camera DQBUF must not miss frames
    // Stack 10KB: 3× struct v4l2_buffer on stack + ioctl call chains
    xTaskCreatePinnedToCore(capture_task, "cap_enc", 10240, NULL, 15, &capture_task_handle, 0);

    // Start audio capture task on Core 1 (same core as SD write to avoid cross-core ring buffer contention)
    // Priority 10: below SD write (14) and ISP (11), but above idle — audio is small periodic reads
#if !DIAG_NO_AUDIO
    if (i2s_rx_handle) {
        xTaskCreatePinnedToCore(audio_capture_task, "audio", 6144, NULL, 10, &audio_task_handle, 1);
    }
#endif
}

// ============================================================================
// MAIN
// ============================================================================

void app_main(void)
{
    // Suppress verbose ISP pipeline debug logs so CAM_REC messages are visible
    esp_log_level_set("ISP", ESP_LOG_INFO);

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== ESP32-P4 Camera Recorder ===");
    ESP_LOGI(TAG, "    FireBeetle 2 + RPi Camera 3 (IMX708)");
    ESP_LOGI(TAG, "    H.264 to SD card, %ds recording", RECORD_DURATION_SEC);
    ESP_LOGI(TAG, "");

    // --- RESET CAUSE ---
    esp_reset_reason_t rst = esp_reset_reason();
    const char *rst_str = "UNKNOWN";
    switch (rst) {
        case ESP_RST_POWERON:  rst_str = "POWER_ON"; break;
        case ESP_RST_SW:       rst_str = "SOFTWARE"; break;
        case ESP_RST_PANIC:    rst_str = "PANIC (crash)"; break;
        case ESP_RST_INT_WDT:  rst_str = "INT_WDT"; break;
        case ESP_RST_TASK_WDT: rst_str = "TASK_WDT"; break;
        case ESP_RST_WDT:      rst_str = "OTHER_WDT"; break;
        case ESP_RST_BROWNOUT: rst_str = "BROWNOUT"; break;
        case ESP_RST_DEEPSLEEP: rst_str = "DEEP_SLEEP"; break;
        default: break;
    }
    ESP_LOGW(TAG, ">>> RESET CAUSE: %s (code %d) <<<", rst_str, (int)rst);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Heap: %lu, PSRAM: %lu",
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    init_led();
    led_blink(2, 100);

    ESP_ERROR_CHECK(init_ldo());
    ESP_LOGI(TAG, "[HEAP] after init_ldo: internal=%lu PSRAM=%lu",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_8BIT),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    // No XCLK needed - RPi Camera Module 3 has onboard 24MHz oscillator
    ESP_ERROR_CHECK(init_i2c());
    ESP_LOGI(TAG, "[HEAP] after init_i2c: internal=%lu PSRAM=%lu",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_8BIT),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    // ---- PRE-EMPTIVE CAMERA RESET ----
    // After a soft reset the FireBeetle 2 keeps 3.3V to the IMX708, so it may
    // still be streaming 1080p MIPI data.  The ESP32-P4 CSI receiver needs
    // quiet LP-11 lanes to calibrate its D-PHY.  Silence the sensor via I2C
    // *before* the video drivers load so the receiver wakes to clean lanes.
    {
        ESP_LOGI(TAG, "Pre-emptive camera reset: silencing IMX708 MIPI lanes...");
        i2c_device_config_t cam_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address  = IMX708_I2C_ADDR,
            .scl_speed_hz    = CAM_I2C_FREQ,
        };
        i2c_master_dev_handle_t cam_dev = NULL;
        esp_err_t pret = i2c_master_bus_add_device(i2c_bus_handle, &cam_cfg, &cam_dev);
        if (pret == ESP_OK) {
            // 1. Force stream OFF  (reg 0x0100 = 0x00)
            uint8_t stream_off[] = { 0x01, 0x00, 0x00 };
            esp_err_t tx_ret = i2c_master_transmit(cam_dev, stream_off, sizeof(stream_off), 100);
            if (tx_ret != ESP_OK) {
                ESP_LOGW(TAG, "Stream-off I2C failed (0x%x), resetting bus...", tx_ret);
                i2c_master_bus_reset(i2c_bus_handle);
                vTaskDelay(pdMS_TO_TICKS(50));
                tx_ret = i2c_master_transmit(cam_dev, stream_off, sizeof(stream_off), 100);
                if (tx_ret != ESP_OK) {
                    ESP_LOGE(TAG, "Stream-off retry failed (0x%x)", tx_ret);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(50));

            // 2. Software reset     (reg 0x0103 = 0x01) — clears PLLs & state machine
            uint8_t sw_reset[]  = { 0x01, 0x03, 0x01 };
            tx_ret = i2c_master_transmit(cam_dev, sw_reset, sizeof(sw_reset), 100);
            if (tx_ret != ESP_OK) {
                ESP_LOGW(TAG, "SW reset I2C failed (0x%x), resetting bus...", tx_ret);
                i2c_master_bus_reset(i2c_bus_handle);
                vTaskDelay(pdMS_TO_TICKS(50));
                i2c_master_transmit(cam_dev, sw_reset, sizeof(sw_reset), 100);
            }

            // 3. Wait for MIPI PHY to power down and lanes to return to LP-11
            vTaskDelay(pdMS_TO_TICKS(500));

            i2c_master_bus_rm_device(cam_dev);
            ESP_LOGI(TAG, "Camera silenced. MIPI lanes are clean.");
        } else {
            ESP_LOGW(TAG, "Pre-emptive reset: could not reach IMX708 (0x%x)", pret);
            // Bus may be stuck — try a reset in case SDA is held low
            i2c_master_bus_reset(i2c_bus_handle);
            ESP_LOGI(TAG, "I2C bus reset attempted — may be fresh cold boot");
        }
    }
    // ---- END PRE-EMPTIVE CAMERA RESET ----
    ESP_LOGI(TAG, "[HEAP] after pre-emptive reset: internal=%lu PSRAM=%lu",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_8BIT),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    ESP_ERROR_CHECK(init_esp_video());
    ESP_LOGI(TAG, "[HEAP] after init_esp_video: internal=%lu PSRAM=%lu",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_8BIT),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    ESP_ERROR_CHECK(init_video_pipeline());
    ESP_LOGI(TAG, "[HEAP] after init_video_pipeline: internal=%lu PSRAM=%lu",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_8BIT),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    if (init_isp_white_balance() != ESP_OK) {
        ESP_LOGW(TAG, "ISP WB init failed - colors may be off");
    }

#if !DIAG_NO_AUDIO
    if (init_pdm_mic() != ESP_OK) {
        ESP_LOGW(TAG, "PDM mic init failed - recording without audio");
    }
    ESP_LOGI(TAG, "[HEAP] after init_pdm_mic: internal=%lu PSRAM=%lu",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_8BIT),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
#else
    ESP_LOGW(TAG, "DIAG: Audio disabled (DIAG_NO_AUDIO=1)");
#endif

    if (init_sd_card() == ESP_OK) {
        sd_card_available = true;

        FILE *tf = fopen("/sdcard/vid/test.txt", "w");
        if (tf) {
            fprintf(tf, "SD write test OK\n");
            fclose(tf);
            ESP_LOGI(TAG, "SD write test: OK");
        } else {
            ESP_LOGE(TAG, "SD write test: FAILED");
        }

        // Tune SDMMC DMA for maximum throughput
        sdmmc_dma_tuning();


    } else {
        ESP_LOGW(TAG, "No SD card - will capture without recording");
    }

    ESP_LOGI(TAG, "[HEAP] init complete: internal=%lu PSRAM=%lu",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_8BIT),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    if (sd_card_available) {
        // Auto-retry loop: if MIPI link fails, cool down and try again
        // The sensor is powered off between attempts to prevent overheating
        for (int boot_attempt = 0; boot_attempt < 5; boot_attempt++) {
            if (boot_attempt > 0) {
                ESP_LOGW(TAG, "=== BOOT RETRY %d/4: Re-initializing video stack ===", boot_attempt);
                // Sensor was powered off by start_streaming failure path.
                // Wait for cooldown, then re-power and re-init.
                ESP_LOGI(TAG, "Cooling down for 5 seconds...");
                vTaskDelay(pdMS_TO_TICKS(5000));

                // Re-acquire LDO3 and re-init the full video stack
                esp_ldo_channel_config_t ldo3 = { .chan_id = 3, .voltage_mv = 2500 };
                if (esp_ldo_acquire_channel(&ldo3, &ldo_mipi_handle) != ESP_OK) {
                    ESP_LOGE(TAG, "LDO3 re-acquire failed");
                    continue;
                }
                vTaskDelay(pdMS_TO_TICKS(500));

                if (init_esp_video() != ESP_OK) {
                    ESP_LOGE(TAG, "esp_video re-init failed");
                    continue;
                }
                if (init_video_pipeline() != ESP_OK) {
                    ESP_LOGE(TAG, "video pipeline re-init failed");
                    continue;
                }
                init_isp_white_balance();
            }

            ESP_LOGI(TAG, "Recording starts in 2 seconds...");
            vTaskDelay(pdMS_TO_TICKS(2000));
            start_recording();

            if (recording || total_written > 0) {
                // Recording started (or already finished) — break out of retry loop
                break;
            }
            ESP_LOGW(TAG, "Recording failed to start (attempt %d)", boot_attempt + 1);
        }

        if (!recording && total_written == 0) {
            ESP_LOGE(TAG, "All boot retries exhausted — restarting ESP32 in 3s...");
            vTaskDelay(pdMS_TO_TICKS(3000));
            esp_restart();
        }
    }

    // Status loop
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "[Status] cap=%d wr=%d drop=%d rec=%s heap=%lu",
                 total_captured, total_written, total_dropped,
                 recording ? "YES" : "NO",
                 (unsigned long)esp_get_free_heap_size());

        // Stack high-water-mark monitoring (minimum free stack bytes ever seen)
        // Only check while recording — tasks self-delete after recording stops
        if (recording && sd_write_task_handle) {
            ESP_LOGI(TAG, "[Stack] sd_wr=%lu cap_enc=%lu audio=%lu",
                     (unsigned long)uxTaskGetStackHighWaterMark(sd_write_task_handle) * sizeof(StackType_t),
                     capture_task_handle ? (unsigned long)uxTaskGetStackHighWaterMark(capture_task_handle) * sizeof(StackType_t) : 0,
                     audio_task_handle ? (unsigned long)uxTaskGetStackHighWaterMark(audio_task_handle) * sizeof(StackType_t) : 0);
        }

        // Heap integrity check (catches corruption early)
        if (!heap_caps_check_integrity_all(false)) {
            ESP_LOGE(TAG, "!!! HEAP CORRUPTION DETECTED !!!");
        }
    }
}

# ESP32-P4 Camera Recorder

**1920×1080 H.264+Audio camera recorder** on **FireBeetle 2 ESP32-P4** with Raspberry Pi Camera Module 3 (IMX708).

Records H.264 video with PDM microphone audio to AVI files on SD card.

## Features

- **1920×1080 @ 30fps** MIPI CSI-2 capture via IMX708
- **H.264 hardware encoding** using ESP32-P4's built-in encoder
- **PDM microphone** audio recording (16-bit, 16kHz)
- **AVI container** output to SD card (~5 FPS write throughput)
- **Auto-focus** via DW9807 VCM

## Hardware

- **FireBeetle 2 ESP32-P4** (DFRobot)
- **Raspberry Pi Camera Module 3** (IMX708 + DW9807 VCM)
- **MicroSD card** (FAT32)
- **PDM microphone** on GPIO 46/47

## Performance Tuning

All clocks maximized for best throughput:
- CPU: 360 MHz
- ISP: 240 MHz (PLL_F240M source)
- H.264 encoder: 160 MHz
- PSRAM: 200 MHz
- Flash: QIO mode
- Compiler: -O2 optimization

## Build & Flash

Requires **ESP-IDF v5.4.1** (needed for ESP32-P4 v1.0/ECO1).

```powershell
cd C:\Users\micha\esp-idf-5.4.1
.\export.ps1
cd "G:\_Organized_Loose_Files\esp32p4_hardware_test"
idf.py build
idf.py -p COM5 flash monitor
```

## Submodules

This project uses a modified fork of esp-video-components with ISP clock and CSI bridge fixes:

```bash
git clone --recursive https://github.com/elliotsnd/esp32p4_hardware_test.git
```

## Key Files

- `main/camera_recorder.c` — Main application (capture, encode, mux, record)
- `components/esp-video-components/` — Modified video driver (ISP 240MHz, CSI afull_thrd fix)
- `sdkconfig.defaults` — Project configuration (QIO flash, -O2, 360MHz CPU)

- Check target: `idf.py set-target esp32p4`

## Resources

- [ESP32-P4 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-p4_datasheet_en.pdf)
- [FireBeetle 2 ESP32-P4 Wiki](https://wiki.dfrobot.com/SKU_DFR1075_FireBeetle_2_Board_ESP32_P4)
- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32p4/)
- [Matter Camera Blog](https://developer.espressif.com/blog/2026/01/introducing-esp-matter-camera/)

# ESP32-P4 Camera Recorder

**1920×1080 H.264+Audio camera recorder** on **FireBeetle 2 ESP32-P4** with Raspberry Pi Camera Module 3 (IMX708).

Records H.264 video with PDM microphone audio to AVI files on SD card.

> **⚠️ DISCLAIMER:** 100% of this project was built with AI (Claude Opus 4.6). No human has reviewed the code. Use at your own risk — but it worked for me.

## Current Performance

| Metric | Value |
|--------|-------|
| **Sensor** | 1920×1080 @ 15fps (IMX708 digital crop, 83% FOV) |
| **Encode** | 1280×720 H.264 (PPA 1:1 crop from 1080p for EIS margin) |
| **Recording FPS** | **7.8 FPS** (stable, 0 drops) |
| **Bitrate** | 8 Mbps, GOP=15, QP 22–40 |
| **Audio** | PDM mic, 16-bit 16kHz mono |
| **Heap** | ~15.5 MB free (stable, no leaks) |
| **File format** | AVI, 100MB segments, crash-safe headers |

### Pipeline Timing (per frame)

| Stage | Time | Thread | Notes |
|-------|------|--------|-------|
| CSI capture (dqbuf) | 66.7ms | Capture | Waiting for sensor frame (1000/15 = 66.7ms) |
| PPA crop | 45.8ms | Capture | Hardware DMA: 1920×1080 → 1280×720 YUV420 |
| H.264 encode | 14.7ms | Capture | Hardware encoder, zero-copy pipeline |
| **Capture total** | **127.2ms** | | **← Bottleneck (= 7.86 FPS ceiling)** |
| SD write (video) | 16.7ms | Write | Internal DMA staging, 4-line SDMMC |
| SD write (audio) | 10.9ms | Write | Interleaved PCM chunks |
| **Write total** | **~28ms** | | Idle ~100ms/frame waiting for encoder |

### Bottleneck Analysis

The **capture/encode pipeline** (127.2ms) is the hard ceiling — the SD write thread finishes in ~28ms and waits ~100ms for each frame. The bottleneck breaks down as:

1. **Sensor rate (66.7ms)** — IMX708 delivers at 15fps. Not reducible at 1080p.
2. **PPA crop (45.8ms)** — Hardware DMA speed, no tunable registers. Can only be eliminated by encoding at sensor resolution (OOMs at 1080p — H.264 ref frame needs ~6.2MB).
3. **H.264 encode (14.7ms)** — Already at max 160MHz clock. Efficient.

**To reach higher FPS**, the only practical path is **720p native sensor mode** (skip PPA entirely) → ~24 FPS theoretical, but with narrower 56% FOV and no EIS margin.

## Features

- **1920×1080** MIPI CSI-2 capture via IMX708 (D-PHY timing, digital crop)
- **H.264 hardware encoding** with configurable resolution (1280×720 default)
- **PDM microphone** audio recording (16-bit, 16kHz mono)
- **AVI container** with crash-safe periodic header updates
- **Auto-focus** via DW9807 VCM
- **EIS crop margins** — PPA crops 720p from 1080p (stabilization-ready overscan)
- **Zero-copy pipeline** — DMA from CSI → PPA → H.264 → SD with PSRAM staging

---

## Complete Setup Guide (Beginner-Friendly)

This guide assumes you're on **Windows** and have never used ESP-IDF before. Follow every step in order.

---

### Step 1: What You Need

- **FireBeetle 2 ESP32-P4** (DFRobot)
- **Raspberry Pi Camera Module 3** (standard, not wide)
- **MicroSD card** (FAT32 formatted)
- **USB-C cable**

### Step 2: Connect the Hardware

1. Connect the Camera Module 3 to the FireBeetle 2's CSI connector via the FPC cable. Make sure the cable latch is closed on both ends.
2. Insert a FAT32-formatted MicroSD card.
3. Plug in USB-C to your PC.

### Step 3: Install Prerequisites on Windows

You need **Git** and **Python 3.x** installed before you can install ESP-IDF.

1. Install **Git**: https://git-scm.com/download/win (accept defaults)
2. Install **Python 3**: https://www.python.org/downloads/ — **check "Add python.exe to PATH"** during install

### Step 4: Install ESP-IDF v5.4.1

> **Why v5.4.1 specifically?** Early FireBeetle 2 boards have an ESP32-P4 chip revision v1.0 (ECO1). Newer ESP-IDF versions (v5.5+) build bootloaders for newer chip revisions and will crash with "Illegal instruction" on v1.0 chips. ESP-IDF v5.4.1 is the version that works.

1. Open **PowerShell** (not CMD).

2. Clone ESP-IDF v5.4.1 (this downloads ~1.5 GB and takes a while):
   ```powershell
   cd C:\Users\$env:USERNAME
   git clone -b v5.4.1 --recursive https://github.com/espressif/esp-idf.git esp-idf-5.4.1
   ```
   This will create `C:\Users\YourName\esp-idf-5.4.1\`.

3. Run the install script to download all the toolchains (compilers, debuggers, etc.):
   ```powershell
   cd C:\Users\$env:USERNAME\esp-idf-5.4.1
   .\install.ps1 esp32p4
   ```
   This downloads everything needed to build for the ESP32-P4. It can take 10-20 minutes.

4. Test that it installed correctly:
   ```powershell
   .\export.ps1
   idf.py --version
   ```
   You should see something like `ESP-IDF v5.4.1`. If you get errors, close PowerShell and try again from step 3.

### Step 5: Clone This Project

1. Open **PowerShell**.

2. Pick a folder where you want to put the project, then clone it:
   ```powershell
   cd C:\Users\$env:USERNAME\Documents
   git clone --recursive https://github.com/elliotsnd/esp32p4_hardware_test.git
   cd esp32p4_hardware_test
   ```
   The `--recursive` flag is important — it also downloads the modified video driver in `components/esp-video-components/`.

### Step 6: Build the Firmware

Every time you open a new PowerShell window, you need to "activate" ESP-IDF first. Here's the full sequence:

```powershell
# Step A: Activate ESP-IDF (required once per PowerShell window)
cd C:\Users\$env:USERNAME\esp-idf-5.4.1
.\export.ps1

# Step B: Go to the project folder
cd C:\Users\$env:USERNAME\Documents\esp32p4_hardware_test

# Step C: Build
idf.py build
```

The first build takes several minutes (it compiles the entire ESP-IDF framework). Subsequent builds are much faster.

If the build succeeds, you'll see:
```
Project build complete. To flash, run:
 idf.py flash
```

### Step 7: Find Your COM Port

1. Plug in the FireBeetle 2 via USB.
2. Open **Device Manager** (press Win+X, click "Device Manager").
3. Expand **"Ports (COM & LPT)"**.
4. Look for something like **"USB-ENHANCED-SERIAL CH9102 (COM5)"** or **"USB Serial Device (COM3)"**.
5. Note the **COM number** (e.g., COM5). You'll use this in the next step.

> If you don't see any COM port, you may need to install the CH340/CH9102 USB driver from the DFRobot wiki for the FireBeetle 2.

### Step 8: Flash the Firmware

**IMPORTANT: Close any serial monitors or terminals that are using the COM port before flashing.** If the port is busy, flashing will fail.

```powershell
idf.py -p COM5 flash
```
Replace `COM5` with your actual COM port from Step 7.

You should see progress bars and "Hash of data verified." messages. When it's done:
```
Leaving...
Hard resetting via RTS pin...
```

### Step 9: Watch the Serial Monitor

To see what the device is doing:

```powershell
idf.py -p COM5 monitor
```

You should see log messages showing:
- Camera initialization (IMX708 detected)
- SD card mounted
- Recording starting
- Frame count and FPS updates

Press **Ctrl+]** to exit the monitor.

### Step 10: Record Video

Once flashed, the device automatically:
1. Boots up and initializes the camera, SD card, and microphone
2. Starts recording H.264 video + audio to an AVI file on the SD card
3. Creates files like `REC_0001.avi`, `REC_0002.avi`, etc.

To get your recordings:
1. Press the **reset button** on the FireBeetle 2 (or unplug it) to stop recording cleanly
2. Remove the MicroSD card
3. Put it in your PC's card reader
4. Copy the `.avi` files — they play in VLC, Windows Media Player, etc.

---

## Troubleshooting

### "Illegal instruction" crash on boot
Your ESP-IDF version is too new for your chip. You **must** use ESP-IDF v5.4.1. See Step 4.

### Flash fails with "could not open port" or "Permission denied"
Another program is using the COM port. Close all serial monitors, Python scripts, and other terminals, then try again. In PowerShell:
```powershell
Stop-Process -Name "python" -Force -ErrorAction SilentlyContinue
```

### No COM port shows up in Device Manager
- Try a different USB cable (some are charge-only with no data wires)
- Install the CH340/CH9102 driver from the DFRobot FireBeetle 2 wiki page

### Build fails with "component not found"
Make sure you cloned with `--recursive`:
```powershell
git submodule update --init --recursive
```

### Camera not detected / I2C errors
- Check the FPC cable is seated properly in both connectors
- Make sure the latch is closed on both ends
- The contacts on the cable should face the correct direction (check your board's marking)

### SD card not detected
- Card must be **FAT32** formatted (not exFAT or NTFS)
- Cards over 32GB often come formatted as exFAT — reformat to FAT32
- Try a different card

### Video files are corrupted
- Don't unplug power during recording — always press the reset button first to close the file cleanly
- If files are still bad, the SD card may be too slow — use a Class 10 / U1 or faster card

---

## Project Structure

| File | Purpose |
|------|---------|
| `main/camera_recorder.c` | Main application — capture, H.264 encode, audio record, AVI mux, SD write |
| `components/esp-video-components/` | Modified video driver fork — ISP clock at 240MHz, CSI bridge afull_thrd fix |
| `sdkconfig.defaults` | Build configuration — chip settings, clock speeds, memory layout |
| `CMakeLists.txt` | Top-level build file — registers video component directories |
| `main/idf_component.yml` | Dependency manifest — pulls `cmake_utilities` from ESP component registry |
| `ESP-IDF_541_SETUP.md` | Notes on why v5.4.1 is required for early ESP32-P4 chips |

## Performance Tuning

All clocks are maximized for best throughput:

| Clock | Speed | Notes |
|-------|-------|-------|
| CPU | 360 MHz | Max for ESP32-P4 in ESP-IDF v5.4.1 |
| ISP | 240 MHz | Uses PLL_F240M source (was 80 MHz default — caused stalls at 1080p) |
| H.264 encoder | 160 MHz | Hardcoded max in ESP32-P4 HAL |
| PSRAM | 200 MHz | Max for hex-SPI PSRAM |
| Flash | QIO mode | 2× read bandwidth vs default DIO |
| Compiler | -O2 | Performance optimization (default is -Og debug) |

- [ESP32-P4 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-p4_datasheet_en.pdf)
- [FireBeetle 2 ESP32-P4 Wiki](https://wiki.dfrobot.com/SKU_DFR1075_FireBeetle_2_Board_ESP32_P4)
- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32p4/)
- [Matter Camera Blog](https://developer.espressif.com/blog/2026/01/introducing-esp-matter-camera/)

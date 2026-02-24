# ESP32-P4 Hardware Test Project

Comprehensive hardware test for **FireBeetle 2 ESP32-P4** while waiting for video components.

## What This Tests

✅ **System Info** - Chip details, PSRAM, flash, heap memory  
✅ **LED Blink** - GPIO output functionality  
✅ **I2C Scan** - Detect connected I2C devices (camera sensor will show here)  
✅ **SD Card** - SDMMC interface, file read/write  
✅ **WiFi** - MAC address, initialization  

## Hardware Requirements

- **FireBeetle 2 ESP32-P4** board
- **TF/SD card** (optional, for storage test)
- **Raspberry Pi Camera 3** (optional, will be detected on I2C)

## Build & Flash

```powershell
# Activate ESP-IDF
cd C:\Users\micha\esp-idf
.\export.ps1

# Navigate to project
cd "G:\_Organized_Loose_Files\esp32p4_hardware_test"

# Configure, build, and flash
idf.py set-target esp32p4
idf.py build
idf.py -p COM3 flash monitor
```

## Expected Output

```
I (xxx) ESP32P4_TEST: ╔════════════════════════════════════════╗
I (xxx) ESP32P4_TEST: ║   ESP32-P4 FireBeetle 2 Hardware Test ║
I (xxx) ESP32P4_TEST: ║   Preparing for future camera support ║
I (xxx) ESP32P4_TEST: ╚════════════════════════════════════════╝

I (xxx) ESP32P4_TEST: === System Information ===
I (xxx) ESP32P4_TEST: Chip: ESP32-P4
I (xxx) ESP32P4_TEST: Cores: 2
I (xxx) ESP32P4_TEST: PSRAM: ENABLED (32 MB)
...

I (xxx) ESP32P4_TEST: === I2C Bus Scan ===
I (xxx) ESP32P4_TEST: Found device at address: 0x1A  (RPi Camera 3 IMX708)
...

I (xxx) ESP32P4_TEST: === SD Card Test ===
I (xxx) ESP32P4_TEST: SD Card: DETECTED
I (xxx) ESP32P4_TEST: Size: 32768 MB
I (xxx) ESP32P4_TEST: File read verification: PASSED
...
```

## Pin Connections

### SD Card (SDMMC 4-bit mode)
- CMD: GPIO 33
- CLK: GPIO 34
- D0: GPIO 37
- D1: GPIO 38
- D2: GPIO 39
- D3: GPIO 40

### I2C (for camera sensor)
- SDA: GPIO 8
- SCL: GPIO 9

### LED (adjust if needed)
- LED: GPIO 15

## Camera Detection

When you connect the **Raspberry Pi Camera 3**, you should see:
```
I (xxx) ESP32P4_TEST: Found device at address: 0x1A
```

The IMX708 sensor is at I2C address `0x1A`. This confirms the camera is electrically connected and ready for video components.

## What's Missing (Coming Q2-Q3 2026)

❌ **MIPI-CSI video capture** - Needs `esp-video` component  
❌ **Hardware H.264 encoding** - Needs `esp_h264` component  
❌ **Video codec middleware** - Needs `esp_video_codec` component  

## Next Steps

1. **Run this test** to verify all peripherals work
2. **Monitor Espressif releases** for video components:
   - https://components.espressif.com/
   - https://github.com/espressif/esp-idf/releases
3. **When released**, upgrade to video recording firmware

## Troubleshooting

**SD Card not detected:**
- Check card is formatted as FAT32
- Try different card (Class 10+ recommended)
- Verify pin connections

**I2C devices not found:**
- Check SDA/SCL pins match your board
- Camera requires 3.3V power
- Try adjusting GPIO numbers in code

**Build errors:**
- Ensure ESP-IDF v5.5 is activated: `.\export.ps1`
- Clean build: `idf.py fullclean`
- Check target: `idf.py set-target esp32p4`

## Resources

- [ESP32-P4 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-p4_datasheet_en.pdf)
- [FireBeetle 2 ESP32-P4 Wiki](https://wiki.dfrobot.com/SKU_DFR1075_FireBeetle_2_Board_ESP32_P4)
- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32p4/)
- [Matter Camera Blog](https://developer.espressif.com/blog/2026/01/introducing-esp-matter-camera/)

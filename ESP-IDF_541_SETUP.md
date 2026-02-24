# ESP-IDF v5.4.1 Setup for ESP32-P4 v1.0

## Why v5.4.1?
Your ESP32-P4 FireBeetle 2 is **chip revision v1.0 (ECO1)** - an early engineering sample. ESP-IDF v5.5.x bootloaders are built for ECO5 (v3.1+) and crash with "Illegal instruction" errors on v1.0 chips due to:
- Different cache management (dcache/icache synchronization)
- Incompatible instruction sets
- ROM code differences

The M5Stack Tab5 documentation (dated May 2025) confirms **ESP-IDF v5.4.1 successfully runs on ESP32-P4 v1.0 chips**.

## Installation Status

### Completed:
✅ WSL2 Ubuntu password reset (admin)
✅ Dependencies installed (git, cmake, ninja-build, libusb, python3-venv)
✅ ESP-IDF v5.4.1 cloning from GitHub (in progress)
⏳ ESP-IDF tools installation (./install.sh esp32p4) - pending clone completion
⏳ Hello World build test - pending

### Next Steps:
1. Install ESP-IDF tools: `./install.sh esp32p4`
2. Source environment: `. ./export.sh`
3. Build Hello World: `idf.py build`
4. Flash to COM5 (requires usbipd for WSL2 USB passthrough)

## USB Access from WSL2

Windows COM5 (ESP32-P4 USB-Serial/JTAG) needs to be shared with WSL2:

```powershell
# On Windows PowerShell (Admin):
usbipd list
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```

Then in WSL2:
```bash
ls /dev/tty*  # Should see /dev/ttyACM0 or similar
```

## Expected Outcome

If v5.4.1 bootloader supports ECO1:
- ✅ Firmware flashes without --force flag
- ✅ Bootloader executes without illegal instruction crash
- ✅ Hello World prints to serial monitor
- ✅ Hardware test project can run

If issues persist:
- Check if CONFIG_ESP32P4_REV_MIN_FULL needs adjustment
- Verify M5Stack Tab5 demo firmware compatibility
- Consider DFRobot support contact for ECO1-specific guidance

## Reference
- ESP-IDF v5.4.1 release: https://github.com/espressif/esp-idf/releases/tag/v5.4.1
- M5Stack Tab5 guide: https://www.cnx-software.com/2025/05/18/esp32-p4-based-m5stack-tab5-esp-idf-5-4-1-getting-started/

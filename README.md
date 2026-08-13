# ESP32 BLE Sniffer (Crowd Alert)

A simplified version of pi-sniffer designed specifically for ESP32 devices. This firmware scans for nearby Bluetooth Low Energy (BLE) devices and estimates crowd density by counting devices.

## Features

- **BLE Device Scanning**: Continuously scans for nearby BLE devices
- **Device Tracking**: Maintains a list of discovered devices with signal strength tracking
- **Kalman Filtering**: Smooths RSSI readings for more accurate distance estimation
- **Distance Calculation**: Estimates device distance based on RSSI values
- **MAC Randomization Handling**: Detects public vs random MAC addresses
- **REST API**: Sends device data to a configurable HTTP endpoint
- **WiFi Provisioning**: Easy WiFi setup via captive portal (no hardcoded credentials!)
- **Double-Press Config**: Re-enter config mode anytime by double-pressing BOOT button

## Hardware Requirements

- ESP32 development board (ESP32-WROOM-32, ESP32-DevKitC, etc.)
- USB cable for programming and power

### ESP32-S3 SIM7670G board (GNSS)

GNSS support (see [gnss.c](src/gnss.c)/[gnss.h](include/gnss.h)) targets the
Waveshare ESP32-S3 SIM7670G 4G development board. Board docs (pinout,
ESP-IDF setup, GNSS notes) are here:
https://docs.waveshare.com/ESP32-S3-SIM7670G-4G/ESP-IDF

## First-Time Setup

1. **Flash the firmware** to your ESP32
2. **Power on** the device
3. The device will create a WiFi network: **`DC Sniffer`**
4. **Connect** to this network with your phone or computer
5. **Open a browser** and go to: `http://192.168.4.1`
6. **Enter your WiFi credentials** and click "Save & Connect"
7. The device will **reboot** and connect to your WiFi network

## Re-Entering Configuration Mode

To change WiFi settings after initial setup:

1. **Double-press the BOOT button** within 0.5 seconds
2. The device will restart in config mode
3. Connect to `DC Sniffer` and reconfigure

Alternatively, on boot:
- You have **3 seconds** to double-press BOOT button to enter config mode
- The LED may flash to indicate the window for double-press

## Configuration

Edit `include/config.h` to configure:

```c
// REST API endpoint
#define API_HOST            "your-server.com"
#define API_PORT            80
#define API_PATH            "/api/devices"

// Scanning parameters
#define SCAN_DURATION_SEC   10      // Duration of each scan cycle
#define REPORT_INTERVAL_SEC 30      // How often to send data to API
#define MAX_DEVICES         128     // Maximum tracked devices
```

## Building and Flashing

### Using PlatformIO

1. Install PlatformIO IDE or CLI
2. Open this folder in PlatformIO
3. Build and upload:
   ```bash
   pio run -t upload
   ```

### Using ESP-IDF

1. Install ESP-IDF v5.0+
2. Configure the project:
   ```bash
   idf.py menuconfig
   ```
3. Build and flash:
   ```bash
   idf.py build flash monitor
   ```

## REST API Data Format

The device sends JSON data to the configured endpoint:

```json
{
  "device_id": "ESP32_AABBCC",
  "timestamp": 1704246600,
  "devices": [
    {
      "mac": "AA:BB:CC:DD:EE:FF",
      "rssi": -65,
      "distance": 2.5,
      "name": "iPhone",
      "category": "phone",
      "address_type": "random",
      "seen_count": 15,
      "first_seen": 1704246000,
      "last_seen": 1704246590
    }
  ],
  "summary": {
    "total_devices": 12,
    "phones": 5,
    "computers": 2,
    "wearables": 1,
    "beacons": 3,
    "other": 1
  }
}
```

## How It Works

1. **Boot Check**: On startup, waits 3 seconds for double-press to enter config mode
2. **Credential Check**: If no WiFi credentials stored, automatically enters config mode
3. **Provisioning**: In config mode, creates AP with captive portal for WiFi setup
4. **WiFi Connect**: Connects to configured WiFi network
5. **Scanning**: Continuously scans for BLE devices
6. **Tracking**: Each discovered device is added to an internal tracking list
7. **Filtering**: RSSI values are smoothed using a Kalman filter
8. **Distance**: Distance is calculated from smoothed RSSI values
9. **Classification**: Devices are categorized based on manufacturer data and names
10. **Reporting**: Periodically sends device data to the REST API endpoint

## Button Functions

| Action | Function |
|--------|----------|
| **Double-press** (during 3s boot window) | Enter config mode |
| **Double-press** (while running) | Clear credentials & reboot to config mode |
| **Long press** (2+ seconds) | Clear all credentials & reboot |
| **Single press** | Print device summary to serial log |

## Crowd Estimation

The system estimates crowd size by counting mobile phones. Since most people carry phones with active Bluetooth, the phone count provides a reasonable estimate of people present.

**Note**: iOS devices randomize their MAC addresses, but the system handles this by:
- Tracking device appearance patterns
- Using manufacturer data for identification
- Maintaining a minimum device count based on overlapping observations

## Troubleshooting

### Can't connect to WiFi after configuration
- Double-press BOOT button to re-enter config mode
- Check that SSID and password are correct
- Ensure your router is 2.4GHz (ESP32 doesn't support 5GHz)

### Captive portal doesn't appear
- Manually navigate to `http://192.168.4.1`
- Try a different browser or device
- Disable mobile data temporarily

### Device keeps rebooting
- Long-press BOOT button (2+ seconds) to clear all settings
- Re-flash the firmware

## Project Structure

```
esp32/
├── include/
│   ├── config.h           # Configuration settings
│   ├── kalman.h           # Kalman filter header
│   ├── device.h           # Device tracking structures
│   ├── ble_scanner.h      # BLE scanner interface
│   ├── wifi_manager.h     # WiFi station mode
│   ├── wifi_provision.h   # Captive portal provisioning
│   ├── button_handler.h   # Button press detection
│   └── http_client.h      # REST API client
└── src/
    ├── main.c             # Main application
    ├── kalman.c           # Kalman filter implementation
    ├── device.c           # Device tracking logic
    ├── ble_scanner.c      # BLE scanning
    ├── wifi_manager.c     # WiFi connection handling
    ├── wifi_provision.c   # Captive portal & NVS storage
    ├── button_handler.c   # Button interrupt handling
    └── http_client.c      # HTTP POST implementation
```

## License

MIT License - See LICENSE file for details.

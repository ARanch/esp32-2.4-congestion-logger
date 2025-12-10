# Wireless Environment Logger

Log 2.4 GHz wireless congestion at live venues using ESP32 devices.

## Hardware

- Heltec ESP32 LoRa V3 board
- USB cable for power and serial communication

## Project Structure

```
├── firmware/           # ESP32 PlatformIO project
│   ├── platformio.ini
│   └── src/
│       └── main.cpp
├── logger.py           # Python serial logger
└── README.md
```

## Firmware Setup

### Prerequisites

- [PlatformIO](https://platformio.org/) (VS Code extension or CLI)

### Build and Upload

```bash
cd firmware
pio run --target upload
```

Or use the PlatformIO IDE to build and upload.

### What it does

Every 5 seconds, the firmware:
1. Scans for WiFi networks (counts total, per-channel for 1/6/11, RSSI stats)
2. Scans for BLE advertising devices
3. Outputs CSV data via serial
4. Updates the OLED display with current stats

### Serial Output Format

```
timestamp_ms,wifi_total,ch1,ch6,ch11,rssi_max,rssi_avg,ble_devices
12345,15,5,7,3,-45,-68,23
```

## Python Logger Setup

### Prerequisites

```bash
pip install pyserial
```

### Usage

```bash
python logger.py --port /dev/cu.usbserial-XXX --name venue1
```

#### Arguments

| Argument | Required | Description |
|----------|----------|-------------|
| `--port`, `-p` | Yes | Serial port (e.g., `/dev/cu.usbserial-XXX` on macOS, `COM3` on Windows) |
| `--name`, `-n` | Yes | Location/device identifier (used in filename) |
| `--baud`, `-b` | No | Baud rate (default: 115200) |
| `--output-dir`, `-o` | No | Directory for output files (default: current directory) |

### Output

Creates a CSV file named: `{name}_{date}_{time}.csv`

Example: `venue1_2024-01-15_19-30-00.csv`

The logger prepends an ISO timestamp to each line from the ESP32:

```csv
iso_timestamp,timestamp_ms,wifi_total,ch1,ch6,ch11,rssi_max,rssi_avg,ble_devices
2024-01-15T19:30:05.123456,12345,15,5,7,3,-45,-68,23
```

## Finding the Serial Port

### macOS
```bash
ls /dev/cu.usb*
```

### Linux
```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

### Windows
Check Device Manager for COM ports, or:
```cmd
mode
```

## Troubleshooting

- **No serial output**: Ensure the correct port is selected and baud rate matches (115200)
- **OLED not working**: Check that you're using a Heltec ESP32 LoRa V3 board (pin mappings differ between boards)
- **BLE scan returns 0**: Some environments may have very few BLE devices; this is normal
- **WiFi shows 0 networks**: Ensure you're in range of WiFi networks; hidden networks are included in the scan

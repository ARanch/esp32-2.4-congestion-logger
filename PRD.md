# Wireless Environment Logger - PRD

## Purpose
Log 2.4 GHz wireless congestion at live venues (concerts, events) to:
1. Understand real-world RF conditions for product validation
2. Generate baseline data to replicate in controlled test environments
3. Compare different venues/scenarios

## Hardware
- 2-3x ESP32 LoRa V3 boards (Heltec-style, SX1262 868MHz - LoRa unused)
- 0.96" OLED display (SSD1306, 128x64)
- USB connection to laptop for power + serial logging

## ESP32 Firmware Requirements

### Measurements (loop every 5 seconds)
- **WiFi scan**: network count, per-channel breakdown (channels 1,6,11 at minimum), strongest RSSI, average RSSI
- **BLE scan**: count of unique advertising devices seen in scan window

### OLED Display
Show live stats so operator can see it's working:
- Total WiFi networks
- BLE device count
- Uptime or scan count
- Keep it simple, update each scan cycle

### Serial Output
CSV format, one line per scan:
```
timestamp_ms,wifi_total,ch1,ch6,ch11,rssi_max,rssi_avg,ble_devices
```
Use millis() for timestamp - the Python logger will add wall-clock time.

### Platform
Arduino framework. Target Heltec ESP32 LoRa V3 board definition.

## Python Logger (laptop side)

### Function
- Read serial from ESP32
- Prepend ISO timestamp to each line
- Write to CSV file with device identifier in filename
- Print to console for monitoring

### Usage
```bash
python logger.py --port /dev/cu.usbserial-XXX --name location1
```
Creates: `location1_2024-01-15_19-30-00.csv`

### Requirements
- pyserial
- Minimal dependencies, single file

## Deliverables
1. `firmware/` - PlatformIO or Arduino project for ESP32
2. `logger.py` - Python serial logger
3. `README.md` - Setup and usage instructions

## Out of Scope
- LoRa functionality
- Real-time graphing (post-process CSVs separately)
- Web interface

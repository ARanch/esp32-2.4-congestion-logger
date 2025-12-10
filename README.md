# ESP32 2.4GHz Wireless Environment Logger

Logs WiFi and BLE congestion at live venues to understand RF conditions over time, and comparing congestion between venues. Calculates a **Wireless Congestion Index (WCI)** from 0-100.

## Note on WCI (Wireless Congestion Index)

WCI is an experimental proxy metric, not a calibrated measurement. It combines WiFi network counts, signal strength, and BLE activity into a single 0-100 score for quick comparison between environments.

The current saturation points are arbitrary. Calibration is in progress using manual noise floor readings from a dedicated 2.4 GHz spectrum analyzer at real venues (concerts, trade shows). Once we have data showing how WCI correlates with actual RF conditions, the formula will be adjusted to reflect reality.

Until then, treat WCI as a rough indicator—useful for spotting relative changes over time, not for absolute judgments.

## What it measures

- **WiFi**: Network count, per-channel breakdown (1/6/11), RSSI stats
- **BLE**: Unique devices, total packets, RSSI distribution, advertising types
- **WCI**: Derived congestion score combining all metrics

## Hardware

- Heltec WiFi LoRa 32 V3 (ESP32-S3)
- USB cable

## Quick Start

```bash
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
./start.sh venue_name
```

### start.sh

```bash
./start.sh              # build + log as "test"
./start.sh venue1       # build + log as "venue1"
./start.sh -n           # skip build, log as "test"
./start.sh -n venue1    # skip build, log as "venue1"
```

Output: `venue_name_2024-01-15_19-30-00.csv`

Ctrl+C to stop logging.

## Usage

- **PRG button**: Cycle display pages (WCI / WiFi / BLE)
- Scans every 5 seconds
- CSV output via serial at 115200 baud

## CSV Columns

```
timestamp_ms,wifi_total,ch1,ch6,ch11,wifi_rssi_max,wifi_rssi_avg,
ble_devices,ble_packets,ble_rssi_min,ble_rssi_max,ble_rssi_avg,
ble_connectable,ble_nonconnectable,ble_scannable,wci
```

## Find Serial Port

```bash
# macOS
ls /dev/cu.usb*

# Linux
ls /dev/ttyUSB*

# Windows
mode
```

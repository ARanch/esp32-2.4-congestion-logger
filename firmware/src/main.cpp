/**
 * Wireless Environment Logger
 * ESP32 firmware for logging 2.4GHz wireless congestion
 *
 * Target: Heltec ESP32 LoRa V3
 * Output: CSV via serial, stats on OLED
 */

#include <Arduino.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <U8g2lib.h>
#include <Wire.h>

// Heltec ESP32 LoRa V3 OLED pins (ESP32-S3)
#define OLED_SDA 17
#define OLED_SCL 18
#define OLED_RST 21
#define VEXT_CTRL 36  // Vext power control for OLED

// Scan configuration
#define SCAN_INTERVAL_MS 5000
#define BLE_SCAN_TIME_SEC 3

// Stats
uint32_t scanCount = 0;

// OLED display using U8g2 - software I2C for Heltec V3
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, OLED_SCL, OLED_SDA, OLED_RST);

// BLE scanner
BLEScan* pBLEScan = nullptr;

// WiFi scan results structure
struct WiFiScanResult {
    int totalNetworks;
    int ch1Count;
    int ch6Count;
    int ch11Count;
    int rssiMax;
    int rssiAvg;
};

// BLE scan results structure
struct BLEScanResult {
    int uniqueDevices;
    int packetCount;
    int rssiMin;
    int rssiMax;
    int rssiAvg;
    int connectable;
    int nonConnectable;
    int scannable;
};

// Global BLE stats for callback
BLEScanResult g_bleStats;

// BLE callback to count every advertising packet
class BLEPacketCallback : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) override {
        g_bleStats.packetCount++;

        int rssi = advertisedDevice.getRSSI();

        // Update RSSI stats
        if (rssi < g_bleStats.rssiMin) g_bleStats.rssiMin = rssi;
        if (rssi > g_bleStats.rssiMax) g_bleStats.rssiMax = rssi;

        // Classify by advertisement flags if available
        // If device has service data or is connectable, classify accordingly
        if (advertisedDevice.haveServiceData() || advertisedDevice.haveServiceUUID()) {
            // Devices advertising services are typically connectable
            g_bleStats.connectable++;
        } else if (advertisedDevice.haveName()) {
            // Named devices are usually scannable
            g_bleStats.scannable++;
        } else {
            // Beacon-like devices (no name, no services) are typically non-connectable
            g_bleStats.nonConnectable++;
        }
    }
};

BLEPacketCallback* bleCallback = nullptr;

void initOLED() {
    // Enable Vext power for OLED (LOW = ON for Heltec V3)
    pinMode(VEXT_CTRL, OUTPUT);
    digitalWrite(VEXT_CTRL, LOW);
    delay(100);

    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, "RF Logger");
    u8g2.drawStr(0, 24, "Initializing...");
    u8g2.sendBuffer();
}

void initBLE() {
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();
    bleCallback = new BLEPacketCallback();
    pBLEScan->setAdvertisedDeviceCallbacks(bleCallback, false);  // false = don't dedupe
    pBLEScan->setActiveScan(false);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
}

WiFiScanResult performWiFiScan() {
    WiFiScanResult result = {0, 0, 0, 0, -100, 0};

    int n = WiFi.scanNetworks(false, true);

    if (n <= 0) {
        return result;
    }

    result.totalNetworks = n;
    int rssiSum = 0;

    for (int i = 0; i < n; i++) {
        int channel = WiFi.channel(i);
        int rssi = WiFi.RSSI(i);

        if (channel == 1) result.ch1Count++;
        else if (channel == 6) result.ch6Count++;
        else if (channel == 11) result.ch11Count++;

        if (rssi > result.rssiMax) {
            result.rssiMax = rssi;
        }
        rssiSum += rssi;
    }

    result.rssiAvg = rssiSum / n;
    WiFi.scanDelete();

    return result;
}

BLEScanResult performBLEScan() {
    // Reset global stats
    g_bleStats = {0, 0, 0, -100, 0, 0, 0, 0};  // rssiMin starts high, rssiMax starts low

    BLEScanResults results = pBLEScan->start(BLE_SCAN_TIME_SEC, false);
    g_bleStats.uniqueDevices = results.getCount();

    // Calculate average RSSI
    if (g_bleStats.packetCount > 0) {
        // We need to recalculate since we only tracked min/max
        // Let's iterate through results for average
        int rssiSum = 0;
        for (int i = 0; i < results.getCount(); i++) {
            rssiSum += results.getDevice(i).getRSSI();
        }
        if (results.getCount() > 0) {
            g_bleStats.rssiAvg = rssiSum / results.getCount();
        }
    }

    pBLEScan->clearResults();
    return g_bleStats;
}

void updateDisplay(WiFiScanResult& wifi, BLEScanResult& ble) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_5x7_tf);  // Smaller font to fit more info

    char buf[32];

    // WiFi line
    snprintf(buf, sizeof(buf), "WiFi:%d Ch1:%d/6:%d/11:%d",
             wifi.totalNetworks, wifi.ch1Count, wifi.ch6Count, wifi.ch11Count);
    u8g2.drawStr(0, 8, buf);

    // WiFi RSSI
    snprintf(buf, sizeof(buf), "WiFi RSSI max:%d avg:%d", wifi.rssiMax, wifi.rssiAvg);
    u8g2.drawStr(0, 18, buf);

    // BLE devices and packets
    snprintf(buf, sizeof(buf), "BLE:%d dev %d pkt", ble.uniqueDevices, ble.packetCount);
    u8g2.drawStr(0, 28, buf);

    // BLE RSSI
    snprintf(buf, sizeof(buf), "BLE RSSI %d/%d/%d", ble.rssiMin, ble.rssiAvg, ble.rssiMax);
    u8g2.drawStr(0, 38, buf);

    // BLE adv types
    snprintf(buf, sizeof(buf), "Conn:%d NC:%d Scan:%d",
             ble.connectable, ble.nonConnectable, ble.scannable);
    u8g2.drawStr(0, 48, buf);

    // Scan count
    snprintf(buf, sizeof(buf), "Scan #%lu", scanCount);
    u8g2.drawStr(0, 58, buf);

    u8g2.sendBuffer();
}

void outputCSV(unsigned long timestamp, WiFiScanResult& wifi, BLEScanResult& ble) {
    // Format: timestamp_ms,wifi_total,ch1,ch6,ch11,wifi_rssi_max,wifi_rssi_avg,
    //         ble_devices,ble_packets,ble_rssi_min,ble_rssi_max,ble_rssi_avg,
    //         ble_connectable,ble_nonconnectable,ble_scannable
    Serial.printf("%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                  timestamp,
                  wifi.totalNetworks,
                  wifi.ch1Count,
                  wifi.ch6Count,
                  wifi.ch11Count,
                  wifi.rssiMax,
                  wifi.rssiAvg,
                  ble.uniqueDevices,
                  ble.packetCount,
                  ble.rssiMin,
                  ble.rssiMax,
                  ble.rssiAvg,
                  ble.connectable,
                  ble.nonConnectable,
                  ble.scannable);
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("Starting RF Logger...");

    Serial.println("Init OLED...");
    initOLED();

    Serial.println("Init WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    Serial.println("Init BLE...");
    initBLE();

    Serial.println("Init complete!");
    Serial.println("timestamp_ms,wifi_total,ch1,ch6,ch11,wifi_rssi_max,wifi_rssi_avg,ble_devices,ble_packets,ble_rssi_min,ble_rssi_max,ble_rssi_avg,ble_connectable,ble_nonconnectable,ble_scannable");

    u8g2.clearBuffer();
    u8g2.drawStr(0, 10, "RF Logger");
    u8g2.drawStr(0, 24, "Ready!");
    u8g2.sendBuffer();
    delay(500);
}

void loop() {
    unsigned long loopStart = millis();

    WiFiScanResult wifiResult = performWiFiScan();
    BLEScanResult bleResult = performBLEScan();

    scanCount++;

    outputCSV(loopStart, wifiResult, bleResult);
    updateDisplay(wifiResult, bleResult);

    unsigned long elapsed = millis() - loopStart;
    if (elapsed < SCAN_INTERVAL_MS) {
        delay(SCAN_INTERVAL_MS - elapsed);
    }
}

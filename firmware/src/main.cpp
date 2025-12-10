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

int performBLEScan() {
    BLEScanResults results = pBLEScan->start(BLE_SCAN_TIME_SEC, false);
    int count = results.getCount();
    pBLEScan->clearResults();
    return count;
}

void updateDisplay(WiFiScanResult& wifi, int bleCount) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);

    u8g2.drawStr(0, 10, "RF Environment Logger");
    u8g2.drawLine(0, 12, 128, 12);

    char buf[32];
    snprintf(buf, sizeof(buf), "WiFi: %d networks", wifi.totalNetworks);
    u8g2.drawStr(0, 24, buf);

    snprintf(buf, sizeof(buf), "Ch1:%d Ch6:%d Ch11:%d",
             wifi.ch1Count, wifi.ch6Count, wifi.ch11Count);
    u8g2.drawStr(0, 36, buf);

    snprintf(buf, sizeof(buf), "BLE: %d devices", bleCount);
    u8g2.drawStr(0, 48, buf);

    snprintf(buf, sizeof(buf), "Scans: %lu", scanCount);
    u8g2.drawStr(0, 60, buf);

    u8g2.sendBuffer();
}

void outputCSV(unsigned long timestamp, WiFiScanResult& wifi, int bleCount) {
    Serial.printf("%lu,%d,%d,%d,%d,%d,%d,%d\n",
                  timestamp,
                  wifi.totalNetworks,
                  wifi.ch1Count,
                  wifi.ch6Count,
                  wifi.ch11Count,
                  wifi.rssiMax,
                  wifi.rssiAvg,
                  bleCount);
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
    Serial.println("timestamp_ms,wifi_total,ch1,ch6,ch11,rssi_max,rssi_avg,ble_devices");

    u8g2.clearBuffer();
    u8g2.drawStr(0, 10, "RF Logger");
    u8g2.drawStr(0, 24, "Ready!");
    u8g2.sendBuffer();
    delay(500);
}

void loop() {
    unsigned long loopStart = millis();

    WiFiScanResult wifiResult = performWiFiScan();
    int bleCount = performBLEScan();

    scanCount++;

    outputCSV(loopStart, wifiResult, bleCount);
    updateDisplay(wifiResult, bleCount);

    unsigned long elapsed = millis() - loopStart;
    if (elapsed < SCAN_INTERVAL_MS) {
        delay(SCAN_INTERVAL_MS - elapsed);
    }
}

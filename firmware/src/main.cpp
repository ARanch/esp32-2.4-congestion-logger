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
#define PRG_BUTTON 0  // PRG button on Heltec V3

// Scan configuration
#define SCAN_INTERVAL_MS 5000
#define BLE_SCAN_TIME_SEC 3

// WCI saturation points (calibrate after real venue tests)
#define WCI_WIFI_SAT 50.0f
#define WCI_BLE_DEV_SAT 400.0f
#define WCI_BLE_ACT_SAT 3.0f

// Stats
uint32_t scanCount = 0;
float lastWCI = 0.0f;
int displayPage = 0;  // 0 = WCI, 1 = WiFi, 2 = BLE
const int NUM_PAGES = 3;

// Button debounce
volatile bool buttonPressed = false;
unsigned long lastButtonTime = 0;
const unsigned long DEBOUNCE_MS = 200;

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

// Button interrupt handler
void IRAM_ATTR onButtonPress() {
    buttonPressed = true;
}

// Calculate Wireless Congestion Index
float calculateWCI(WiFiScanResult& wifi, BLEScanResult& ble) {
    // W_net = min(wifi_total / 50, 1) * 100
    float wNet = fmin((float)wifi.totalNetworks / WCI_WIFI_SAT, 1.0f) * 100.0f;

    // W_rssi = max(0, (wifi_rssi_avg + 90) / 40) * 100
    float wRssi = fmax(0.0f, ((float)wifi.rssiAvg + 90.0f) / 40.0f) * 100.0f;
    wRssi = fmin(wRssi, 100.0f);  // Cap at 100

    // B_dev = min(ble_devices / 400, 1) * 100
    float bDev = fmin((float)ble.uniqueDevices / WCI_BLE_DEV_SAT, 1.0f) * 100.0f;

    // B_act = min((ble_packets / ble_devices) / 3, 1) * 100
    float bAct = 0.0f;
    if (ble.uniqueDevices > 0) {
        float packetsPerDevice = (float)ble.packetCount / (float)ble.uniqueDevices;
        bAct = fmin(packetsPerDevice / WCI_BLE_ACT_SAT, 1.0f) * 100.0f;
    }

    // WCI = (W_net * 0.25) + (W_rssi * 0.20) + (B_dev * 0.35) + (B_act * 0.20)
    float wci = (wNet * 0.25f) + (wRssi * 0.20f) + (bDev * 0.35f) + (bAct * 0.20f);

    return wci;
}

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

void displayPageWCI(WiFiScanResult& wifi, BLEScanResult& ble) {
    u8g2.clearBuffer();
    char buf[32];

    // Large WCI display
    u8g2.setFont(u8g2_font_logisoso28_tf);
    snprintf(buf, sizeof(buf), "%.1f", lastWCI);
    // Center the WCI value
    int width = u8g2.getStrWidth(buf);
    u8g2.drawStr((128 - width) / 2, 38, buf);

    // Label above
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, "WCI");

    // Summary below
    u8g2.setFont(u8g2_font_5x7_tf);
    snprintf(buf, sizeof(buf), "WiFi:%d BLE:%d/%d Scan#%lu",
             wifi.totalNetworks, ble.uniqueDevices, ble.packetCount, scanCount);
    u8g2.drawStr(0, 58, buf);

    // Page indicator
    u8g2.drawStr(118, 58, "1/3");

    u8g2.sendBuffer();
}

void displayPageWiFi(WiFiScanResult& wifi) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    char buf[32];

    u8g2.drawStr(0, 10, "WiFi Details");
    u8g2.drawLine(0, 12, 128, 12);

    u8g2.setFont(u8g2_font_5x7_tf);

    snprintf(buf, sizeof(buf), "Total networks: %d", wifi.totalNetworks);
    u8g2.drawStr(0, 22, buf);

    snprintf(buf, sizeof(buf), "Channel 1:  %d", wifi.ch1Count);
    u8g2.drawStr(0, 32, buf);

    snprintf(buf, sizeof(buf), "Channel 6:  %d", wifi.ch6Count);
    u8g2.drawStr(0, 42, buf);

    snprintf(buf, sizeof(buf), "Channel 11: %d", wifi.ch11Count);
    u8g2.drawStr(0, 52, buf);

    snprintf(buf, sizeof(buf), "RSSI max:%d avg:%d", wifi.rssiMax, wifi.rssiAvg);
    u8g2.drawStr(0, 62, buf);

    // Page indicator
    u8g2.drawStr(118, 62, "2/3");

    u8g2.sendBuffer();
}

void displayPageBLE(BLEScanResult& ble) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    char buf[32];

    u8g2.drawStr(0, 10, "BLE Details");
    u8g2.drawLine(0, 12, 128, 12);

    u8g2.setFont(u8g2_font_5x7_tf);

    snprintf(buf, sizeof(buf), "Unique devices: %d", ble.uniqueDevices);
    u8g2.drawStr(0, 22, buf);

    snprintf(buf, sizeof(buf), "Total packets:  %d", ble.packetCount);
    u8g2.drawStr(0, 32, buf);

    snprintf(buf, sizeof(buf), "RSSI min/avg/max: %d/%d/%d",
             ble.rssiMin, ble.rssiAvg, ble.rssiMax);
    u8g2.drawStr(0, 42, buf);

    snprintf(buf, sizeof(buf), "Connectable:  %d", ble.connectable);
    u8g2.drawStr(0, 52, buf);

    snprintf(buf, sizeof(buf), "NonConn:%d Scan:%d", ble.nonConnectable, ble.scannable);
    u8g2.drawStr(0, 62, buf);

    // Page indicator
    u8g2.drawStr(118, 62, "3/3");

    u8g2.sendBuffer();
}

void updateDisplay(WiFiScanResult& wifi, BLEScanResult& ble) {
    switch (displayPage) {
        case 0:
            displayPageWCI(wifi, ble);
            break;
        case 1:
            displayPageWiFi(wifi);
            break;
        case 2:
            displayPageBLE(ble);
            break;
    }
}

void outputCSV(unsigned long timestamp, WiFiScanResult& wifi, BLEScanResult& ble) {
    // Format: timestamp_ms,wifi_total,ch1,ch6,ch11,wifi_rssi_max,wifi_rssi_avg,
    //         ble_devices,ble_packets,ble_rssi_min,ble_rssi_max,ble_rssi_avg,
    //         ble_connectable,ble_nonconnectable,ble_scannable,wci
    Serial.printf("%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.1f\n",
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
                  ble.scannable,
                  lastWCI);
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("Starting RF Logger...");

    // Setup PRG button with interrupt
    pinMode(PRG_BUTTON, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PRG_BUTTON), onButtonPress, FALLING);

    Serial.println("Init OLED...");
    initOLED();

    Serial.println("Init WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    Serial.println("Init BLE...");
    initBLE();

    Serial.println("Init complete!");
    Serial.println("timestamp_ms,wifi_total,ch1,ch6,ch11,wifi_rssi_max,wifi_rssi_avg,ble_devices,ble_packets,ble_rssi_min,ble_rssi_max,ble_rssi_avg,ble_connectable,ble_nonconnectable,ble_scannable,wci");

    u8g2.clearBuffer();
    u8g2.drawStr(0, 10, "RF Logger");
    u8g2.drawStr(0, 24, "Ready!");
    u8g2.sendBuffer();
    delay(500);
}

// Store last results for display updates
WiFiScanResult lastWifiResult;
BLEScanResult lastBleResult;

void loop() {
    unsigned long loopStart = millis();

    // Check for button press (with debounce)
    if (buttonPressed) {
        unsigned long now = millis();
        if (now - lastButtonTime > DEBOUNCE_MS) {
            displayPage = (displayPage + 1) % NUM_PAGES;
            lastButtonTime = now;
            // Immediately update display with last results
            updateDisplay(lastWifiResult, lastBleResult);
        }
        buttonPressed = false;
    }

    WiFiScanResult wifiResult = performWiFiScan();
    BLEScanResult bleResult = performBLEScan();

    // Calculate WCI
    lastWCI = calculateWCI(wifiResult, bleResult);

    // Store for button-triggered updates
    lastWifiResult = wifiResult;
    lastBleResult = bleResult;

    scanCount++;

    outputCSV(loopStart, wifiResult, bleResult);
    updateDisplay(wifiResult, bleResult);

    unsigned long elapsed = millis() - loopStart;
    if (elapsed < SCAN_INTERVAL_MS) {
        delay(SCAN_INTERVAL_MS - elapsed);
    }
}

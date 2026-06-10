#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <RadioLib.h>
#include <SPI.h>
#include <vector>
#include <WiFi.h>
#include <WiFiUdp.h>

// ============================================================
// CC1101
// ============================================================
CC1101 radio = new Module(5, 12, RADIOLIB_NC, 27);

struct __attribute__((packed)) TpmsData {
    uint32_t timestamp;
    uint16_t statusCode;
    uint16_t volt_mV;
    uint16_t kpa_x10;
    uint8_t  tireId;
    int8_t   temp;
    int8_t   bleRssi;
    uint8_t  chksum;
};

// ============================================================
// TX QUEUE
// ============================================================
#define TX_QUEUE_SIZE  48
#define TX_INTERVAL_MS 300

TpmsData txQueue[TX_QUEUE_SIZE];
volatile int txHead = 0;
volatile int txTail = 0;
unsigned long lastTxTime = 0;
bool radioReady = false;

// ============================================================
// RADIO DIAGNOSTIC
// ============================================================
struct RadioDiag {
    uint32_t txCount    = 0;   // total paket dikirim
    uint32_t txOk       = 0;   // berhasil transmit
    uint32_t txErr      = 0;   // gagal transmit
    uint32_t txPerTire[12] = {0}; // per tire
};

RadioDiag diag;
unsigned long diagStartTime = 0;

void printRadioDiag() {
    unsigned long uptime = (millis() - diagStartTime) / 1000;
    float txRate = diag.txCount > 0
                   ? (diag.txOk * 100.0f / diag.txCount)
                   : 0.0f;

    Serial.println();
    Serial.println("╔══════════════════════════════════════╗");
    Serial.println("║       RADIO TX DIAGNOSTIC            ║");
    Serial.println("╠══════════════════════════════════════╣");
    Serial.printf( "║ Uptime       : %lu detik\n", uptime);
    Serial.printf( "║ Total TX     : %lu paket\n", diag.txCount);
    Serial.printf( "║ TX OK        : %lu\n", diag.txOk);
    Serial.printf( "║ TX ERR       : %lu\n", diag.txErr);
    Serial.printf( "║ TX Success   : %.1f%%\n", txRate);
    Serial.println("╠══════════════════════════════════════╣");
    Serial.println("║ Per-Tire TX Count:                   ║");
    for (int i = 0; i < 12; i++) {
        Serial.printf("║   Tire %-2d    : %lu paket\n",
                      i, diag.txPerTire[i]);
    }
    Serial.println("╚══════════════════════════════════════╝");
    Serial.println();
}

// ============================================================
// TX QUEUE FUNCTIONS
// ============================================================
bool txQueuePush(const TpmsData& d) {
    int next = (txHead + 1) % TX_QUEUE_SIZE;
    if (next == txTail) return false;
    txQueue[txHead] = d;
    txHead = next;
    return true;
}

bool txQueuePop(TpmsData& d) {
    if (txHead == txTail) return false;
    d = txQueue[txTail];
    txTail = (txTail + 1) % TX_QUEUE_SIZE;
    return true;
}

void initRadio() {
    SPI.begin(18, 19, 23, 5);
    int state = radio.begin(434.0, 9.6, 50.0, 250.0, 10, 32);
    if (state == RADIOLIB_ERR_NONE) {
        uint8_t syncWord[] = {0x12, 0xAD};
        radio.setSyncWord(syncWord, 2);
        radio.setCrcFiltering(false);
        radio.fixedPacketLengthMode(sizeof(TpmsData));
        radioReady = true;
        Serial.println("[CC1101] Radio siap.");
    } else {
        Serial.printf("[CC1101] Init gagal: %d\n", state);
    }
}

void processRadioQueue() {
    if (!radioReady) return;
    unsigned long now = millis();
    if (now - lastTxTime < TX_INTERVAL_MS) return;

    TpmsData d;
    if (!txQueuePop(d)) return;

    lastTxTime = millis();
    diag.txCount++;
    diag.txPerTire[d.tireId]++;

    int state = radio.transmit((uint8_t*)&d, sizeof(TpmsData));
    if (state == RADIOLIB_ERR_NONE) {
        diag.txOk++;
        Serial.printf("[TX OK] ID:%-2d | %.1fkPa | %dC | %.2fV | Queue:%d\n",
                      d.tireId, d.kpa_x10 / 10.0f, d.temp,
                      d.volt_mV / 1000.0f,
                      (txHead - txTail + TX_QUEUE_SIZE) % TX_QUEUE_SIZE);
    } else {
        diag.txErr++;
        Serial.printf("[TX ERR] ID:%d state:%d\n", d.tireId, state);
    }
}

// ============================================================
// WIFI + UDP
// ============================================================
const char* WIFI_SSID = "qutmarni";
const char* WIFI_PASS = "zxdh6690";

WiFiUDP udp;
IPAddress udpTargetIP;
const uint16_t UDP_PORT = 4210;

void sendUdpLog(const String& msg) {
    if (WiFi.status() != WL_CONNECTED) return;
    udp.beginPacket(udpTargetIP, UDP_PORT);
    udp.print(msg);
    udp.endPacket();
}

void logLine(const String& msg) {
    Serial.println(msg);
    sendUdpLog(msg);
}

void setupWifiUdp() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("[WIFI] Connecting");
    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
        delay(500); Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("[WIFI] IP: "); Serial.println(WiFi.localIP());
        udpTargetIP = WiFi.gatewayIP();
        udp.begin(UDP_PORT);
        sendUdpLog("[UDP] Ready. IP: " + WiFi.localIP().toString());
    } else {
        Serial.println("[WIFI] Gagal. UDP dinonaktifkan.");
    }
}

// ============================================================
// 12 SENSOR CONFIG
// ============================================================
#define TIRE_COUNT        12
#define PRINT_INTERVAL_MS 10000

const char* tireLabels[TIRE_COUNT] = {
    "Front Left",   "Front Left-2",  "Front Left-3",
    "Front Right",  "Front Right-2", "Front Right-3",
    "Rear Left",    "Rear Left-2",   "Rear Left-3",
    "Rear Right",   "Rear Right-2",  "Rear Right-3"
};

const char* tireCommands[TIRE_COUNT] = {
    "fl",   "fl-2", "fl-3",
    "fr",   "fr-2", "fr-3",
    "rl",   "rl-2", "rl-3",
    "rr",   "rr-2", "rr-3"
};

std::string pairedMacs[TIRE_COUNT];

// ============================================================
// DATA SENSOR
// ============================================================
struct TireData {
    float    voltage    = 0.0f;
    int      celsius    = 0;
    float    kpa        = 0.0f;
    float    psi        = 0.0f;
    String   statusStr  = "NO DATA";
    uint16_t statusCode = 0x0000;
    int      rssi       = 0;
    bool     hasData    = false;
    unsigned long lastSeen = 0;
};

TireData tireData[TIRE_COUNT];
unsigned long lastSeenBLE[TIRE_COUNT] = {0};
unsigned long lastPrintTime = 0;

// ============================================================
// SCAN MODE
// ============================================================
struct ScannedDevice { std::string mac; int rssi; };
std::vector<ScannedDevice> scannedTpms;
bool scanTpmsMode = false;
bool scanTpmsDone = false;

void onScanDone(NimBLEScanResults results) {
    scanTpmsMode = false;
    scanTpmsDone = true;
}

// ============================================================
// FLASH
// ============================================================
Preferences preferences;

void saveMacToFlash(int index, String mac) {
    preferences.begin("tpms", false);
    preferences.putString(("mac" + String(index)).c_str(), mac);
    preferences.end();
}

void loadMacsFromFlash() {
    preferences.begin("tpms", true);
    for (int i = 0; i < TIRE_COUNT; i++) {
        String saved = preferences.getString(("mac" + String(i)).c_str(), "");
        if (saved.length() > 0) {
            saved.toLowerCase();
            pairedMacs[i] = std::string(saved.c_str());
        }
    }
    preferences.end();
}

// ============================================================
// HELPER
// ============================================================
int findTireIndex(const std::string& addr) {
    for (int i = 0; i < TIRE_COUNT; i++)
        if (!pairedMacs[i].empty() && pairedMacs[i] == addr) return i;
    return -1;
}

bool isAlreadyPaired(const std::string& addr) { return findTireIndex(addr) != -1; }

int getIndexFromCommand(String pos) {
    for (int i = 0; i < TIRE_COUNT; i++)
        if (pos == tireCommands[i]) return i;
    return -1;
}

String getStatusText(uint16_t code) {
    switch (code) {
        case 0x0002: return "STABLE_FAST";
        case 0x0003: return "STABLE_SLOW";
        case 0x0004: return "DOWN";
        case 0x0005: return "UP";
        case 0x0006: return "TRANSITION";
        case 0x0008: return "IDLE";
        default:
            char buf[12];
            snprintf(buf, sizeof(buf), "UNK_0x%04X", code);
            return String(buf);
    }
}

void printValidPositions() {
    Serial.println("  fl, fl-2, fl-3 | fr, fr-2, fr-3");
    Serial.println("  rl, rl-2, rl-3 | rr, rr-2, rr-3");
}

// ============================================================
// PRINT TABLE
// ============================================================
void printAllTires() {
    bool adaData = false;
    for (int i = 0; i < TIRE_COUNT; i++)
        if (tireData[i].hasData || !pairedMacs[i].empty()) { adaData = true; break; }
    if (!adaData) return;

    unsigned long detik = millis() / 1000;
    Serial.println();
    Serial.println("==========================================================================");
    Serial.printf( "TPMS REPORT | Uptime %02lu:%02lu:%02lu\n",
                   detik/3600, (detik%3600)/60, detik%60);
    Serial.println("==========================================================================");
    Serial.printf("| %-13s | %-11s | %-6s | %-5s | %-4s | %-7s | %-5s | %-4s |\n",
                  "Tire","Status","Code","Volt","Temp","KPa","PSI","RSSI");
    Serial.println("==========================================================================");

    for (int i = 0; i < TIRE_COUNT; i++) {
        if (pairedMacs[i].empty()) continue;
        if (tireData[i].hasData) {
            unsigned long selisih = (millis() - tireData[i].lastSeen) / 1000;
            char codeStr[8];
            snprintf(codeStr, sizeof(codeStr), "0x%04X", tireData[i].statusCode);
            Serial.printf("| %-13s | %-11s | %-6s | %-5.2f | %-4d | %-7.1f | %-5.1f | %-4d | %lus ago\n",
                          tireLabels[i], tireData[i].statusStr.c_str(), codeStr,
                          tireData[i].voltage, tireData[i].celsius,
                          tireData[i].kpa, tireData[i].psi,
                          tireData[i].rssi, selisih);
        } else {
            Serial.printf("| %-13s | %-11s | %-6s | %-5s | %-4s | %-7s | %-5s | %-4s |\n",
                          tireLabels[i], "NO DATA", "-", "-", "-", "-", "-", "-");
        }
    }
    Serial.println("==========================================================================");
    Serial.println();
}

// ============================================================
// BLE CALLBACK
// ============================================================
class MyAdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* advertisedDevice) {

        if (!advertisedDevice->isAdvertisingService(NimBLEUUID((uint16_t)0xA827))) return;

        std::string addr = advertisedDevice->getAddress().toString();

        if (scanTpmsMode) {
            if (isAlreadyPaired(addr)) return;
            for (auto& d : scannedTpms) if (d.mac == addr) return;
            std::string mfr = advertisedDevice->getManufacturerData();
            String rawHex = "";
            for (size_t i = 0; i < mfr.length(); i++) {
                char buf[4];
                snprintf(buf, sizeof(buf), "%02X ", (uint8_t)mfr[i]);
                rawHex += buf;
            }
            scannedTpms.push_back({addr, advertisedDevice->getRSSI()});
            logLine("[SCAN] " + String(addr.c_str()) +
                    " | RSSI:" + String(advertisedDevice->getRSSI()) +
                    " | Raw:" + rawHex);
            return;
        }

        int idx = findTireIndex(addr);
        if (idx == -1) return;

        std::string mfr = advertisedDevice->getManufacturerData();
        if (mfr.length() < 6) return;

        uint8_t* data = (uint8_t*)mfr.data();

        uint16_t statusCode = ((uint16_t)data[1] << 8) | data[0];
        float    voltage    = data[2] / 60.0f;
        int      celsius    = (int)data[3] - 50;
        uint16_t kpaAbs     = ((uint16_t)data[4] << 8) | data[5];

        float kpaGauge = 0.0f;
        float psi      = 0.0f;
        if (kpaAbs > 101) {
            kpaGauge = kpaAbs - 101.3f;
            psi      = kpaGauge * 0.145f;
            if (kpaGauge < 0.0f) kpaGauge = 0.0f;
            if (psi      < 0.0f) psi      = 0.0f;
        }

        tireData[idx].statusCode = statusCode;
        tireData[idx].statusStr  = getStatusText(statusCode);
        tireData[idx].voltage    = voltage;
        tireData[idx].celsius    = celsius;
        tireData[idx].kpa        = kpaGauge;
        tireData[idx].psi        = psi;
        tireData[idx].rssi       = advertisedDevice->getRSSI();
        tireData[idx].hasData    = true;
        tireData[idx].lastSeen   = millis();

        TpmsData pkt;
        pkt.timestamp  = millis();
        pkt.statusCode = statusCode;
        pkt.volt_mV    = (uint16_t)(voltage * 1000.0f);
        pkt.kpa_x10    = (uint16_t)(kpaGauge * 10.0f);
        pkt.tireId     = (uint8_t)idx;
        pkt.temp       = (int8_t)celsius;
        pkt.bleRssi    = (int8_t)advertisedDevice->getRSSI();

        uint8_t* p = (uint8_t*)&pkt;
        uint8_t cs = 0;
        for (int i = 0; i < (int)sizeof(TpmsData) - 1; i++) cs ^= p[i];
        pkt.chksum = cs;

        if (!txQueuePush(pkt))
            Serial.printf("[WARN] TX queue penuh, ID:%d dibuang\n", idx);

        unsigned long now_ms = millis();
        char codeStr[8];
        snprintf(codeStr, sizeof(codeStr), "0x%04X", statusCode);
        String logMsg = "[BLE] " + String(tireLabels[idx]) +
                        " | " + String(codeStr) +
                        " | " + tireData[idx].statusStr +
                        " | " + String(kpaGauge, 1) + "kPa" +
                        " | " + String(celsius) + "C" +
                        " | " + String(voltage, 2) + "V" +
                        " | RSSI:" + String(advertisedDevice->getRSSI());
        if (lastSeenBLE[idx] > 0)
            logMsg += " | " + String((now_ms - lastSeenBLE[idx]) / 1000.0f, 1) + "s";
        logLine(logMsg);

        lastSeenBLE[idx] = now_ms;
        NimBLEDevice::getScan()->clearResults();
    }
};

// ============================================================
// SCAN MAINTENANCE
// ============================================================
unsigned long lastScanRestart = 0;
#define SCAN_RESTART_MS 120000

void maintainScan() {
    if (scanTpmsMode) return;
    unsigned long now = millis();
    if (now - lastScanRestart < SCAN_RESTART_MS) return;
    lastScanRestart = now;
    NimBLEDevice::getScan()->stop();
    NimBLEDevice::getScan()->clearResults();
    NimBLEDevice::getScan()->start(0, nullptr, false);
}

// ============================================================
// SERIAL COMMAND
// ============================================================
void handleSerial() {
    if (!Serial.available()) return;
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toLowerCase();

    if (input == "scan_tpms") {
        scannedTpms.clear();
        scanTpmsMode = true;
        scanTpmsDone = false;
        logLine("[INFO] Scanning TPMS 120 detik...");
        NimBLEDevice::getScan()->stop();
        NimBLEDevice::getScan()->clearResults();
        NimBLEDevice::getScan()->start(120, onScanDone, false);
        return;
    }

    if (input == "diag") {
        printRadioDiag();
        return;
    }

    if (input == "diag_reset") {
        diag = RadioDiag{};
        diagStartTime = millis();
        Serial.println("[OK] Diagnostic direset.");
        return;
    }

    if (input == "unpair_all") {
        for (int i = 0; i < TIRE_COUNT; i++) {
            pairedMacs[i] = "";
            saveMacToFlash(i, "");
            tireData[i]    = TireData{};
            lastSeenBLE[i] = 0;
        }
        logLine("[OK] Semua sensor berhasil di-unpair.");
        return;
    }

    if (input.startsWith("p-")) {
        int spaceIdx = input.indexOf(' ');
        if (spaceIdx == -1) {
            Serial.println("[ERROR] Format: p-fl <MAC>");
            printValidPositions();
            return;
        }
        String pos = input.substring(2, spaceIdx);
        String mac = input.substring(spaceIdx + 1);
        mac.trim(); mac.toLowerCase();
        if (mac.length() != 17) {
            Serial.println("[ERROR] Format MAC salah.");
            return;
        }
        int idx = getIndexFromCommand(pos);
        if (idx == -1) {
            Serial.println("[ERROR] Posisi tidak valid.");
            printValidPositions();
            return;
        }
        if (!pairedMacs[idx].empty()) {
            Serial.printf("[ERROR] %s sudah dipair. Ketik 'unpair-%s' dulu.\n",
                          tireLabels[idx], pos.c_str());
            return;
        }
        if (isAlreadyPaired(std::string(mac.c_str()))) {
            Serial.println("[ERROR] MAC sudah dipair ke posisi lain.");
            return;
        }
        pairedMacs[idx] = std::string(mac.c_str());
        saveMacToFlash(idx, mac);
        tireData[idx]    = TireData{};
        lastSeenBLE[idx] = 0;
        logLine("[OK] " + String(tireLabels[idx]) + " dipasangkan ke: " + mac);
        return;
    }

    if (input.startsWith("unpair-")) {
        String pos = input.substring(7);
        int idx = getIndexFromCommand(pos);
        if (idx == -1) { Serial.println("[ERROR] Posisi tidak valid."); return; }
        if (pairedMacs[idx].empty()) {
            Serial.printf("[ERROR] %s belum dipasangkan.\n", tireLabels[idx]);
            return;
        }
        pairedMacs[idx] = "";
        saveMacToFlash(idx, "");
        tireData[idx]    = TireData{};
        lastSeenBLE[idx] = 0;
        logLine("[OK] " + String(tireLabels[idx]) + " berhasil di-unpair.");
        return;
    }

    if (input == "status") {
        Serial.println("\n[STATUS] Sensor terdaftar:");
        for (int i = 0; i < TIRE_COUNT; i++)
            Serial.printf("  %-14s : %s\n", tireLabels[i],
                          pairedMacs[i].empty() ? "(belum dipasang)" : pairedMacs[i].c_str());
        return;
    }

    if (input == "report") { printAllTires(); return; }

    Serial.println("[ERROR] Perintah tersedia:");
    Serial.println("  scan_tpms    — cari sensor TPMS");
    Serial.println("  p-<pos> <MAC>— pair sensor");
    Serial.println("  unpair-<pos> — lepas pairing satu");
    Serial.println("  unpair_all   — lepas semua pairing");
    Serial.println("  status       — sensor terdaftar");
    Serial.println("  report       — tabel data sensor");
    Serial.println("  diag         — radio TX diagnostic");
    Serial.println("  diag_reset   — reset counter diagnostic");
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== TPMS BLE + CC1101 GATEWAY ===");

    initRadio();
    setupWifiUdp();
    loadMacsFromFlash();

    Serial.println("\nPerintah: scan_tpms | p-<pos> <MAC> | unpair-<pos> | unpair_all | status | report | diag | diag_reset");
    printValidPositions();
    Serial.println();

    bool adaPairing = false;
    for (int i = 0; i < TIRE_COUNT; i++)
        if (!pairedMacs[i].empty()) { adaPairing = true; break; }

    if (adaPairing) {
        Serial.println("[INFO] Pairing tersimpan:");
        for (int i = 0; i < TIRE_COUNT; i++)
            if (!pairedMacs[i].empty())
                Serial.printf("  %-14s : %s\n", tireLabels[i], pairedMacs[i].c_str());
        Serial.println();
    }

    NimBLEDevice::init("");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), false);
    pScan->setActiveScan(false);
    pScan->setInterval(625);
    pScan->setWindow(625);
    pScan->setDuplicateFilter(false);
    pScan->start(0, nullptr, false);

    diagStartTime = millis();
    lastScanRestart = millis();
    Serial.println("[BLE] Scanner aktif.");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
    handleSerial();
    processRadioQueue();
    maintainScan();

    unsigned long now = millis();
    if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
        lastPrintTime = now;
        printAllTires();
    }

    if (scanTpmsDone) {
        scanTpmsDone = false;
        if (scannedTpms.empty()) {
            logLine("[INFO] Tidak ada sensor TPMS ditemukan.");
        } else {
            Serial.printf("\n[HASIL] %d sensor TPMS ditemukan:\n", scannedTpms.size());
            for (auto& d : scannedTpms)
                logLine("  " + String(d.mac.c_str()) + " | RSSI: " + String(d.rssi) + " dBm");
            Serial.println("[TIP] Gunakan: p-fl <MAC> untuk pair sensor");
        }
        NimBLEDevice::getScan()->clearResults();
        NimBLEDevice::getScan()->start(0, nullptr, false);
    }
}
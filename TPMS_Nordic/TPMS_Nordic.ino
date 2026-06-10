/*
 * TPMS Scanner untuk nRF52840 DK
 * Dikonversi dari ESP32 + NimBLE ke Adafruit nRF52 BSP (Bluefruit52Lib)
 *
 * Board  : Nordic nRF52840 DK (adafruit:nrf52:pca10056)
 * BSP    : Adafruit nRF52 v1.7.0
 * Serial : USB Serial (Serial monitor Arduino, 115200 baud)
 *
 * Fitur:
 *   scan_tpms              — Scan sensor TPMS baru selama 30 detik
 *   p-<posisi> <MAC>       — Pair sensor ke posisi ban
 *   unpair-<posisi>        — Lepas pairing
 *   status                 — Lihat semua pairing
 *   report                 — Cetak tabel data terbaru
 *
 * Perbedaan dari versi ESP32:
 *   - NimBLE       → Adafruit Bluefruit52Lib (BLEScanner)
 *   - Preferences  → InternalFileSystem (LittleFS)
 *   - ESP TX Power → Bluefruit.setTxPower(8)
 *   - Filter UUID  → manual di callback (cek manufacturer data)
 */

#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
// TIDAK pakai <vector> — nRF52 BSP pakai --specs=nano.specs yang
// strip exception handling; std::vector butuh std::__throw_length_error
// Solusi: array statis dengan counter manual.

using namespace Adafruit_LittleFS_Namespace;

// ============================================================
// 12 SENSOR CONFIG
// ============================================================
#define TIRE_COUNT        12
#define PRINT_INTERVAL_MS 10000   // cetak tabel tiap 10 detik
#define SCAN_DURATION_S   10      // durasi scan TPMS (detik) — ubah sesuai kebutuhan
#define MAC_FILE_PREFIX   "/mac"  // prefix nama file di LittleFS

const char* tireLabels[TIRE_COUNT] = {
    "Front Left",    "Front Left-2",  "Front Left-3",
    "Front Right",   "Front Right-2", "Front Right-3",
    "Rear Left",     "Rear Left-2",   "Rear Left-3",
    "Rear Right",    "Rear Right-2",  "Rear Right-3"
};

const char* tireCommands[TIRE_COUNT] = {
    "fl",    "fl-2",  "fl-3",
    "fr",    "fr-2",  "fr-3",
    "rl",    "rl-2",  "rl-3",
    "rr",    "rr-2",  "rr-3"
};

// MAC tersimpan per slot ban (format lowercase "xx:xx:xx:xx:xx:xx")
String pairedMacs[TIRE_COUNT];

// ============================================================
// DATA SENSOR
// ============================================================
struct TireData {
    float    voltage   = 0.0f;
    int      celsius   = 0;
    float    kpa       = 0.0f;
    float    psi       = 0.0f;
    String   statusStr = "NO DATA";
    uint16_t statusCode= 0x0000;
    int      rssi      = 0;
    bool     hasData   = false;
    unsigned long lastSeen = 0;
};

TireData tireData[TIRE_COUNT];
unsigned long lastSeenBLE[TIRE_COUNT] = {0};
unsigned long lastPrintTime = 0;

// ============================================================
// SCAN TPMS
// ============================================================
#define MAX_SCAN_RESULTS 30   // maksimal sensor yang bisa ditampung saat scan

struct ScannedDevice {
    char mac[18];  // "xx:xx:xx:xx:xx:xx" + null
    int  rssi;
};

// Array statis — tidak pakai std::vector karena tidak kompatibel
// dengan --specs=nano.specs (no exception support)
ScannedDevice scannedTpms[MAX_SCAN_RESULTS];
int           scannedCount = 0;
bool scanTpmsMode = false;
bool scanTpmsDone = false;

// Helper pengganti vector.clear() / push_back() / empty() / size()
void scannedClear() { scannedCount = 0; }
bool scannedEmpty() { return scannedCount == 0; }

bool scannedContains(const char* mac) {
    for (int i = 0; i < scannedCount; i++) {
        if (strcmp(scannedTpms[i].mac, mac) == 0) return true;
    }
    return false;
}

void scannedAdd(const char* mac, int rssi) {
    if (scannedCount >= MAX_SCAN_RESULTS) return;
    strncpy(scannedTpms[scannedCount].mac, mac, 17);
    scannedTpms[scannedCount].mac[17] = '\0';
    scannedTpms[scannedCount].rssi = rssi;
    scannedCount++;
}

// ============================================================
// HELPER
// ============================================================
int findTireIndex(const String& addr) {
    for (int i = 0; i < TIRE_COUNT; i++) {
        if (pairedMacs[i].length() > 0 && pairedMacs[i] == addr) {
            return i;
        }
    }
    return -1;
}

bool isAlreadyPaired(const String& addr) {
    return findTireIndex(addr) != -1;
}

int getIndexFromCommand(const String& pos) {
    for (int i = 0; i < TIRE_COUNT; i++) {
        if (pos == tireCommands[i]) {
            return i;
        }
    }
    return -1;
}

String getStatusText(uint16_t code) {
    switch (code) {
        case 0x0002: return "STABLE_FAST";
        case 0x0003: return "STABLE_SLOW";
        case 0x0004: return "DOWN";
        case 0x0005: return "UP";
        case 0x0006: return "FAST";
        case 0x0008: return "NORMAL";
        default:     return "UNKNOWN";
    }
}

void printValidPositions() {
    Serial.println("Posisi valid:");
    Serial.println("  fl, fl-2, fl-3");
    Serial.println("  fr, fr-2, fr-3");
    Serial.println("  rl, rl-2, rl-3");
    Serial.println("  rr, rr-2, rr-3");
}

// ============================================================
// FLASH — Simpan & Load pairing via LittleFS
//
// Setiap slot ban disimpan sebagai file kecil:
//   /mac0  berisi MAC slot 0
//   /mac1  berisi MAC slot 1
//   dst.
// ============================================================
void saveMacToFlash(int index, const String& mac) {
    String path = String(MAC_FILE_PREFIX) + index;

    File f = InternalFS.open(path.c_str(), FILE_O_WRITE);
    if (!f) {
        Serial.printf("[FLASH] Gagal buka %s untuk tulis\n", path.c_str());
        return;
    }
    f.write((const uint8_t*)mac.c_str(), mac.length());
    f.close();
}

void loadMacsFromFlash() {
    for (int i = 0; i < TIRE_COUNT; i++) {
        String path = String(MAC_FILE_PREFIX) + i;

        File f = InternalFS.open(path.c_str(), FILE_O_READ);
        if (!f) continue;  // file tidak ada → belum pernah dipair

        uint32_t sz = f.size();
        if (sz == 0 || sz > 17) { f.close(); continue; }

        char buf[18] = {0};
        f.read((uint8_t*)buf, sz);
        f.close();

        String mac = String(buf);
        mac.trim();
        mac.toLowerCase();

        if (mac.length() == 17) {
            pairedMacs[i] = mac;
        }
    }
}

// ============================================================
// PRINT TABLE
// ============================================================
void printAllTires() {
    bool adaData = false;
    for (int i = 0; i < TIRE_COUNT; i++) {
        if (tireData[i].hasData || pairedMacs[i].length() > 0) {
            adaData = true;
            break;
        }
    }
    if (!adaData) return;

    unsigned long detik = millis() / 1000;
    unsigned int jam    = detik / 3600;
    unsigned int menit  = (detik % 3600) / 60;
    unsigned int dtk    = detik % 60;

    Serial.println();
    Serial.println("====================================================================================================================");
    Serial.printf("TPMS REPORT | Uptime %02d:%02d:%02d\n", jam, menit, dtk);
    Serial.println("====================================================================================================================");
    Serial.printf("| %-16s | %-11s | %-6s | %-4s | %-4s | %-6s | %-6s | %-5s | %-11s |\n",
                  "Tire", "Status", "Code", "Volt", "Temp", "KPa", "PSI", "RSSI", "Last Update");
    Serial.println("====================================================================================================================");

    for (int i = 0; i < TIRE_COUNT; i++) {
        if (pairedMacs[i].length() == 0) continue;

        if (tireData[i].hasData) {
            unsigned long selisih = (millis() - tireData[i].lastSeen) / 1000;
            char codeStr[8];
            snprintf(codeStr, sizeof(codeStr), "0x%04X", tireData[i].statusCode);

            Serial.printf("| %-16s | %-11s | %-6s | %-4.2f | %-4d | %-6.1f | %-6.1f | %-5d | %-11lus |\n",
                          tireLabels[i],
                          tireData[i].statusStr.c_str(),
                          codeStr,
                          tireData[i].voltage,
                          tireData[i].celsius,
                          tireData[i].kpa,
                          tireData[i].psi,
                          tireData[i].rssi,
                          selisih);
        } else {
            Serial.printf("| %-16s | %-11s | %-6s | %-4s | %-4s | %-6s | %-6s | %-5s | %-11s |\n",
                          tireLabels[i], "NO DATA", "-", "-", "-", "-", "-", "-", "-");
        }
    }

    Serial.println("====================================================================================================================");
    Serial.println();
}

// ============================================================
// BLE SCAN CALLBACK
//
// Adafruit Bluefruit52Lib menggunakan pendekatan berbeda dari NimBLE.
// Kita daftarkan callback lewat:
//   Bluefruit.Scanner.setRxCallback(scanCallback);
//
// Parameter:  report → struct ble_gap_evt_adv_report_t
//             Kita parse sendiri manufacturer data & UUID.
//
// UUID TPMS sensor target: 0xA827 (little-endian di AD type 0x03 / 0x02)
// ============================================================

// Cek apakah report mengandung service UUID 0xA827
bool hasServiceUUID_A827(const uint8_t* data, uint8_t len) {
    uint8_t i = 0;
    while (i < len) {
        uint8_t fieldLen  = data[i];
        if (fieldLen == 0 || i + fieldLen >= len) break;
        uint8_t fieldType = data[i + 1];
        // Type 0x02 = Incomplete 16-bit UUIDs
        // Type 0x03 = Complete 16-bit UUIDs
        if (fieldType == 0x02 || fieldType == 0x03) {
            for (uint8_t j = 2; j + 1 <= fieldLen; j += 2) {
                uint16_t uuid = data[i + j] | ((uint16_t)data[i + j + 1] << 8);
                if (uuid == 0xA827) return true;
            }
        }
        i += fieldLen + 1;
    }
    return false;
}

// Ambil manufacturer data dari advertising payload
// Kembalikan pointer ke payload manufacturer, isi *outLen
const uint8_t* getManufacturerData(const uint8_t* data, uint8_t len, uint8_t* outLen) {
    uint8_t i = 0;
    while (i < len) {
        uint8_t fieldLen  = data[i];
        if (fieldLen == 0 || i + fieldLen >= len) break;
        uint8_t fieldType = data[i + 1];
        if (fieldType == 0xFF) {  // Manufacturer Specific Data
            *outLen = fieldLen - 1;
            return &data[i + 2];
        }
        i += fieldLen + 1;
    }
    *outLen = 0;
    return nullptr;
}

void scanCallback(ble_gap_evt_adv_report_t* report) {
    // Ambil payload mentah
    const uint8_t* payload = report->data.p_data;
    uint8_t        payLen  = report->data.len;

    // Format MAC address jadi string lowercase "xx:xx:xx:xx:xx:xx"
    const uint8_t* m = report->peer_addr.addr;  // little-endian (LSB first)
    char macBuf[18];
    snprintf(macBuf, sizeof(macBuf), "%02x:%02x:%02x:%02x:%02x:%02x",
             m[5], m[4], m[3], m[2], m[1], m[0]);
    String addr = String(macBuf);

    int rssi = report->rssi;

    // ---- MODE SCAN TPMS ----
    // Filter ringan: hanya device dengan prefix MAC b9:41:fa (TPMS sensor ini)
    // Ubah prefix jika sensor Anda berbeda.
    // Untuk dump semua device: hapus blok if prefix di bawah.
    if (scanTpmsMode) {
        // Filter prefix MAC — sesuaikan jika MAC sensor berbeda
        if (strncmp(macBuf, "b9:41:fa", 8) != 0) {
            Bluefruit.Scanner.resume();
            return;
        }
        if (isAlreadyPaired(addr)) {
            Bluefruit.Scanner.resume();
            return;
        }
        if (scannedContains(macBuf)) {
            Bluefruit.Scanner.resume();
            return;
        }

        scannedAdd(macBuf, rssi);

        // Dump SELURUH raw payload AD structure
        // supaya kita tahu persis di mana data TPMS disimpan
        Serial.printf("[SCAN] %s | RSSI: %3d dBm | payLen:%d | RAW:", macBuf, rssi, payLen);
        for (int i = 0; i < payLen; i++) Serial.printf(" %02X", payload[i]);
        Serial.println();

        // Bantu parse: tampilkan tiap AD field dengan type-nya
        {
            uint8_t i = 0;
            while (i < payLen) {
                uint8_t fLen  = payload[i];
                if (fLen == 0 || i + fLen >= payLen) break;
                uint8_t fType = payload[i + 1];
                Serial.printf("  AD type=0x%02X len=%d data:", fType, fLen - 1);
                for (uint8_t j = 2; j <= fLen; j++) Serial.printf(" %02X", payload[i + j]);
                Serial.println();
                i += fLen + 1;
            }
        }

        Bluefruit.Scanner.resume();
        return;
    }

    // ---- MODE MONITORING ----
    // Filter: hanya proses MAC yang sudah dipair — tidak perlu cek UUID
    int idx = findTireIndex(addr);
    if (idx == -1) {
        Bluefruit.Scanner.resume();
        return;
    }

    // Coba manufacturer data (AD 0xFF) dulu
    uint8_t mfrLen = 0;
    const uint8_t* mfr = getManufacturerData(payload, payLen, &mfrLen);

    // Jika tidak ada / kurang, coba service data (AD 0x16) — beberapa TPMS pakai ini
    if (!mfr || mfrLen < 11) {
        uint8_t i = 0;
        while (i < payLen) {
            uint8_t fLen  = payload[i];
            if (fLen == 0 || i + fLen >= payLen) break;
            uint8_t fType = payload[i + 1];
            if (fType == 0x16 && (fLen - 1) >= 11) {  // Service Data, skip 2 byte UUID
                mfr    = &payload[i + 2];
                mfrLen = fLen - 1;
                break;
            }
            i += fLen + 1;
        }
    }

    // Masih tidak ada data yang cukup → skip
    if (!mfr || mfrLen < 11) {
        Bluefruit.Scanner.resume();
        return;
    }

    /*
      Layout manufacturer data (sama persis dengan versi ESP32):
        Byte 0-1 : status code, little-endian
        Byte 2   : voltage raw  (÷60 → volt)
        Byte 3   : suhu raw     (−50 → °C)
        Byte 4-5 : tekanan kPa absolut, big-endian
    */
    uint16_t statusCode = ((uint16_t)mfr[1] << 8) | mfr[0];
    tireData[idx].statusCode = statusCode;
    tireData[idx].statusStr  = getStatusText(statusCode);
    tireData[idx].voltage    = mfr[2] / 60.0f;
    tireData[idx].celsius    = mfr[3] - 50;

    uint16_t kpaAbsolut = ((uint16_t)mfr[4] << 8) | mfr[5];
    if (kpaAbsolut < 100) {
        tireData[idx].kpa = 0.0f;
        tireData[idx].psi = 0.0f;
    } else {
        tireData[idx].kpa = kpaAbsolut - 101.3f;
        tireData[idx].psi = tireData[idx].kpa * 0.145f;
        if (tireData[idx].kpa < 0.0f) tireData[idx].kpa = 0.0f;
        if (tireData[idx].psi < 0.0f) tireData[idx].psi = 0.0f;
    }

    tireData[idx].rssi     = rssi;
    tireData[idx].hasData  = true;
    tireData[idx].lastSeen = millis();

    unsigned long now_ms = millis();
    char codeStr[8];
    snprintf(codeStr, sizeof(codeStr), "0x%04X", statusCode);

    if (lastSeenBLE[idx] > 0) {
        unsigned long interval_ms = now_ms - lastSeenBLE[idx];
        Serial.printf("[BLE RECEIVE] %-16s | Code: %-6s | Status: %-11s | RSSI: %d dBm | Interval: %.2f detik | %.1f kPa | %.1f PSI\n",
                      tireLabels[idx], codeStr, tireData[idx].statusStr.c_str(),
                      rssi, interval_ms / 1000.0f,
                      tireData[idx].kpa, tireData[idx].psi);
    } else {
        Serial.printf("[BLE RECEIVE] %-16s | Code: %-6s | Status: %-11s | RSSI: %d dBm | Interval: pertama kali | %.1f kPa | %.1f PSI\n",
                      tireLabels[idx], codeStr, tireData[idx].statusStr.c_str(),
                      rssi, tireData[idx].kpa, tireData[idx].psi);
    }

    lastSeenBLE[idx] = now_ms;

    Bluefruit.Scanner.resume();
}

// Dipanggil ketika scan selesai (durasi habis)
void scanStopCallback() {
    if (scanTpmsMode) {
        scanTpmsMode = false;
        scanTpmsDone = true;
    }
}

// ============================================================
// SERIAL COMMAND HANDLER
// ============================================================
void handleSerial() {
    if (!Serial.available()) return;

    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toLowerCase();

    // ---- scan_tpms ----
    if (input == "scan_tpms") {
        scannedClear();
        scanTpmsMode = true;
        scanTpmsDone = false;

        Serial.printf("[INFO] Scanning TPMS %d detik...\n", SCAN_DURATION_S);

        // Stop scan aktif, lalu mulai scan durasi terbatas
        Bluefruit.Scanner.stop();
        Bluefruit.Scanner.start(SCAN_DURATION_S);  // detik; 0 = non-stop
        return;
    }

    // ---- p-<posisi> <MAC> ----
    if (input.startsWith("p-")) {
        int spaceIdx = input.indexOf(' ');
        if (spaceIdx == -1) {
            Serial.println("[ERROR] Format: p-fl <MAC>");
            printValidPositions();
            return;
        }

        String pos = input.substring(2, spaceIdx);
        String mac = input.substring(spaceIdx + 1);
        mac.trim();
        mac.toLowerCase();

        if (mac.length() != 17) {
            Serial.println("[ERROR] Format MAC salah. Contoh: b9:41:fa:00:04:d0");
            return;
        }

        int idx = getIndexFromCommand(pos);
        if (idx == -1) {
            Serial.println("[ERROR] Posisi tidak valid.");
            printValidPositions();
            return;
        }

        if (pairedMacs[idx].length() > 0) {
            Serial.printf("[ERROR] %s sudah di-pair ke %s. Ketik 'unpair-%s' dulu.\n",
                          tireLabels[idx], pairedMacs[idx].c_str(), pos.c_str());
            return;
        }

        if (isAlreadyPaired(mac)) {
            Serial.println("[ERROR] MAC ini sudah dipair ke sensor lain.");
            return;
        }

        pairedMacs[idx] = mac;
        saveMacToFlash(idx, mac);
        tireData[idx]    = TireData{};
        lastSeenBLE[idx] = 0;

        Serial.printf("[OK] %s dipasangkan ke: %s\n", tireLabels[idx], pairedMacs[idx].c_str());
        return;
    }

    // ---- unpair-<posisi> ----
    if (input.startsWith("unpair-")) {
        String pos = input.substring(7);
        int idx = getIndexFromCommand(pos);

        if (idx == -1) {
            Serial.println("[ERROR] Posisi tidak valid.");
            printValidPositions();
            return;
        }

        if (pairedMacs[idx].length() == 0) {
            Serial.printf("[ERROR] %s memang belum dipasangkan.\n", tireLabels[idx]);
            return;
        }

        pairedMacs[idx] = "";
        saveMacToFlash(idx, "");   // tulis string kosong → hapus pairing
        tireData[idx]    = TireData{};
        lastSeenBLE[idx] = 0;

        Serial.printf("[OK] %s berhasil di-unpair.\n", tireLabels[idx]);
        return;
    }

    // ---- status ----
    if (input == "status") {
        Serial.println("\n[STATUS] Sensor terdaftar:");
        for (int i = 0; i < TIRE_COUNT; i++) {
            Serial.printf("  %-16s : %s\n",
                          tireLabels[i],
                          pairedMacs[i].length() == 0 ? "(belum dipasang)" : pairedMacs[i].c_str());
        }
        return;
    }

    // ---- report ----
    if (input == "report") {
        printAllTires();
        return;
    }

    // ---- help / unknown ----
    Serial.println("[ERROR] Perintah:");
    Serial.println("  scan_tpms");
    Serial.println("  p-<posisi> <MAC>   contoh: p-fl b9:41:fa:00:04:d0");
    Serial.println("  unpair-<posisi>    contoh: unpair-fl");
    Serial.println("  status");
    Serial.println("  report");
    Serial.println();
    printValidPositions();
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);

    // Tunggu Serial siap (USB CDC di nRF52 butuh sedikit waktu)
    // Timeout 3 detik agar tidak hang jika tidak ada PC terhubung
    for (int i = 0; i < 300 && !Serial; i++) delay(10);

    // ---- Init LittleFS ----
    if (!InternalFS.begin()) {
        Serial.println("[ERROR] LittleFS gagal mount!");
        // Tidak fatal — lanjut tanpa persistensi
    }

    // ---- Load pairing dari flash ----
    loadMacsFromFlash();

    // ---- Banner ----
    Serial.println();
    Serial.println("Memulai Scanner TPMS (12 SENSOR - nRF52840 DK)...");
    Serial.println("1. Ketik 'scan_tpms' untuk mencari sensor.");
    Serial.println("2. Ketik 'p-<posisi> <MAC>' untuk pair sensor.");
    Serial.println("3. Ketik 'status' untuk melihat sensor terdaftar.");
    Serial.println("4. Ketik 'report' untuk cetak tabel manual.");
    Serial.println();
    printValidPositions();
    Serial.println();

    bool adaPairing = false;
    for (int i = 0; i < TIRE_COUNT; i++) {
        if (pairedMacs[i].length() > 0) { adaPairing = true; break; }
    }
    if (adaPairing) {
        Serial.println("[INFO] Pairing tersimpan ditemukan:");
        for (int i = 0; i < TIRE_COUNT; i++) {
            if (pairedMacs[i].length() > 0)
                Serial.printf("  %-16s : %s\n", tireLabels[i], pairedMacs[i].c_str());
        }
        Serial.println();
    }

    // ---- Init Bluefruit / BLE ----
    Bluefruit.begin();
    Bluefruit.setTxPower(8);        // max TX power nRF52840 (+8 dBm ≈ ESP_PWR_LVL_P9)
    Bluefruit.setName("TPMS-nRF52");

    // Konfigurasi scanner
    Bluefruit.Scanner.setRxCallback(scanCallback);
    Bluefruit.Scanner.useActiveScan(true);      // active scan = minta scan response juga
    Bluefruit.Scanner.setInterval(160, 160);    // interval & window dalam unit 0.625ms
                                                // 160 × 0.625ms = 100ms → duty cycle 100%
                                                // (setara setInterval(50)/setWindow(50) ms di NimBLE)
    Bluefruit.Scanner.filterRssi(-127);         // terima semua RSSI
    Bluefruit.Scanner.start(0);                 // 0 = scan non-stop
}

// ============================================================
// LOOP
// ============================================================
void loop() {
    handleSerial();

    unsigned long now = millis();

    // Cetak tabel periodik
    if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
        lastPrintTime = now;
        printAllTires();
    }

    // Proses hasil scan TPMS selesai
    if (scanTpmsDone) {
        scanTpmsDone = false;

        if (scannedEmpty()) {
            Serial.println("[INFO] Tidak ada sensor TPMS baru ditemukan.");
        } else {
            Serial.printf("\n[HASIL] %d sensor TPMS ditemukan:\n", scannedCount);
            for (int i = 0; i < scannedCount; i++) {
                Serial.printf("  %s | RSSI: %d dBm\n",
                              scannedTpms[i].mac,
                              scannedTpms[i].rssi);
            }
        }

        // Kembali ke scan continuous (monitoring)
        Bluefruit.Scanner.start(0);
    }
}

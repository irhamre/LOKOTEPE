/*
 * ============================================================
 * TPMS GATEWAY v4 – ESP32C3 Mini + SX1276
 * Receiver | Multi-Node | Pair/Unpair per Node ID
 * Hardware  : ESP32C3 Mini
 * Radio     : SX1276 (LoRa) – Custom wiring
 *   SCK=4  MISO=5  MOSI=6  CS=7  RST=2  DIO0=3
 * Library   : arduino-LoRa, ArduinoJson, PubSubClient
 *
 * FITUR:
 *  - Scan node LoRa yang ada di udara
 *  - Pair / Unpair node berdasarkan ID (TRL001, TRL002, dst)
 *  - Hanya data dari node yang di-pair yang diproses & dikirim
 *  - Forward data ke MQTT broker (bevywise)
 *  - 1 paket JSON berisi seluruh 16 ban per node (bukan per ban)
 *  - Sub-topic MQTT per node: tpms/TRL001, tpms/TRL002, dst.
 *  - Data ban di-hold dari cache; jika >3 menit tidak update → NO_PACKET
 *  - Serial monitor lengkap: help, scan, pair, unpair, list, status
 * ============================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <Preferences.h>

// ============================================================
// PIN LoRa SX1276 (ESP32C3 Mini)z
// ============================================================
#define LORA_SCK   4
#define LORA_MISO  5
#define LORA_MOSI  6
#define LORA_CS    7
#define LORA_RST   2
#define LORA_DIO0  3
#define LORA_FREQ  922E6   // Ganti 868E6 jika region 868 MHz

// ============================================================
// WIFI & MQTT
// ============================================================
const char* WIFI_SSID  = "testwifi";
const char* WIFI_PASS  = "hambacuan";

const char* MQTT_BROKER    = "public-mqtt-broker.bevywise.com";
const int   MQTT_PORT      = 1883;
const char* MQTT_CLIENT_ID = "TPMS_GW_001";

// Topic utama: tpms/<nodeId>
// Contoh: tpms/TRL001  |  tpms/TRL002
// Status gateway: tpms/gateway/status
const char* MQTT_TOPIC_ROOT   = "tpms";
const char* MQTT_TOPIC_STATUS = "tpms/gateway/status";

WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

// ============================================================
// PROTOCOL (harus sama dengan node)
// ============================================================
#define PKT_MAGIC  0xA55A
#define PKT_VER    0x04

struct __attribute__((packed)) PktHeader {
    uint16_t magic;
    uint8_t  ver;
    char     nodeId[7];
    uint8_t  tireCount;
};

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

struct __attribute__((packed)) LoRaPacket {
    PktHeader hdr;
    TpmsData  data;
};

// ============================================================
// NODE MANAGEMENT
// ============================================================
#define MAX_NODES      10
#define MAX_TIRES      16

// Jika tidak ada update selama 3 menit → status NO_PACKET
#define DATA_STALE_MS  180000UL   // 3 menit

struct NodeEntry {
    char     id[8];          // "TRL001"
    bool     paired;
    uint8_t  tireCount;
    unsigned long firstSeen;
    unsigned long lastSeen;
    float    lastRssi;
    uint32_t rxCount;
    // Cache data per tire
    TpmsData      tireCache[MAX_TIRES];
    unsigned long tireLastUpdate[MAX_TIRES];
    bool          tireHasData[MAX_TIRES];
};

NodeEntry nodes[MAX_NODES];
int       nodeCount = 0;

Preferences preferences;

// ============================================================
// SCAN MODE – kumpulkan node yang terdeteksi
// ============================================================
struct ScannedNode { char id[8]; float rssi; unsigned long seenAt; };
ScannedNode scannedNodes[MAX_NODES];
int         scannedCount = 0;
bool        scanMode     = false;
unsigned long scanStart  = 0;
#define SCAN_DURATION_MS 30000

// ============================================================
// HELPER – cari/buat node entry
// ============================================================
int findNode(const char* id) {
    for (int i = 0; i < nodeCount; i++)
        if (strcmp(nodes[i].id, id) == 0) return i;
    return -1;
}

int findOrCreateNode(const char* id, uint8_t tireCount) {
    int idx = findNode(id);
    if (idx != -1) return idx;
    if (nodeCount >= MAX_NODES) return -1;
    idx = nodeCount++;
    memset(&nodes[idx], 0, sizeof(NodeEntry));
    strncpy(nodes[idx].id, id, 7);
    nodes[idx].paired    = false;
    nodes[idx].tireCount = tireCount;
    nodes[idx].firstSeen = millis();
    return idx;
}

// ============================================================
// FLASH – simpan/load node yang di-pair
// ============================================================
void savePairedNodes() {
    preferences.begin("gwpair", false);
    int count = 0;
    for (int i = 0; i < nodeCount; i++)
        if (nodes[i].paired) count++;
    preferences.putInt("count", count);
    int k = 0;
    for (int i = 0; i < nodeCount; i++) {
        if (!nodes[i].paired) continue;
        preferences.putString(("id" + String(k)).c_str(), nodes[i].id);
        k++;
    }
    preferences.end();
}

void loadPairedNodes() {
    preferences.begin("gwpair", true);
    int count = preferences.getInt("count", 0);
    for (int i = 0; i < count; i++) {
        String id = preferences.getString(("id" + String(i)).c_str(), "");
        if (id.length() > 0) {
            int idx = findOrCreateNode(id.c_str(), MAX_TIRES);
            if (idx != -1) nodes[idx].paired = true;
        }
    }
    preferences.end();
    if (count > 0)
        Serial.printf("[FLASH] %d node di-load dari memori.\n", count);
}

// ============================================================
// RADIO DIAGNOSTIC
// ============================================================
struct GwDiag {
    uint32_t rxTotal       = 0;
    uint32_t rxOk          = 0;
    uint32_t rxCsErr       = 0;
    uint32_t rxMagicErr    = 0;
    uint32_t rxIgnored     = 0;
    float    rssiLast      = 0;
    float    rssiSum       = 0;
    float    snrLast       = 0;
    uint32_t mqttSent      = 0;
    uint32_t mqttFail      = 0;
};
GwDiag        gwDiag;
unsigned long diagStartTime = 0;

// ============================================================
// WIFI
// ============================================================
void setupWifi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("[WIFI] Connecting");
    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 20000) {
        delay(500); Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("[WIFI] OK – IP: "); Serial.println(WiFi.localIP());
    } else {
        Serial.println("[WIFI] Gagal. MQTT tidak tersedia.");
    }
}

// ============================================================
// MQTT
// ============================================================
void mqttReconnect() {
    if (WiFi.status() != WL_CONNECTED) return;
    if (mqttClient.connected()) return;

    Serial.print("[MQTT] Connecting...");
    if (mqttClient.connect(MQTT_CLIENT_ID)) {
        Serial.println(" OK");
        mqttClient.publish(MQTT_TOPIC_STATUS, "{\"status\":\"online\",\"hw\":\"ESP32C3_SX1276\"}");
    } else {
        Serial.printf(" Gagal rc=%d\n", mqttClient.state());
    }
}

// ============================================================
// PUBLISH – 1 JSON berisi seluruh 16 ban untuk satu node
// Topic: tpms/<nodeId>
// ============================================================
void publishNodeBundle(int nodeIdx) {
    if (!mqttClient.connected()) return;
    if (!nodes[nodeIdx].paired) return;

    // Topic: tpms/TRL001
    char topic[48];
    snprintf(topic, sizeof(topic), "%s/%s", MQTT_TOPIC_ROOT, nodes[nodeIdx].id);

    unsigned long now = millis();

    // Gunakan buffer JSON yang cukup besar (16 ban × ~120 byte)
    // ArduinoJson v7: JsonDocument otomatis mengelola ukuran
    JsonDocument doc;

    doc["node"]       = nodes[nodeIdx].id;
    doc["paired"]     = nodes[nodeIdx].paired;
    doc["rx_count"]   = nodes[nodeIdx].rxCount;
    doc["rssi_rf"]    = nodes[nodeIdx].lastRssi;
    doc["node_age_s"] = (now - nodes[nodeIdx].lastSeen) / 1000;
    doc["uptime_s"]   = now / 1000;
    doc["ts"]         = now;

    JsonArray tires = doc["tires"].to<JsonArray>();

    uint8_t numTires = nodes[nodeIdx].tireCount;
    if (numTires == 0 || numTires > MAX_TIRES) numTires = MAX_TIRES;

    for (int t = 0; t < numTires; t++) {
        JsonObject tire = tires.add<JsonObject>();
        tire["id"] = t;

        bool noData  = !nodes[nodeIdx].tireHasData[t];
        bool expired = noData ? false :
                       (now - nodes[nodeIdx].tireLastUpdate[t] > DATA_STALE_MS);

        if (noData || expired) {
            // Belum pernah ada data, atau sudah lebih dari 3 menit tidak update
            tire["status"] = "NO_PACKET";
        } else {
            // Gunakan data cache (mungkin bukan yang terbaru, tapi masih valid)
            TpmsData& d = nodes[nodeIdx].tireCache[t];
            unsigned long age_s = (now - nodes[nodeIdx].tireLastUpdate[t]) / 1000;

            tire["status"]      = "OK";
            tire["kpa"]         = d.kpa_x10 / 10.0f;
            tire["temp_c"]      = d.temp;
            tire["volt_v"]      = d.volt_mV / 1000.0f;
            tire["rssi_ble"]    = d.bleRssi;
            tire["status_code"] = d.statusCode;
            tire["age_s"]       = age_s;   // berapa detik sejak update terakhir
        }
    }

    // Serialisasi ke buffer
    // Ukuran aman: 16 ban × 120 byte + overhead ~200 byte ≈ 2120 byte
    // PubSubClient default buffer 256 byte – harus diperbesar di setup!
    char payload[2200];
    size_t len = serializeJson(doc, payload, sizeof(payload));

    if (mqttClient.publish(topic, (const uint8_t*)payload, len, true)) {
        gwDiag.mqttSent++;
        Serial.printf("[MQTT] Published %s (%d byte)\n", topic, (int)len);
    } else {
        gwDiag.mqttFail++;
        Serial.printf("[MQTT] FAIL publish %s (payload %d byte – cek buffer!)\n",
                      topic, (int)len);
    }
}

// ============================================================
// INISIALISASI RADIO
// ============================================================
bool radioReady = false;

void initRadio() {
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

    if (!LoRa.begin(LORA_FREQ)) {
        Serial.println("[LoRa] Init GAGAL! Periksa wiring SX1276.");
        return;
    }
    // Parameter HARUS sama persis dengan node
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.setSyncWord(0x12);
    LoRa.enableCrc();

    radioReady = true;
    Serial.printf("[LoRa] SX1276 siap RX. Freq:%ld SF7 BW125 Pkt:%dB\n",
                  (long)LORA_FREQ, (int)sizeof(LoRaPacket));
}

// ============================================================
// PROSES PAKET MASUK
// ============================================================
void processIncoming() {
    if (!radioReady) return;
    int packetSize = LoRa.parsePacket();
    if (packetSize == 0) return;

    gwDiag.rxTotal++;

    if (packetSize != sizeof(LoRaPacket)) {
        while (LoRa.available()) LoRa.read();
        Serial.printf("[WARN] Ukuran paket tidak sesuai: %d byte (exp:%d)\n",
                      packetSize, (int)sizeof(LoRaPacket));
        return;
    }

    LoRaPacket pkt;
    uint8_t* p = (uint8_t*)&pkt;
    for (int i = 0; i < (int)sizeof(LoRaPacket); i++)
        p[i] = (uint8_t)LoRa.read();

    float rssi = LoRa.packetRssi();
    float snr  = LoRa.packetSnr();

    gwDiag.rssiLast = rssi;
    gwDiag.rssiSum += rssi;
    gwDiag.snrLast  = snr;

    // Validasi magic number
    if (pkt.hdr.magic != PKT_MAGIC) {
        gwDiag.rxMagicErr++;
        Serial.printf("[ERR] Magic mismatch: 0x%04X\n", pkt.hdr.magic);
        return;
    }

    pkt.hdr.nodeId[6] = '\0';

    // Verifikasi checksum
    uint8_t* dp = (uint8_t*)&pkt.data;
    uint8_t  cs = 0;
    for (int i = 0; i < (int)sizeof(TpmsData) - 1; i++) cs ^= dp[i];
    if (cs != pkt.data.chksum) {
        gwDiag.rxCsErr++;
        Serial.printf("[ERR] Checksum error dari %s\n", pkt.hdr.nodeId);
        return;
    }

    gwDiag.rxOk++;

    // SCAN MODE – catat node yang terdeteksi
    if (scanMode) {
        bool found = false;
        for (int i = 0; i < scannedCount; i++) {
            if (strcmp(scannedNodes[i].id, pkt.hdr.nodeId) == 0) {
                scannedNodes[i].rssi   = rssi;
                scannedNodes[i].seenAt = millis();
                found = true; break;
            }
        }
        if (!found && scannedCount < MAX_NODES) {
            strncpy(scannedNodes[scannedCount].id, pkt.hdr.nodeId, 7);
            scannedNodes[scannedCount].rssi   = rssi;
            scannedNodes[scannedCount].seenAt = millis();
            scannedCount++;
            Serial.printf("[SCAN] Node ditemukan: %s | RSSI:%.1fdBm | SNR:%.1fdB\n",
                          pkt.hdr.nodeId, rssi, snr);
        }
    }

    // Cari/buat entry node
    int nIdx = findOrCreateNode(pkt.hdr.nodeId, pkt.hdr.tireCount);
    if (nIdx == -1) {
        Serial.println("[WARN] Tabel node penuh!");
        return;
    }

    nodes[nIdx].lastSeen = millis();
    nodes[nIdx].lastRssi = rssi;
    nodes[nIdx].rxCount++;

    // Filter – hanya proses node yang sudah di-pair
    if (!nodes[nIdx].paired) {
        gwDiag.rxIgnored++;
        if (nodes[nIdx].rxCount % 20 == 1)
            Serial.printf("[IGN] Node %s belum di-pair (ketik: pair %s)\n",
                          pkt.hdr.nodeId, pkt.hdr.nodeId);
        return;
    }

    // Simpan ke cache (data sebelumnya tetap tersimpan sampai expired)
    uint8_t tireId = pkt.data.tireId;
    if (tireId < MAX_TIRES) {
        nodes[nIdx].tireCache[tireId]      = pkt.data;
        nodes[nIdx].tireLastUpdate[tireId] = millis();
        nodes[nIdx].tireHasData[tireId]    = true;
    }

    Serial.printf("[RX] %s | T%-2d | %.1fkPa | %dC | %.2fV | RF:%.1fdBm | SNR:%.1fdB | BLE:%ddBm\n",
                  pkt.hdr.nodeId, pkt.data.tireId,
                  pkt.data.kpa_x10 / 10.0f, pkt.data.temp,
                  pkt.data.volt_mV / 1000.0f,
                  rssi, snr, pkt.data.bleRssi);

    // Setiap kali ada data masuk dari node yang di-pair,
    // langsung publish bundle 16 ban node tersebut
    publishNodeBundle(nIdx);
}

// ============================================================
// PRINT STATUS SERIAL
// ============================================================
void printList() {
    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║         DAFTAR NODE TPMS GATEWAY         ║");
    Serial.println("╠══════════════════════════════════════════╣");
    if (nodeCount == 0) {
        Serial.println("║ (kosong – lakukan scan terlebih dahulu)  ║");
    } else {
        for (int i = 0; i < nodeCount; i++) {
            unsigned long age = (millis() - nodes[i].lastSeen) / 1000;
            Serial.printf("║  [%d] %-8s | %-8s | RSSI:%.1fdBm | %lus ago\n",
                          i, nodes[i].id,
                          nodes[i].paired ? "PAIRED" : "unpaired",
                          nodes[i].lastRssi, age);
        }
    }
    Serial.println("╚══════════════════════════════════════════╝");
}

void printStatus() {
    Serial.println();
    unsigned long now = millis();

    for (int n = 0; n < nodeCount; n++) {
        if (!nodes[n].paired) continue;

        Serial.printf("═══ Node: %-8s | RX:%lu | RSSI:%.1fdBm ═══\n",
                      nodes[n].id, nodes[n].rxCount, nodes[n].lastRssi);
        Serial.printf("  %-6s | %-10s | %5s | %3s | %5s | %4s | %5s\n",
                      "Tire", "Status", "KPa", "°C", "Volt", "BLE", "Age_s");
        Serial.println("  ─────────────────────────────────────────────────────");

        for (int t = 0; t < nodes[n].tireCount && t < MAX_TIRES; t++) {
            bool noData  = !nodes[n].tireHasData[t];
            bool expired = noData ? false :
                           (now - nodes[n].tireLastUpdate[t] > DATA_STALE_MS);

            if (noData || expired) {
                Serial.printf("  T%-5d | %-10s | %5s | %3s | %5s | %4s | %5s\n",
                              t, noData ? "NO DATA" : "NO_PACKET",
                              "-", "-", "-", "-", "-");
            } else {
                unsigned long age = (now - nodes[n].tireLastUpdate[t]) / 1000;
                Serial.printf("  T%-5d | %-10s | %5.1f | %3d | %5.2f | %4d | %5lu\n",
                              t, "OK",
                              nodes[n].tireCache[t].kpa_x10 / 10.0f,
                              nodes[n].tireCache[t].temp,
                              nodes[n].tireCache[t].volt_mV / 1000.0f,
                              nodes[n].tireCache[t].bleRssi,
                              age);
            }
        }
        Serial.println();
    }

    Serial.printf("[MQTT] Sent:%lu Fail:%lu Connected:%s\n",
                  gwDiag.mqttSent, gwDiag.mqttFail,
                  mqttClient.connected() ? "YES" : "NO");
    Serial.printf("[RADIO] RX:%lu OK:%lu CsErr:%lu Ignored:%lu\n",
                  gwDiag.rxTotal, gwDiag.rxOk, gwDiag.rxCsErr, gwDiag.rxIgnored);
}

void printDiag() {
    float rssiAvg = gwDiag.rxOk > 0 ? gwDiag.rssiSum / gwDiag.rxOk : 0;
    unsigned long uptime = (millis() - diagStartTime) / 1000;

    Serial.println("\n═══════════════════════════════════════════════");
    Serial.println("═       GATEWAY DIAGNOSTIC                    ═");
    Serial.println("═══════════════════════════════════════════════");
    Serial.printf( "═ Uptime      : %lu detik\n", uptime);
    Serial.printf( "═ RX Total    : %lu\n", gwDiag.rxTotal);
    Serial.printf( "═ RX OK       : %lu\n", gwDiag.rxOk);
    Serial.printf( "═ CS Error    : %lu\n", gwDiag.rxCsErr);
    Serial.printf( "═ Magic Error : %lu\n", gwDiag.rxMagicErr);
    Serial.printf( "═ Ignored     : %lu (unpaired nodes)\n", gwDiag.rxIgnored);
    Serial.printf( "═ RSSI Last   : %.1f dBm\n", gwDiag.rssiLast);
    Serial.printf( "═ RSSI Avg    : %.1f dBm\n", rssiAvg);
    Serial.printf( "═ SNR Last    : %.1f dB\n", gwDiag.snrLast);
    Serial.println("═══════════════════════════════════════════════");
    Serial.printf( "═ MQTT Sent   : %lu\n", gwDiag.mqttSent);
    Serial.printf( "═ MQTT Fail   : %lu\n", gwDiag.mqttFail);
    Serial.printf( "═ MQTT Status : %s\n", mqttClient.connected() ? "CONNECTED" : "DISCONNECTED");
    Serial.printf( "═ WiFi Status : %s\n",
                   WiFi.status() == WL_CONNECTED
                     ? WiFi.localIP().toString().c_str()
                     : "DISCONNECTED");
    Serial.println("═══════════════════════════════════════════════\n");

    for (int i = 0; i < nodeCount; i++)
        Serial.printf("  Node %-8s | %s | RX:%lu | RSSI:%.1fdBm\n",
                      nodes[i].id,
                      nodes[i].paired ? "PAIRED  " : "unpaired",
                      nodes[i].rxCount, nodes[i].lastRssi);
    Serial.println();
}

void sendAllJson() {
    // Output JSON semua node ke serial (untuk debug)
    JsonDocument doc;
    doc["type"]     = "gateway_all";
    doc["uptime_s"] = millis() / 1000;

    JsonArray nodeArr = doc["nodes"].to<JsonArray>();
    unsigned long now = millis();

    for (int n = 0; n < nodeCount; n++) {
        JsonObject nObj = nodeArr.add<JsonObject>();
        nObj["id"]     = nodes[n].id;
        nObj["paired"] = nodes[n].paired;
        nObj["rx"]     = nodes[n].rxCount;
        JsonArray tires = nObj["tires"].to<JsonArray>();
        for (int t = 0; t < nodes[n].tireCount && t < MAX_TIRES; t++) {
            JsonObject tire = tires.add<JsonObject>();
            tire["id"] = t;
            bool noData  = !nodes[n].tireHasData[t];
            bool expired = noData ? false :
                           (now - nodes[n].tireLastUpdate[t] > DATA_STALE_MS);
            if (noData || expired) {
                tire["status"] = noData ? "NO_DATA" : "NO_PACKET";
            } else {
                tire["status"] = "OK";
                tire["kpa"]    = nodes[n].tireCache[t].kpa_x10 / 10.0f;
                tire["temp"]   = nodes[n].tireCache[t].temp;
                tire["volt"]   = nodes[n].tireCache[t].volt_mV / 1000.0f;
                tire["ble"]    = nodes[n].tireCache[t].bleRssi;
                tire["age_s"]  = (now - nodes[n].tireLastUpdate[t]) / 1000;
            }
        }
    }
    serializeJson(doc, Serial);
    Serial.println();
}

// ============================================================
// SERIAL COMMAND HANDLER
// ============================================================
void printHelp() {
    Serial.println("\n═══════════════════════════════════════════════");
    Serial.println("═    TPMS GATEWAY v4 – SERIAL COMMANDS       ═");
    Serial.println("═══════════════════════════════════════════════");
    Serial.println("═  help                 Tampilkan perintah   ═");
    Serial.println("═  scan                 Scan node aktif      ═");
    Serial.println("═  pair <ID>            Pair node (e.g. pair TRL001) ═");
    Serial.println("═  unpair <ID>          Unpair node          ═");
    Serial.println("═  unpair_all           Unpair semua node    ═");
    Serial.println("═  list                 Daftar semua node    ═");
    Serial.println("═  status               Data tire per node   ═");
    Serial.println("═  json                 Output JSON ke serial═");
    Serial.println("═  diag                 Diagnostik radio     ═");
    Serial.println("═  diag_reset           Reset diagnostik     ═");
    Serial.println("═  mqtt_status          Cek koneksi MQTT     ═");
    Serial.println("═══════════════════════════════════════════════");
    Serial.println("═  MQTT Topic: tpms/<nodeId>  (1 JSON 16 ban)═");
    Serial.printf( "═  Data stale: >%lu menit → NO_PACKET        ═\n",
                   DATA_STALE_MS / 60000UL);
    Serial.println("═══════════════════════════════════════════════\n");
}

void handleSerial() {
    if (!Serial.available()) return;
    String input = Serial.readStringUntil('\n');
    input.trim();

    String cmd = input;
    cmd.toLowerCase();

    if (cmd == "help") { printHelp(); return; }

    if (cmd == "scan") {
        scannedCount = 0;
        scanMode     = true;
        scanStart    = millis();
        Serial.printf("[INFO] Scanning node LoRa selama %d detik...\n",
                      SCAN_DURATION_MS / 1000);
        return;
    }

    if (input.startsWith("pair ") || input.startsWith("PAIR ")) {
        String id = input.substring(5);
        id.trim();
        id.toUpperCase();
        if (id.length() == 0 || id.length() > 7) {
            Serial.println("[ERROR] Format: pair TRL001"); return;
        }
        int idx = findNode(id.c_str());
        if (idx == -1) {
            idx = findOrCreateNode(id.c_str(), MAX_TIRES);
            if (idx == -1) { Serial.println("[ERROR] Tabel node penuh."); return; }
            Serial.printf("[WARN] Node %s belum pernah terdeteksi, di-pair secara manual.\n", id.c_str());
        }
        if (nodes[idx].paired) {
            Serial.printf("[INFO] Node %s sudah di-pair.\n", id.c_str()); return;
        }
        nodes[idx].paired = true;
        savePairedNodes();
        Serial.printf("[OK] Node %s berhasil di-pair.\n", id.c_str());
        Serial.printf("     MQTT Topic: %s/%s\n", MQTT_TOPIC_ROOT, id.c_str());
        return;
    }

    if (input.startsWith("unpair ") || input.startsWith("UNPAIR ")) {
        String id = input.substring(7);
        id.trim();
        id.toUpperCase();
        int idx = findNode(id.c_str());
        if (idx == -1 || !nodes[idx].paired) {
            Serial.printf("[ERROR] Node %s tidak ditemukan atau belum di-pair.\n", id.c_str()); return;
        }
        nodes[idx].paired = false;
        savePairedNodes();
        Serial.printf("[OK] Node %s di-unpair. Data tidak akan dikirim ke MQTT.\n", id.c_str());
        return;
    }

    if (cmd == "unpair_all") {
        for (int i = 0; i < nodeCount; i++) nodes[i].paired = false;
        savePairedNodes();
        Serial.println("[OK] Semua node di-unpair.");
        return;
    }

    if (cmd == "list")   { printList();   return; }
    if (cmd == "status") { printStatus(); return; }
    if (cmd == "json")   { sendAllJson(); return; }
    if (cmd == "diag")   { printDiag();   return; }

    if (cmd == "diag_reset") {
        gwDiag = GwDiag{};
        diagStartTime = millis();
        Serial.println("[OK] Diagnostic direset.");
        return;
    }

    if (cmd == "mqtt_status") {
        Serial.printf("[MQTT] Status: %s | Broker: %s:%d\n",
                      mqttClient.connected() ? "CONNECTED" : "DISCONNECTED",
                      MQTT_BROKER, MQTT_PORT);
        Serial.printf("[WiFi] Status: %s\n",
                      WiFi.status() == WL_CONNECTED
                        ? WiFi.localIP().toString().c_str()
                        : "DISCONNECTED");
        Serial.println("[Topics aktif]:");
        for (int i = 0; i < nodeCount; i++)
            if (nodes[i].paired)
                Serial.printf("  %s/%s\n", MQTT_TOPIC_ROOT, nodes[i].id);
        return;
    }

    Serial.println("Tidak dikenal. Ketik 'help' untuk daftar perintah.");
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== TPMS GATEWAY v4 – ESP32C3 + SX1276 ===");

    initRadio();
    setupWifi();

    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setKeepAlive(60);

    // PENTING: buffer diperbesar ke 2560 byte untuk JSON 16 ban
    // Default PubSubClient hanya 256 byte – tidak cukup untuk 16 ban!
    mqttClient.setBufferSize(2560);

    mqttReconnect();
    loadPairedNodes();

    diagStartTime = millis();

    printHelp();
    Serial.println("[GATEWAY] Siap. Menunggu paket dari node...");
}

// ============================================================
// LOOP
// ============================================================
unsigned long lastReportTime      = 0;
unsigned long lastPeriodicPublish = 0;
unsigned long lastMqttPing        = 0;

// Laporan serial setiap 30 detik
#define REPORT_INTERVAL_MS   30000
// Publish periodik bundle semua node setiap 60 detik
// (bahkan jika tidak ada paket baru masuk – agar NO_PACKET terdeteksi)
#define PERIODIC_PUBLISH_MS  60000
#define MQTT_PING_MS         10000

void loop() {
    handleSerial();
    processIncoming();

    // MQTT loop & reconnect
    if (WiFi.status() == WL_CONNECTED) {
        mqttClient.loop();
        unsigned long now = millis();
        if (now - lastMqttPing > MQTT_PING_MS && !mqttClient.connected()) {
            lastMqttPing = now;
            mqttReconnect();
        }
    }

    // Scan mode timeout
    if (scanMode && millis() - scanStart > SCAN_DURATION_MS) {
        scanMode = false;
        Serial.printf("\n[SCAN SELESAI] %d node ditemukan:\n", scannedCount);
        if (scannedCount == 0) {
            Serial.println("  (tidak ada node terdeteksi)");
        } else {
            for (int i = 0; i < scannedCount; i++)
                Serial.printf("  %s | RSSI:%.1fdBm\n", scannedNodes[i].id, scannedNodes[i].rssi);
            Serial.println("[TIP] Gunakan: pair <ID>  contoh: pair TRL001");
        }
        Serial.println();
    }

    unsigned long now = millis();

    // Laporan periodik ke serial
    if (now - lastReportTime > REPORT_INTERVAL_MS) {
        lastReportTime = now;
        printStatus();
    }

    // Publish periodik – kirim bundle semua node yang di-pair
    // Ini memastikan NO_PACKET terdeteksi walau tidak ada paket LoRa baru
    if (now - lastPeriodicPublish > PERIODIC_PUBLISH_MS && mqttClient.connected()) {
        lastPeriodicPublish = now;
        for (int i = 0; i < nodeCount; i++) {
            if (nodes[i].paired) {
                publishNodeBundle(i);
            }
        }
    }
}

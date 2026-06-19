/*
 * ============================================================
 * TPMS GATEWAY v6 – ESP32C3 Mini + SX1276
 * Receiver | Multi-Node | Pair/Unpair per Node ID
 * + LITTLEFS STORE & FORWARD
 * + WIFI & MQTT BROKER CONFIGURABLE (WiFiManager, tidak hardcode lagi)
 * + OTA FIRMWARE UPDATE (ArduinoOTA, tidak perlu kabel USB)
 *
 * Hardware  : ESP32C3 Mini
 * Radio     : SX1276 (LoRa) – Custom wiring
 *   SCK=4  MISO=5  MOSI=6  CS=7  RST=2  DIO0=3
 *
 * LIBRARY YANG DIBUTUHKAN (install lewat Library Manager):
 *   - arduino-LoRa     (sandeepmistry)
 *   - ArduinoJson      (v7)
 *   - PubSubClient
 *   - WiFiManager      (tzapu)  <-- WAJIB DIINSTALL, ini yang baru
 *   - ArduinoOTA & Preferences & LittleFS sudah bundled di ESP32 core
 *
 * ============================================================
 * APA YANG BERUBAH DI v6
 * ============================================================
 *  1) WIFI TIDAK HARDCODE LAGI
 *     - Saat pertama kali nyala (atau setelah "wifi_reset"), ESP32
 *       akan membuat Access Point bernama "TPMS-Gateway-Setup".
 *     - Konek HP/laptop ke AP itu (password: lihat AP_PASSWORD di
 *       bawah), browser akan otomatis terbuka halaman konfigurasi
 *       (captive portal). Pilih WiFi rumah/router, isi password.
 *     - Kredensial WiFi disimpan otomatis oleh ESP32, dipakai lagi
 *       di boot-boot berikutnya tanpa perlu setting ulang.
 *
 *  2) MQTT BROKER JUGA BISA DIATUR DARI PORTAL YANG SAMA
 *     - Di halaman konfigurasi WiFi tadi, ada 2 field tambahan:
 *       "MQTT Broker Host/IP" dan "MQTT Broker Port".
 *     - Disimpan di NVS (Preferences), tidak perlu reflash firmware
 *       kalau mau ganti broker.
 *     - Bisa juga diganti cepat lewat serial command:
 *           set_broker <host> [port]
 *
 *  3) OTA FIRMWARE UPDATE (ArduinoOTA)
 *     - Setelah gateway konek WiFi, dia akan muncul di Arduino IDE
 *       sebagai "Network Port" (Tools > Port > tpms-gateway at x.x.x.x).
 *     - Upload sketch seperti biasa lewat port itu (tanpa kabel USB),
 *       nanti akan diminta password OTA (lihat OTA_PASSWORD di bawah).
 *
 *  4) SERIAL COMMAND BARU
 *     - wifi_config         : buka ulang portal konfigurasi WiFi/MQTT
 *     - wifi_reset          : hapus kredensial WiFi tersimpan
 *     - set_broker <h> [p]  : ganti broker MQTT cepat lewat serial
 *     - show_config         : tampilkan konfigurasi aktif saat ini
 *
 *  CATATAN: selama portal konfigurasi terbuka (AP mode), gateway
 *  TIDAK memproses paket LoRa/serial (sifatnya blocking) — ini
 *  perilaku normal WiFiManager, karena ganti network memang
 *  sebaiknya jadi aksi sengaja, bukan terjadi di tengah operasi.
 * ============================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <WiFiManager.h>          // by tzapu - install lewat Library Manager
#include <ArduinoOTA.h>           // bundled di ESP32 core
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include "LittleFS.h"

// ============================================================
// PIN LoRa SX1276 (ESP32C3 Mini)
// ============================================================
#define LORA_SCK   4
#define LORA_MISO  5
#define LORA_MOSI  6
#define LORA_CS    7
#define LORA_RST   2
#define LORA_DIO0  3
#define LORA_FREQ  922E6   // Ganti 868E6 jika region 868 MHz

// ============================================================
// KONFIGURASI AP SETUP & OTA
// (Ganti password ini untuk keamanan sebelum dipakai di lapangan)
// ============================================================
#define AP_NAME       "TPMS-Gateway-Setup"
#define AP_PASSWORD   "tpms12345"     // min 8 karakter, password buat masuk AP config
#define OTA_HOSTNAME  "tpms-gateway"
#define OTA_PASSWORD  "tpms12345"     // password upload firmware OTA

#define MQTT_CLIENT_ID "TPMS_GW_001"

// Topic utama: tpms/<nodeId>
// Contoh: tpms/TRL001  |  tpms/TRL002
// Status gateway: tpms/gateway/status
const char* MQTT_TOPIC_ROOT   = "tpms";
const char* MQTT_TOPIC_STATUS = "tpms/gateway/status";

WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

Preferences preferences;   // dipakai bergantian: namespace "gwpair" & "gwcfg"

// ============================================================
// KONFIGURASI MQTT BROKER (tidak hardcode lagi)
// Default di bawah hanya dipakai SEBELUM user mengisi portal /
// serial command, setelah itu nilai aktif diambil dari NVS.
// ============================================================
char mqttHost[64]    = "public-mqtt-broker.bevywise.com";
char mqttPortStr[6]  = "1883";

WiFiManager wm;
WiFiManagerParameter custom_mqtt_host("host", "MQTT Broker Host/IP", mqttHost, 64);
WiFiManagerParameter custom_mqtt_port("port", "MQTT Broker Port (default 1883)", mqttPortStr, 6);
bool shouldSaveConfig   = false;
bool mqttServerConfigured = false;
bool otaStarted          = false;

void saveConfigCallback() {
    shouldSaveConfig = true;
}

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
// FLASH – simpan/load node yang di-pair (namespace "gwpair")
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
// FLASH – simpan/load konfigurasi MQTT broker (namespace "gwcfg")
// ============================================================
void loadBrokerConfig() {
    preferences.begin("gwcfg", true);
    String h = preferences.getString("mqtt_host", mqttHost);
    String p = preferences.getString("mqtt_port", mqttPortStr);
    preferences.end();
    h.toCharArray(mqttHost, sizeof(mqttHost));
    p.toCharArray(mqttPortStr, sizeof(mqttPortStr));
}

void saveBrokerConfig() {
    preferences.begin("gwcfg", false);
    preferences.putString("mqtt_host", mqttHost);
    preferences.putString("mqtt_port", mqttPortStr);
    preferences.end();
    Serial.printf("[CFG] Disimpan -> broker:%s port:%s\n", mqttHost, mqttPortStr);
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
    // --- store & forward ---
    uint32_t fsSaved       = 0;   // data disimpan ke LittleFS karena gagal kirim
    uint32_t fsForwarded   = 0;   // data berhasil di-forward dari LittleFS
    uint32_t fsSaveFail    = 0;   // gagal buka file FS untuk menyimpan
};
GwDiag        gwDiag;
unsigned long diagStartTime = 0;

// ============================================================
// WIFI MANAGER – setup portal + parameter custom MQTT
// ============================================================

// Tambahkan parameter custom ke WiFiManager (sekali saja, walau
// fungsi ini dipanggil berulang lewat "wifi_config")
void initWifiManagerParams() {
    loadBrokerConfig();
    custom_mqtt_host.setValue(mqttHost, 64);
    custom_mqtt_port.setValue(mqttPortStr, 6);

    static bool added = false;
    if (!added) {
        wm.addParameter(&custom_mqtt_host);
        wm.addParameter(&custom_mqtt_port);
        wm.setSaveConfigCallback(saveConfigCallback);
        added = true;
    }
}

// Ambil nilai field setelah portal ditutup, simpan ke NVS bila berubah
void readBackWifiManagerParams() {
    strncpy(mqttHost, custom_mqtt_host.getValue(), sizeof(mqttHost) - 1);
    mqttHost[sizeof(mqttHost) - 1] = '\0';
    strncpy(mqttPortStr, custom_mqtt_port.getValue(), sizeof(mqttPortStr) - 1);
    mqttPortStr[sizeof(mqttPortStr) - 1] = '\0';

    if (shouldSaveConfig) {
        saveBrokerConfig();
        shouldSaveConfig = false;
    }
}

// Dipanggil di setup(): coba pakai kredensial tersimpan, kalau gagal
// (atau belum pernah diset) -> buka captive portal otomatis.
// Timeout 180s supaya gateway tidak macet selamanya kalau tidak ada
// yang melakukan setup (lanjut jalan offline, data ke LittleFS).
bool setupWifiAndConfig() {
    initWifiManagerParams();
    wm.setConfigPortalTimeout(180);
    bool ok = wm.autoConnect(AP_NAME, AP_PASSWORD);
    readBackWifiManagerParams();
    return ok;
}

// Dipanggil manual lewat serial command "wifi_config" untuk
// memaksa membuka portal kapan saja (misal mau ganti WiFi/broker).
void forceConfigPortal() {
    Serial.printf("[WIFI] Membuka config portal. Konek ke AP \"%s\" lalu buka browser.\n", AP_NAME);
    initWifiManagerParams();
    wm.setConfigPortalTimeout(180);
    bool ok = wm.startConfigPortal(AP_NAME, AP_PASSWORD);
    readBackWifiManagerParams();
    Serial.println(ok ? "[WIFI] Konfigurasi baru disimpan." : "[WIFI] Portal timeout/dibatalkan, memakai konfigurasi lama.");
    Serial.println("[SYSTEM] Restart untuk menerapkan...");
    delay(1500);
    ESP.restart();
}

// ============================================================
// OTA – ArduinoOTA (update firmware lewat WiFi, tanpa USB)
// ============================================================
void setupOTA() {
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);

    ArduinoOTA
        .onStart([]() {
            String type = (ArduinoOTA.getCommand() == U_FLASH) ? "firmware" : "filesystem";
            Serial.println("[OTA] Mulai update: " + type);
        })
        .onEnd([]() {
            Serial.println("\n[OTA] Update selesai. Restart...");
        })
        .onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("[OTA] Progress: %u%%\r", (progress * 100) / total);
        })
        .onError([](ota_error_t error) {
            Serial.printf("[OTA] Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR)         Serial.println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR)     Serial.println("End Failed");
        });

    ArduinoOTA.begin();
    otaStarted = true;
    Serial.printf("[OTA] Siap. Hostname:%s.local  Port:3232\n", OTA_HOSTNAME);
}

// ============================================================
// MQTT
// ============================================================
void mqttReconnect() {
    if (WiFi.status() != WL_CONNECTED) return;
    if (mqttClient.connected()) return;

    Serial.printf("[MQTT] Connecting ke %s:%s... ", mqttHost, mqttPortStr);
    if (mqttClient.connect(MQTT_CLIENT_ID)) {
        Serial.println("OK");
        mqttClient.publish(MQTT_TOPIC_STATUS, "{\"status\":\"online\",\"hw\":\"ESP32C3_SX1276\"}");
    } else {
        Serial.printf("Gagal rc=%d\n", mqttClient.state());
    }
}

// ============================================================
// LITTLEFS STORE & FORWARD
// ============================================================
#define QUEUE_FILE   "/queue.txt"
#define TEMP_FILE    "/temp.txt"
#define FS_DELIM     '\t'   // pemisah antara topic & payload dalam 1 baris

bool fsReady = false;

unsigned long lastFsProcess = 0;
#define FS_PROCESS_INTERVAL_MS  5000UL   // coba forward queue tiap 5 detik saat MQTT connect

// Simpan 1 pesan (topic + payload) yang gagal terkirim ke queue.txt
void saveToFS(const char* topic, const char* payload) {
    if (!fsReady) {
        Serial.println("[FS] LittleFS tidak siap, data HILANG (tidak bisa di-queue)!");
        return;
    }

    File f = LittleFS.open(QUEUE_FILE, FILE_APPEND);
    if (!f) {
        gwDiag.fsSaveFail++;
        Serial.println("[FS] Gagal buka queue.txt untuk menyimpan!");
        return;
    }

    f.print(topic);
    f.print(FS_DELIM);
    f.println(payload);
    f.close();

    gwDiag.fsSaved++;
    Serial.printf("[FS] Disimpan ke queue -> topic:%s\n", topic);
}

// Coba forward semua isi queue.txt ke MQTT.
// Baris yang berhasil dihapus, baris yang gagal disimpan ulang.
void processStoredData() {
    if (!fsReady) return;
    if (!mqttClient.connected()) return;
    if (!LittleFS.exists(QUEUE_FILE)) return;

    File readFile = LittleFS.open(QUEUE_FILE, FILE_READ);
    if (!readFile) return;

    if (readFile.size() == 0) {
        readFile.close();
        LittleFS.remove(QUEUE_FILE);
        return;
    }

    File tempFile = LittleFS.open(TEMP_FILE, FILE_WRITE);
    if (!tempFile) {
        readFile.close();
        Serial.println("[FS] Gagal buat temp.txt, forward dibatalkan kali ini.");
        return;
    }

    Serial.println("[FS] Queue ditemukan, mencoba forward ke MQTT...");

    int total = 0, ok_count = 0, fail_count = 0;

    while (readFile.available()) {
        String line = readFile.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;

        int sep = line.indexOf(FS_DELIM);
        if (sep == -1) continue;   // baris korup/format lama, dibuang

        total++;

        String topic   = line.substring(0, sep);
        String payload = line.substring(sep + 1);

        bool ok = mqttClient.publish(topic.c_str(),
                                      (const uint8_t*)payload.c_str(),
                                      payload.length(), true);

        if (ok) {
            ok_count++;
            gwDiag.fsForwarded++;
            Serial.printf("[FS->MQTT] Forwarded : %s\n", topic.c_str());
            delay(50);   // beri jeda kecil antar publish agar broker tidak overload
        } else {
            fail_count++;
            tempFile.println(line);   // simpan lagi untuk dicoba di kesempatan berikutnya
        }
    }

    readFile.close();
    tempFile.close();

    LittleFS.remove(QUEUE_FILE);

    File check = LittleFS.open(TEMP_FILE, FILE_READ);
    bool hasLeftover = check && check.size() > 0;
    if (check) check.close();

    if (hasLeftover) {
        LittleFS.rename(TEMP_FILE, QUEUE_FILE);
    } else {
        LittleFS.remove(TEMP_FILE);
    }

    Serial.printf("[FS] Selesai forward: total=%d ok=%d gagal=%d\n",
                  total, ok_count, fail_count);
}

// Info ringkas isi queue (untuk perintah serial "queue")
void printQueueInfo() {
    if (!fsReady) {
        Serial.println("[FS] LittleFS tidak aktif.");
        return;
    }
    if (!LittleFS.exists(QUEUE_FILE)) {
        Serial.println("[FS] Queue kosong (file tidak ada).");
        return;
    }

    File f = LittleFS.open(QUEUE_FILE, FILE_READ);
    size_t sz = f.size();
    int lineCount = 0;
    while (f.available()) {
        String l = f.readStringUntil('\n');
        if (l.length() > 0) lineCount++;
    }
    f.close();

    Serial.println("\n----------------");
    Serial.println("QUEUE INFO");
    Serial.printf("Entri tersimpan : %d\n", lineCount);
    Serial.printf("Ukuran file     : %u byte\n", (unsigned)sz);
    Serial.println("----------------\n");
}

// Hapus queue secara manual (perintah serial "clearqueue")
void clearQueueFS() {
    bool any = false;
    if (LittleFS.exists(QUEUE_FILE)) { LittleFS.remove(QUEUE_FILE); any = true; }
    if (LittleFS.exists(TEMP_FILE))  { LittleFS.remove(TEMP_FILE);  any = true; }
    Serial.println(any ? "[FS] Queue dihapus manual." : "[FS] Tidak ada queue untuk dihapus.");
}

// ============================================================
// PUBLISH – 1 JSON berisi seluruh 16 ban untuk satu node
// Topic: tpms/<nodeId>
// Jika gagal kirim (offline / publish gagal) -> disimpan ke LittleFS
// ============================================================
void publishNodeBundle(int nodeIdx) {
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

    bool sent = false;

    if (WiFi.status() == WL_CONNECTED && mqttClient.connected()) {

        sent = mqttClient.publish(topic, (const uint8_t*)payload, len, true);

        if (sent) {
            gwDiag.mqttSent++;
            Serial.printf("[MQTT] Published %s (%d byte)\n", topic, (int)len);
        } else {
            gwDiag.mqttFail++;
            Serial.printf("[MQTT] FAIL publish %s (payload %d byte – cek buffer!) -> simpan ke FS\n",
                          topic, (int)len);
        }

    } else {
        Serial.printf("[MQTT] OFFLINE -> simpan %s ke FS\n", topic);
    }

    if (!sent) {
        saveToFS(topic, payload);
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
    // (kalau gagal/offline, otomatis disimpan ke LittleFS di dalam fungsi ini)
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

    Serial.printf("[MQTT] Sent:%lu Fail:%lu Connected:%s Broker:%s:%s\n",
                  gwDiag.mqttSent, gwDiag.mqttFail,
                  mqttClient.connected() ? "YES" : "NO",
                  mqttHost, mqttPortStr);
    Serial.printf("[RADIO] RX:%lu OK:%lu CsErr:%lu Ignored:%lu\n",
                  gwDiag.rxTotal, gwDiag.rxOk, gwDiag.rxCsErr, gwDiag.rxIgnored);
    Serial.printf("[FS] Saved:%lu Forwarded:%lu SaveFail:%lu Ready:%s\n",
                  gwDiag.fsSaved, gwDiag.fsForwarded, gwDiag.fsSaveFail,
                  fsReady ? "YES" : "NO");
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
    Serial.printf( "═ MQTT Broker : %s:%s\n", mqttHost, mqttPortStr);
    Serial.printf( "═ MQTT Sent   : %lu\n", gwDiag.mqttSent);
    Serial.printf( "═ MQTT Fail   : %lu\n", gwDiag.mqttFail);
    Serial.printf( "═ MQTT Status : %s\n", mqttClient.connected() ? "CONNECTED" : "DISCONNECTED");
    Serial.printf( "═ WiFi SSID   : %s\n", WiFi.SSID().c_str());
    Serial.printf( "═ WiFi Status : %s\n",
                   WiFi.status() == WL_CONNECTED
                     ? WiFi.localIP().toString().c_str()
                     : "DISCONNECTED");
    Serial.printf( "═ OTA         : %s (host:%s port:3232)\n",
                   otaStarted ? "READY" : "BELUM AKTIF", OTA_HOSTNAME);
    Serial.println("═══════════════════════════════════════════════");
    Serial.printf( "═ FS Ready    : %s\n", fsReady ? "YES" : "NO");
    Serial.printf( "═ FS Saved    : %lu (data yg masuk ke queue)\n", gwDiag.fsSaved);
    Serial.printf( "═ FS Forward  : %lu (data yg berhasil di-forward)\n", gwDiag.fsForwarded);
    Serial.printf( "═ FS SaveFail : %lu\n", gwDiag.fsSaveFail);
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

void printConfig() {
    Serial.println("\n----- KONFIGURASI SAAT INI -----");
    Serial.printf("WiFi SSID     : %s\n", WiFi.SSID().c_str());
    Serial.printf("WiFi IP       : %s\n",
                  WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "(tidak terhubung)");
    Serial.printf("MQTT Broker   : %s\n", mqttHost);
    Serial.printf("MQTT Port     : %s\n", mqttPortStr);
    Serial.printf("OTA Hostname  : %s.local (port 3232)\n", OTA_HOSTNAME);
    Serial.printf("AP Setup Name : %s\n", AP_NAME);
    Serial.println("---------------------------------\n");
}

// ============================================================
// SERIAL COMMAND HANDLER
// ============================================================
void printHelp() {
    Serial.println("\n═══════════════════════════════════════════════");
    Serial.println("═    TPMS GATEWAY v6 – SERIAL COMMANDS       ═");
    Serial.println("═══════════════════════════════════════════════");
    Serial.println("═  help                 Tampilkan perintah   ═");
    Serial.println("═  scan                 Scan node aktif      ═");
    Serial.println("═  pair <ID>            Pair node (e.g. pair TRL001) ═");
    Serial.println("═  unpair <ID>          Unpair node          ═");
    Serial.println("═  unpair_all           Unpair semua node    ═");
    Serial.println("═  list                 Daftar semua node    ═");
    Serial.println("═  status               Data tire per node   ═");
    Serial.println("═  json                 Output JSON ke serial═");
    Serial.println("═  diag                 Diagnostik radio/FS  ═");
    Serial.println("═  diag_reset           Reset diagnostik     ═");
    Serial.println("═  mqtt_status          Cek koneksi MQTT     ═");
    Serial.println("═  queue                Info queue LittleFS  ═");
    Serial.println("═  clearqueue           Hapus queue manual   ═");
    Serial.println("═───────────────────────────────────────────═");
    Serial.println("═  show_config          Lihat konfigurasi    ═");
    Serial.println("═  set_broker <h> [p]   Ganti broker cepat   ═");
    Serial.println("═  wifi_config          Buka portal WiFi/MQTT═");
    Serial.println("═  wifi_reset           Hapus kredensial WiFi═");
    Serial.println("═══════════════════════════════════════════════");
    Serial.println("═  MQTT Topic: tpms/<nodeId>  (1 JSON 16 ban)═");
    Serial.printf( "═  Data stale: >%lu menit → NO_PACKET        ═\n",
                   DATA_STALE_MS / 60000UL);
    Serial.println("═  Store&Forward: data gagal kirim disimpan  ═");
    Serial.println("═  ke LittleFS, dikirim ulang otomatis saat  ═");
    Serial.println("═  MQTT kembali tersambung.                  ═");
    Serial.println("═  OTA: upload firmware lewat network port   ═");
    Serial.printf( "═  (%s, password OTA tersendiri)     ═\n", OTA_HOSTNAME);
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

    if (cmd == "list")       { printList();      return; }
    if (cmd == "status")     { printStatus();     return; }
    if (cmd == "json")       { sendAllJson();     return; }
    if (cmd == "diag")       { printDiag();       return; }
    if (cmd == "queue")      { printQueueInfo();  return; }
    if (cmd == "clearqueue") { clearQueueFS();    return; }
    if (cmd == "show_config"){ printConfig();     return; }

    if (cmd == "diag_reset") {
        gwDiag = GwDiag{};
        diagStartTime = millis();
        Serial.println("[OK] Diagnostic direset.");
        return;
    }

    if (cmd == "mqtt_status") {
        Serial.printf("[MQTT] Status: %s | Broker: %s:%s\n",
                      mqttClient.connected() ? "CONNECTED" : "DISCONNECTED",
                      mqttHost, mqttPortStr);
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

    // --- Ganti broker MQTT cepat tanpa buka portal ---
    if (input.startsWith("set_broker ") || input.startsWith("SET_BROKER ")) {
        String rest = input.substring(11);
        rest.trim();
        if (rest.length() == 0) {
            Serial.println("[ERROR] Format: set_broker <host> [port]");
            return;
        }
        int sp = rest.indexOf(' ');
        String host, portS;
        if (sp == -1) {
            host  = rest;
            portS = String(mqttPortStr);   // port tidak diubah
        } else {
            host  = rest.substring(0, sp);
            portS = rest.substring(sp + 1);
            portS.trim();
        }
        if (host.length() == 0 || host.length() >= sizeof(mqttHost)) {
            Serial.println("[ERROR] Host tidak valid.");
            return;
        }
        host.toCharArray(mqttHost, sizeof(mqttHost));
        portS.toCharArray(mqttPortStr, sizeof(mqttPortStr));
        saveBrokerConfig();
        Serial.println("[CFG] Broker baru disimpan. Restart untuk menerapkan...");
        delay(1200);
        ESP.restart();
        return;
    }

    if (cmd == "wifi_config") {
        forceConfigPortal();   // fungsi ini akan ESP.restart() di akhir
        return;
    }

    if (cmd == "wifi_reset") {
        wm.resetSettings();
        Serial.println("[WIFI] Kredensial WiFi dihapus. Restart ke mode setup...");
        delay(1200);
        ESP.restart();
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
    Serial.println("\n=== TPMS GATEWAY v6 – OTA + WiFiManager + Store&Forward ===");

    // --- LittleFS ---
    if (LittleFS.begin(true)) {
        fsReady = true;
        Serial.println("[FS] LittleFS OK");
    } else {
        fsReady = false;
        Serial.println("[FS] LittleFS GAGAL! Fitur store&forward NONAKTIF.");
    }

    // --- Radio (tidak tergantung WiFi, jalan duluan) ---
    initRadio();

    // --- WiFi + MQTT broker config (WiFiManager) ---
    Serial.println("[WIFI] Mencoba konek dgn kredensial tersimpan...");
    Serial.printf("       (kalau belum pernah setup, AP \"%s\" akan terbuka)\n", AP_NAME);
    bool wifiOk = setupWifiAndConfig();

    mqttClient.setKeepAlive(60);
    // PENTING: buffer diperbesar ke 2560 byte untuk JSON 16 ban
    // Default PubSubClient hanya 256 byte – tidak cukup untuk 16 ban!
    mqttClient.setBufferSize(2560);

    if (wifiOk) {
        Serial.print("[WIFI] Terhubung. IP: ");
        Serial.println(WiFi.localIP());

        mqttClient.setServer(mqttHost, atoi(mqttPortStr));
        mqttServerConfigured = true;
        mqttReconnect();

        setupOTA();
    } else {
        Serial.println("[WIFI] Tidak terhubung (timeout config portal).");
        Serial.println("       Gateway tetap berjalan OFFLINE, data tersimpan ke LittleFS.");
        Serial.println("       Ketik 'wifi_config' kapan saja untuk setup ulang WiFi/MQTT.");
    }

    loadPairedNodes();
    diagStartTime = millis();

    printHelp();

    if (fsReady && LittleFS.exists(QUEUE_FILE)) {
        Serial.println("[FS] Ditemukan queue tersisa dari sesi sebelumnya:");
        printQueueInfo();
    }

    Serial.println("[GATEWAY] Siap. Menunggu paket dari node...");
}

// ============================================================
// LOOP
// ============================================================
unsigned long lastReportTime      = 0;
unsigned long lastPeriodicPublish = 0;
unsigned long lastMqttPing        = 0;
unsigned long lastWifiRetry       = 0;

// Laporan serial setiap 30 detik
#define REPORT_INTERVAL_MS   30000
// Publish periodik bundle semua node setiap 60 detik
// (bahkan jika tidak ada paket baru masuk – agar NO_PACKET terdeteksi)
#define PERIODIC_PUBLISH_MS  60000
#define MQTT_PING_MS         10000
#define WIFI_RETRY_MS        15000

void loop() {
    handleSerial();
    processIncoming();

    if (WiFi.status() == WL_CONNECTED) {

        // Aktifkan OTA & MQTT server sekali setelah WiFi connect
        // (berlaku juga kalau WiFi tadinya gagal di setup() lalu
        // konek kemudian lewat WiFi.reconnect() di bawah)
        if (!mqttServerConfigured) {
            mqttClient.setServer(mqttHost, atoi(mqttPortStr));
            mqttServerConfigured = true;
        }
        if (!otaStarted) {
            setupOTA();
        }

        ArduinoOTA.handle();
        mqttClient.loop();

        unsigned long now = millis();
        if (now - lastMqttPing > MQTT_PING_MS && !mqttClient.connected()) {
            lastMqttPing = now;
            mqttReconnect();
        }

        // Begitu MQTT connect, coba forward isi queue LittleFS secara periodik
        if (mqttClient.connected() &&
            now - lastFsProcess > FS_PROCESS_INTERVAL_MS) {
            lastFsProcess = now;
            processStoredData();
        }

    } else {
        // WiFi putus -> coba reconnect berkala memakai kredensial tersimpan
        // (TIDAK membuka config portal otomatis; itu hanya manual lewat
        // serial command "wifi_config" supaya tidak mengganggu operasi)
        unsigned long now = millis();
        if (now - lastWifiRetry > WIFI_RETRY_MS) {
            lastWifiRetry = now;
            Serial.println("[WIFI] Terputus, mencoba reconnect...");
            WiFi.reconnect();
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
    // (kalau gagal/offline, otomatis disimpan ke LittleFS di dalam publishNodeBundle)
    if (now - lastPeriodicPublish > PERIODIC_PUBLISH_MS) {
        lastPeriodicPublish = now;
        for (int i = 0; i < nodeCount; i++) {
            if (nodes[i].paired) {
                publishNodeBundle(i);
            }
        }
    }
}

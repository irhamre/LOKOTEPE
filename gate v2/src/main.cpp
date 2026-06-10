#include <RadioLib.h>
#include <SPI.h>
#include <ArduinoJson.h>

SX1278 radio = new Module(18, 26, 23, RADIOLIB_NC);

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

#define TIRE_COUNT 30

TpmsData      cache[TIRE_COUNT];
unsigned long lastUpdate[TIRE_COUNT]      = {0};
bool          dataInitialized[TIRE_COUNT] = {false};

// ============================================================
// RADIO DIAGNOSTIC
// ============================================================
struct RadioDiag {
    uint32_t rxTotal          = 0;
    uint32_t rxOk             = 0;
    uint32_t rxChecksumErr    = 0;
    uint32_t rxIdErr          = 0;
    uint32_t rxPerTire[TIRE_COUNT] = {0};
    float    rssiSum          = 0;
    float    rssiMin          = 0;
    float    rssiMax          = -200;
    float    rssiLast         = 0;
    float    snrSum           = 0;
    float    snrMin           = 999;
    float    snrMax           = -999;
    float    snrLast          = 0;
    uint32_t latencySum       = 0;
    uint32_t latencyMin       = 0xFFFFFFFF;
    uint32_t latencyMax       = 0;
    uint32_t latencyLast      = 0;
    uint32_t latencyCount     = 0;
    uint32_t expectedSeq[TIRE_COUNT] = {0};
};

RadioDiag     diag;
unsigned long diagStartTime = 0;
unsigned long lastRxTime[TIRE_COUNT] = {0};
uint32_t      rxInterval[TIRE_COUNT] = {0};

void updateDiag(const TpmsData& d, float rssi, float snr) {
    diag.rssiSum += rssi;
    diag.rssiLast = rssi;
    if (rssi < diag.rssiMin) diag.rssiMin = rssi;
    if (rssi > diag.rssiMax) diag.rssiMax = rssi;

    diag.snrSum += snr;
    diag.snrLast = snr;
    if (snr < diag.snrMin) diag.snrMin = snr;
    if (snr > diag.snrMax) diag.snrMax = snr;

    uint32_t now = millis();
    if (d.timestamp < now) {
        uint32_t lat = now - d.timestamp;
        if (lat < 30000) {
            diag.latencyLast = lat;
            diag.latencySum += lat;
            if (lat < diag.latencyMin) diag.latencyMin = lat;
            if (lat > diag.latencyMax) diag.latencyMax = lat;
            diag.latencyCount++;
        }
    }

    if (lastRxTime[d.tireId] > 0)
        rxInterval[d.tireId] = now - lastRxTime[d.tireId];
    lastRxTime[d.tireId] = now;

    diag.rxPerTire[d.tireId]++;
    diag.expectedSeq[d.tireId]++;
}

void printRadioDiag() {
    unsigned long uptime  = (millis() - diagStartTime) / 1000;
    float loss    = diag.rxTotal > 0 ? ((diag.rxTotal - diag.rxOk) * 100.0f / diag.rxTotal) : 0;
    float ok_rate = diag.rxTotal > 0 ? (diag.rxOk * 100.0f / diag.rxTotal) : 0;
    float rssiAvg = diag.rxOk > 0 ? diag.rssiSum / diag.rxOk : 0;
    float snrAvg  = diag.rxOk > 0 ? diag.snrSum  / diag.rxOk : 0;
    uint32_t latAvg = diag.latencyCount > 0 ? diag.latencySum / diag.latencyCount : 0;

    Serial.println();
    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║     RADIO RX DIAGNOSTIC (30 SENSOR)     ║");
    Serial.println("╠══════════════════════════════════════════╣");
    Serial.printf( "║ Uptime         : %lu detik\n", uptime);
    Serial.printf( "║ Total RX       : %lu\n", diag.rxTotal);
    Serial.printf( "║ Valid OK        : %lu (%.1f%%)\n", diag.rxOk, ok_rate);
    Serial.printf( "║ Checksum Error  : %lu\n", diag.rxChecksumErr);
    Serial.printf( "║ ID Error        : %lu\n", diag.rxIdErr);
    Serial.printf( "║ Packet Loss     : %.1f%%\n", loss);
    Serial.println("╠══════════════════════════════════════════╣");
    Serial.printf( "║ RSSI Last/Avg   : %.1f / %.1f dBm\n", diag.rssiLast, rssiAvg);
    Serial.printf( "║ RSSI Min/Max    : %.1f / %.1f dBm\n", diag.rssiMin, diag.rssiMax);
    Serial.printf( "║ SNR Last/Avg    : %.1f / %.1f dB\n", diag.snrLast, snrAvg);
    Serial.printf( "║ Latency Avg     : %lu ms | Max: %lu ms\n", latAvg, diag.latencyMax);
    Serial.println("╠══════════════════════════════════════════╣");
    for (int i = 0; i < TIRE_COUNT; i++) {
        if (rxInterval[i] > 0)
            Serial.printf("║   Tire %-2d : %4lu pkt | ~%lums\n", i, diag.rxPerTire[i], rxInterval[i]);
        else
            Serial.printf("║   Tire %-2d : %4lu pkt | -\n", i, diag.rxPerTire[i]);
    }
    Serial.println("╚══════════════════════════════════════════╝");
}

void sendDiagJson() {
    float rssiAvg = diag.rxOk > 0 ? diag.rssiSum / diag.rxOk : 0;
    float snrAvg  = diag.rxOk > 0 ? diag.snrSum  / diag.rxOk : 0;
    uint32_t latAvg = diag.latencyCount > 0 ? diag.latencySum / diag.latencyCount : 0;

    JsonDocument doc;
    doc["type"]            = "diag";
    doc["uptime_s"]        = (millis() - diagStartTime) / 1000;
    doc["rx_total"]        = diag.rxTotal;
    doc["rx_ok"]           = diag.rxOk;
    doc["rx_cs_err"]       = diag.rxChecksumErr;
    doc["rx_id_err"]       = diag.rxIdErr;
    doc["success_rate"]    = diag.rxTotal > 0 ? (diag.rxOk * 100.0f / diag.rxTotal) : 0;
    doc["packet_loss_pct"] = diag.rxTotal > 0 ? ((diag.rxTotal - diag.rxOk) * 100.0f / diag.rxTotal) : 0;
    doc["rssi_last"]       = diag.rssiLast;
    doc["rssi_avg"]        = rssiAvg;
    doc["rssi_min"]        = diag.rssiMin;
    doc["rssi_max"]        = diag.rssiMax;
    doc["snr_avg"]         = snrAvg;
    doc["latency_avg_ms"]  = latAvg;
    doc["latency_max_ms"]  = diag.latencyMax;

    JsonArray tireArr = doc["per_tire"].to<JsonArray>();
    for (int i = 0; i < TIRE_COUNT; i++) {
        JsonObject t = tireArr.add<JsonObject>();
        t["id"]          = i;
        t["rx_count"]    = diag.rxPerTire[i];
        t["interval_ms"] = rxInterval[i];
    }
    serializeJson(doc, Serial);
    Serial.println();
}

// ============================================================
// INTERRUPT
// ============================================================
volatile bool packetReceived = false;
void IRAM_ATTR setFlag() { packetReceived = true; }

// ============================================================
// REPORT & JSON
// ============================================================
void printReport() {
    unsigned long detik = millis() / 1000;
    Serial.println();
    Serial.println("╔══════════════════════════════════════════╗");
    Serial.printf( "║ TPMS REPORT | %02lu:%02lu:%02lu | 30 sensor\n",
                   detik/3600, (detik%3600)/60, detik%60);
    Serial.println("╠══════════════════════════════════════════╣");
    Serial.printf( "║ %-7s | %-10s | %5s | %3s | %5s | BLE ║\n",
                   "Tire","Status","kPa","°C","Volt");
    Serial.println("╠══════════════════════════════════════════╣");
    for (int i = 0; i < TIRE_COUNT; i++) {
        unsigned long age = lastUpdate[i] > 0 ? (millis() - lastUpdate[i]) / 1000 : 0;
        bool stale = !dataInitialized[i] || (millis() - lastUpdate[i] > 100000);
        if (stale) {
            Serial.printf("║ Tire %-2d | %-10s | %5s | %3s | %5s |     ║\n",
                          i, "NO UPDATE", "-", "-", "-");
        } else {
            Serial.printf("║ Tire %-2d | OK         | %5.1f | %3d | %5.2f | %3d ║ %lus\n",
                          i,
                          cache[i].kpa_x10 / 10.0f,
                          cache[i].temp,
                          cache[i].volt_mV / 1000.0f,
                          cache[i].bleRssi,
                          age);
        }
    }
    Serial.println("╚══════════════════════════════════════════╝");
}

void sendJson() {
    JsonDocument doc;
    doc["type"]     = "tpms";
    doc["uptime_s"] = millis() / 1000;
    doc["rssi_rf"]  = diag.rssiLast;
    doc["snr_rf"]   = diag.snrLast;

    JsonArray tires = doc["tires"].to<JsonArray>();
    for (int i = 0; i < TIRE_COUNT; i++) {
        JsonObject tire = tires.add<JsonObject>();
        tire["id"] = i;
        bool stale = !dataInitialized[i] || (millis() - lastUpdate[i] > 100000);
        if (stale) {
            tire["status"] = "NO_UPDATE";
        } else {
            tire["status"]   = "OK";
            tire["kpa"]      = cache[i].kpa_x10 / 10.0f;
            tire["temp_c"]   = cache[i].temp;
            tire["volt_v"]   = cache[i].volt_mV / 1000.0f;
            tire["rssi_ble"] = cache[i].bleRssi;
            tire["age_s"]    = (millis() - lastUpdate[i]) / 1000;
        }
    }
    serializeJson(doc, Serial);
    Serial.println();
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== TPMS GATEWAY AGGREGATOR (30 SENSOR) ===");

    SPI.begin(5, 19, 27, 18);

    if (radio.beginFSK(434.0, 9.6, 50.0, 250.0, 10, 32) != RADIOLIB_ERR_NONE) {
        Serial.println("[ERR] Radio init gagal!");
        while (1);
    }

    uint8_t syncWord[] = {0x12, 0xAD};
    radio.setSyncWord(syncWord, 2);
    radio.setCrcFiltering(false);
    radio.fixedPacketLengthMode(sizeof(TpmsData));
    radio.setDio0Action(setFlag, RISING);
    radio.startReceive();

    diag.rssiMin  = 0;
    diagStartTime = millis();

    Serial.println("Receiver Ready.");
    Serial.printf("Paket: %d byte | tireId 0-29\n", sizeof(TpmsData));
    Serial.println("Perintah: diag | diag_reset | report | json");
}

// ============================================================
// LOOP
// ============================================================
unsigned long lastReportTime = 0;
unsigned long lastDiagTime   = 0;
#define REPORT_INTERVAL_MS 10000
#define DIAG_INTERVAL_MS   30000

void loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim(); cmd.toLowerCase();
        if      (cmd == "diag")       { printRadioDiag(); sendDiagJson(); }
        else if (cmd == "diag_reset") {
            diag = RadioDiag{};
            diag.rssiMin = 0;
            diagStartTime = millis();
            memset(lastRxTime, 0, sizeof(lastRxTime));
            memset(rxInterval, 0, sizeof(rxInterval));
            Serial.println("[OK] Diagnostic direset.");
        }
        else if (cmd == "report") printReport();
        else if (cmd == "json")   sendJson();
        else Serial.println("Perintah: diag | diag_reset | report | json");
    }

    if (packetReceived) {
        packetReceived = false;

        TpmsData d;
        int state = radio.readData((uint8_t*)&d, sizeof(TpmsData));

        if (state == RADIOLIB_ERR_NONE) {
            diag.rxTotal++;
            float rssi = radio.getRSSI();
            float snr  = radio.getSNR();

            uint8_t cs = 0;
            uint8_t* p = (uint8_t*)&d;
            for (int i = 0; i < (int)sizeof(TpmsData) - 1; i++) cs ^= p[i];

            if (cs != d.chksum) {
                diag.rxChecksumErr++;
                Serial.printf("[ERR] Checksum | RSSI:%.1f\n", rssi);
            } else if (d.tireId >= TIRE_COUNT) {
                diag.rxIdErr++;
                Serial.printf("[ERR] ID invalid: %d\n", d.tireId);
            } else {
                diag.rxOk++;
                updateDiag(d, rssi, snr);

                cache[d.tireId]           = d;
                lastUpdate[d.tireId]      = millis();
                dataInitialized[d.tireId] = true;

                Serial.printf("[RX] ID:%-2d | %.1fkPa | %dC | %.2fV | RF:%.1fdBm | BLE:%ddBm | Lat:%lums\n",
                              d.tireId,
                              d.kpa_x10 / 10.0f,
                              d.temp,
                              d.volt_mV / 1000.0f,
                              rssi,
                              d.bleRssi,
                              diag.latencyLast);
            }
        }
        radio.startReceive();
    }

    unsigned long now = millis();
    if (now - lastReportTime >= REPORT_INTERVAL_MS) {
        lastReportTime = now;
        printReport();
        sendJson();
    }
    if (now - lastDiagTime >= DIAG_INTERVAL_MS) {
        lastDiagTime = now;
        printRadioDiag();
    }
}
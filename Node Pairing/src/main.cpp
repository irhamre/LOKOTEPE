/*

 * ============================================================

 * TPMS NODE v4.2 – ESP32U + LilyGo XY LoRa Shield (SX1276)

 * 16 Sensor BLE | P2P ke Gateway v4

 * Hardware  : ESP32U DevKit + LilyGo XY LoRa Shield

 * Radio     : SX1276 (LoRa)

 *   SCK=18  MISO=19  MOSI=23  SS=5  RST=14  DIO0=26

 * Library   : arduino-LoRa (sandeepmistry), NimBLE-Arduino

 *

 * NODE_ID   -> nama device BLE: "TPMS_TRL001"

 * Terdeteksi di HP seperti TWS/smartwatch.

 * App yang cocok: "Serial Bluetooth Terminal" atau "nRF Connect"

 *

 * BLE UART Service (Nordic UART Standard):

 *   Service : 6E400001-B5A3-F393-E0A9-E50E24DCCA9E

 *   TX Char : 6E400003-...  (Node->HP, enable Notify)

 *   RX Char : 6E400002-...  (HP->Node, Write)

 *

 * PERINTAH via BLE / Serial:

 *   P-<0..15>    Auto-pair slot (dekatkan sensor, kirim P-0 dst)

 *   status       Daftar sensor terpasang

 *   report       Data tekanan/suhu semua ban

 *   unpair-<pos> Hapus pairing satu posisi

 *   unpair_all   Hapus semua

 *   diag         Statistik radio TX

 *   help         Daftar perintah

 * ============================================================

 */



#include <Arduino.h>

#include <SPI.h>

#include <LoRa.h>

#include <NimBLEDevice.h>

#include <NimBLEServer.h>

#include <NimBLEUtils.h>

#include <Preferences.h>



// ============================================================

// *** GANTI NODE_ID & BLE_DEV_NAME SESUAI UNIT ***

// ============================================================

#define NODE_ID       "TRL001"

#define BLE_DEV_NAME  "TPMS_TRL001"   // Nama muncul di list Bluetooth HP



// ============================================================

// PIN LoRa SX1276

// ============================================================

#define LORA_SS    5

#define LORA_RST  14

#define LORA_DIO0 26

#define LORA_SCK  18

#define LORA_MISO 19

#define LORA_MOSI 23

#define LORA_FREQ 922E6



// ============================================================

// KONFIGURASI NODE

// ============================================================

#define TIRE_COUNT         16

#define TX_QUEUE_SIZE      64

#define TX_INTERVAL_MS     300

#define SCAN_RESTART_MS    120000

#define PRINT_INTERVAL_MS  10000



// ============================================================

// KONFIGURASI AUTO-PAIR

// ============================================================

#define AUTOPAIR_SCAN_SEC   10

#define AUTOPAIR_RSSI_MIN  -50    // dBm terjauh yang diterima

#define AUTOPAIR_RSSI_MAX  -30    // dBm terdekat (hindari 0/saturasi)



// ============================================================

// Nordic UART Service UUIDs

// ============================================================

#define NUS_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"

#define NUS_RX_UUID       "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

#define NUS_TX_UUID       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"



// ============================================================

// PROTOCOL PAKET LORA

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

// TX QUEUE (LoRa)

// ============================================================

TpmsData      txQueue[TX_QUEUE_SIZE];

volatile int  txHead     = 0;

volatile int  txTail     = 0;

unsigned long lastTxTime = 0;

bool          radioReady = false;



// ============================================================

// RADIO DIAG

// ============================================================

struct RadioDiag {

    uint32_t txCount               = 0;

    uint32_t txOk                  = 0;

    uint32_t txErr                 = 0;

    uint32_t txPerTire[TIRE_COUNT] = {0};

    uint32_t queueDropped          = 0;

};

RadioDiag     diag;

unsigned long diagStartTime = 0;



// ============================================================

// BLE SERVER (GATT / Nordic UART)

// ============================================================

NimBLEServer*         pServer      = nullptr;

NimBLECharacteristic* pTxChar      = nullptr;

NimBLECharacteristic* pRxChar      = nullptr;

bool                  bleConnected = false;



// Buffer TX ke HP

#define BLE_TX_BUF_SIZE 16

String        bleTxBuf[BLE_TX_BUF_SIZE];

volatile int  bleTxHead = 0;

volatile int  bleTxTail = 0;



// Antri string ke HP (non-blocking)

void bleSend(const String& s) {

    Serial.print("[BLE->HP] "); Serial.println(s);

    if (!bleConnected || pTxChar == nullptr) return;

    int next = (bleTxHead + 1) % BLE_TX_BUF_SIZE;

    if (next == bleTxTail) return;

    bleTxBuf[bleTxHead] = s;

    bleTxHead = next;

}



// Flush satu item dari buffer ke HP via notify

void bleFlushTx() {

    if (!bleConnected || pTxChar == nullptr) return;

    if (bleTxHead == bleTxTail) return;

    String s = bleTxBuf[bleTxTail];

    bleTxTail = (bleTxTail + 1) % BLE_TX_BUF_SIZE;

    s += "\n";

    int len = s.length();

    int offset = 0;

    while (offset < len) {

        int chunk = min(20, len - offset);

        pTxChar->setValue((uint8_t*)(s.c_str() + offset), chunk);

        pTxChar->notify();

        offset += chunk;

        delay(10);

    }

}



// ============================================================

// TIRE LABELS & COMMANDS

// ============================================================

const char* tireLabels[TIRE_COUNT] = {

    "FL",  "FL-2", "FR",   "FR-2",

    "RL",  "RL-2", "RR",   "RR-2",

    "ML",  "ML-2", "MR",   "MR-2",

    "R2L", "R2L-2","R2R",  "R2R-2"

};



const char* tireCommands[TIRE_COUNT] = {

    "fl",  "fl-2", "fr",   "fr-2",

    "rl",  "rl-2", "rr",   "rr-2",

    "ml",  "ml-2", "mr",   "mr-2",

    "r2l", "r2l-2","r2r",  "r2r-2"

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



TireData      tireData[TIRE_COUNT];

unsigned long lastSeenBLE[TIRE_COUNT] = {0};

unsigned long lastPrintTime           = 0;



// ============================================================

// AUTO-PAIR STATE MACHINE

// ============================================================

enum AutoPairState { AP_IDLE, AP_SCANNING, AP_DONE };



struct AutoPairCandidate { std::string mac; int rssi; };



AutoPairState apState      = AP_IDLE;

int           apTargetSlot = -1;

bool          apToBle      = false;

std::vector<AutoPairCandidate> apCandidates;



void onAutoPairScanDone(NimBLEScanResults results) { apState = AP_DONE; }



// ============================================================

// LEGACY SCAN MODE

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

// FLASH STORAGE

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

        default: char buf[12]; snprintf(buf, sizeof(buf), "UNK_%04X", code); return String(buf);

    }

}



// ============================================================

// QUEUE HELPER (LoRa)

// ============================================================

bool txQueuePush(const TpmsData& d) {

    int pos = txTail;

    while (pos != txHead) {

        if (txQueue[pos].tireId == d.tireId) { txQueue[pos] = d; return true; }

        pos = (pos + 1) % TX_QUEUE_SIZE;

    }

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



// ============================================================

// RADIO

// ============================================================

void initRadio() {

    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);

    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

    if (!LoRa.begin(LORA_FREQ)) {

        Serial.println("[LoRa] Init GAGAL!");

        return;

    }

    LoRa.setSpreadingFactor(7);

    LoRa.setSignalBandwidth(125E3);

    LoRa.setCodingRate4(5);

    LoRa.setTxPower(20);

    LoRa.setSyncWord(0x12);

    LoRa.enableCrc();

    radioReady = true;

    Serial.printf("[LoRa] Siap. Node:%s Freq:%ld SF7 BW125\n", NODE_ID, (long)LORA_FREQ);

}



void processRadioQueue() {

    if (!radioReady) return;

    if (millis() - lastTxTime < TX_INTERVAL_MS) return;

    TpmsData d;

    if (!txQueuePop(d)) return;

    lastTxTime = millis();



    LoRaPacket pkt;

    pkt.hdr.magic     = PKT_MAGIC;

    pkt.hdr.ver       = PKT_VER;

    strncpy(pkt.hdr.nodeId, NODE_ID, 7);

    pkt.hdr.tireCount = TIRE_COUNT;

    pkt.data          = d;



    diag.txCount++;

    diag.txPerTire[d.tireId]++;



    LoRa.beginPacket();

    LoRa.write((uint8_t*)&pkt, sizeof(LoRaPacket));

    if (LoRa.endPacket() == 1) {

        diag.txOk++;

        Serial.printf("[TX] ID:%-2d %.1fkPa %dC %.2fV Q:%d\n",

                      d.tireId, d.kpa_x10/10.0f, d.temp, d.volt_mV/1000.0f,

                      (txHead-txTail+TX_QUEUE_SIZE)%TX_QUEUE_SIZE);

    } else {

        diag.txErr++;

        Serial.printf("[TX ERR] ID:%d\n", d.tireId);

    }

}



// ============================================================

// OUTPUT HELPERS (Serial + BLE)

// ============================================================

void outLine(const char* s, bool toBle) {

    Serial.println(s);

    if (toBle) bleSend(String(s));

}



void printRadioDiag(bool toBle) {

    unsigned long uptime = (millis()-diagStartTime)/1000;

    float txRate = diag.txCount>0 ? (diag.txOk*100.0f/diag.txCount) : 0;

    char buf[80];

    outLine("=DIAG=", toBle);

    snprintf(buf,sizeof(buf),"Up:%lus TX:%lu OK:%lu ERR:%lu Drop:%lu %.1f%%",

             uptime,diag.txCount,diag.txOk,diag.txErr,diag.queueDropped,txRate);

    outLine(buf, toBle);

    for (int i=0;i<TIRE_COUNT;i++) {

        if (diag.txPerTire[i]>0) {

            snprintf(buf,sizeof(buf),"T%d(%s):%lu",i,tireLabels[i],diag.txPerTire[i]);

            outLine(buf, toBle);

        }

    }

    outLine("=END=", toBle);

}



void sendAllTires(bool toBle) {

    bool ada = false;

    for (int i=0;i<TIRE_COUNT;i++) if (tireData[i].hasData||!pairedMacs[i].empty()){ada=true;break;}

    if (!ada) { outLine("No data.", toBle); return; }



    char buf[80];

    unsigned long detik = millis()/1000;

    snprintf(buf,sizeof(buf),"=REPORT %s %02lu:%02lu:%02lu=",

             NODE_ID,detik/3600,(detik%3600)/60,detik%60);

    outLine(buf, toBle);



    for (int i=0;i<TIRE_COUNT;i++) {

        if (pairedMacs[i].empty()) continue;

        if (tireData[i].hasData) {

            unsigned long sel=(millis()-tireData[i].lastSeen)/1000;

            snprintf(buf,sizeof(buf),"%s|%s|%.1fkPa|%dC|%.2fV|%ds ago",

                     tireLabels[i],tireData[i].statusStr.c_str(),

                     tireData[i].kpa,tireData[i].celsius,tireData[i].voltage,(int)sel);

        } else {

            snprintf(buf,sizeof(buf),"%s|NO DATA",tireLabels[i]);

        }

        outLine(buf, toBle);

    }

    outLine("=END=", toBle);

}



void sendStatus(bool toBle) {

    char buf[60];

    snprintf(buf,sizeof(buf),"=STATUS %s=",NODE_ID);

    outLine(buf, toBle);

    for (int i=0;i<TIRE_COUNT;i++) {

        snprintf(buf,sizeof(buf),"[%2d]%s:%s",i,tireLabels[i],

                 pairedMacs[i].empty()?"EMPTY":pairedMacs[i].c_str());

        outLine(buf, toBle);

    }

    outLine("=END=", toBle);

}



// ============================================================

// AUTO-PAIR

// ============================================================

void startAutoPair(int slot, bool toBle) {

    if (slot<0||slot>=TIRE_COUNT) {

        outLine("ERROR slot 0..15", toBle); return;

    }

    if (apState==AP_SCANNING) { outLine("BUSY tunggu scan selesai", toBle); return; }

    if (scanTpmsMode)          { outLine("BUSY scan_tpms aktif", toBle);    return; }

    if (!pairedMacs[slot].empty()) {

        char buf[60];

        snprintf(buf,sizeof(buf),"ERROR Slot%d(%s) terisi. unpair-%s dulu.",

                 slot,tireLabels[slot],tireCommands[slot]);

        outLine(buf, toBle); return;

    }



    apTargetSlot = slot;

    apToBle      = toBle;

    apCandidates.clear();

    apState      = AP_SCANNING;



    char buf[80];

    snprintf(buf,sizeof(buf),"AutoPair slot%d(%s) scan%ds RSSI>%ddBm...",

             slot,tireLabels[slot],AUTOPAIR_SCAN_SEC,AUTOPAIR_RSSI_MIN);

    outLine(buf, toBle);



    NimBLEDevice::getScan()->stop();

    NimBLEDevice::getScan()->clearResults();

    NimBLEDevice::getScan()->start(AUTOPAIR_SCAN_SEC, onAutoPairScanDone, false);

}



void processAutoPairResult() {

    if (apState != AP_DONE) return;

    apState = AP_IDLE;

    bool toBle = apToBle;

    char buf[80];



    if (apCandidates.empty()) {

        snprintf(buf,sizeof(buf),"FAIL no sensor RSSI%d~%d for slot%d(%s)",

                 AUTOPAIR_RSSI_MIN,AUTOPAIR_RSSI_MAX,apTargetSlot,tireLabels[apTargetSlot]);

        outLine(buf, toBle);

        apTargetSlot=-1;

        NimBLEDevice::getScan()->start(0,nullptr,false);

        return;

    }



    AutoPairCandidate best = apCandidates[0];

    for (auto& c : apCandidates) if (c.rssi>best.rssi) best=c;



    pairedMacs[apTargetSlot] = best.mac;

    saveMacToFlash(apTargetSlot, String(best.mac.c_str()));

    tireData[apTargetSlot]    = TireData{};

    lastSeenBLE[apTargetSlot] = 0;



    snprintf(buf,sizeof(buf),"OK Slot%d(%s)=%s RSSI:%ddBm",

             apTargetSlot,tireLabels[apTargetSlot],best.mac.c_str(),best.rssi);

    outLine(buf, toBle);



    // Format parseable untuk app

    snprintf(buf,sizeof(buf),"AP_RESULT:%d,%s,%s,%d",

             apTargetSlot,tireLabels[apTargetSlot],best.mac.c_str(),best.rssi);

    outLine(buf, toBle);



    apTargetSlot=-1;

    apCandidates.clear();

    NimBLEDevice::getScan()->start(0,nullptr,false);

}



// ============================================================

// COMMAND HANDLER (Serial & BLE pakai fungsi yang sama)

// ============================================================

void handleCommand(String input, bool toBle) {

    input.trim();

    String lower = input; lower.toLowerCase();



    // P-<n> auto-pair

    if (lower.startsWith("p-")) {

        String slotStr = input.substring(2); slotStr.trim();

        bool valid = slotStr.length()>0;

        for (int i=0;i<(int)slotStr.length();i++) if (!isDigit(slotStr[i])){valid=false;break;}

        if (!valid) { outLine("ERROR P-<0..15> e.g. P-0 P-7", toBle); return; }

        startAutoPair(slotStr.toInt(), toBle);

        return;

    }



    input = lower;



    if (input=="help") {

        outLine("=CMD=",                    toBle);

        outLine("P-<0..15> AutoPair slot",  toBle);

        outLine("status    Daftar sensor",   toBle);

        outLine("report    Data ban",        toBle);

        outLine("unpair-<pos>",              toBle);

        outLine("unpair_all",                toBle);

        outLine("diag      Radio stat",      toBle);

        outLine("diag_reset",                toBle);

        outLine("scan_tpms 120s scan",       toBle);

        outLine("Slot:0=FL 1=FL2 2=FR 3=FR2",toBle);

        outLine("     4=RL 5=RL2 6=RR 7=RR2",toBle);

        outLine("     8=ML 9=ML2 10=MR 11=MR2",toBle);

        outLine("     12=R2L 13=R2L2 14=R2R 15=R2R2",toBle);

        outLine("=END=", toBle);

        return;

    }



    if (input=="status")     { sendStatus(toBle);     return; }

    if (input=="report")     { sendAllTires(toBle);   return; }

    if (input=="diag")       { printRadioDiag(toBle); return; }



    if (input=="diag_reset") {

        diag=RadioDiag{}; diagStartTime=millis();

        outLine("OK Diag reset.", toBle); return;

    }



    if (input=="unpair_all") {

        for (int i=0;i<TIRE_COUNT;i++) {

            pairedMacs[i]=""; saveMacToFlash(i,"");

            tireData[i]=TireData{}; lastSeenBLE[i]=0;

        }

        outLine("OK Semua unpair.", toBle); return;

    }



    if (input.startsWith("unpair-")) {

        String pos=input.substring(7);

        int idx=getIndexFromCommand(pos);

        if (idx==-1) { outLine("ERROR posisi invalid",toBle); return; }

        if (pairedMacs[idx].empty()) { outLine("ERROR belum pair",toBle); return; }

        pairedMacs[idx]=""; saveMacToFlash(idx,"");

        tireData[idx]=TireData{}; lastSeenBLE[idx]=0;

        char buf[40]; snprintf(buf,sizeof(buf),"OK %s unpaired",tireLabels[idx]);

        outLine(buf, toBle); return;

    }



    // Manual pair: p-fl <MAC>

    if (input.startsWith("p-")) {

        int sp=input.indexOf(' ');

        if (sp==-1){outLine("ERROR p-fl <MAC>",toBle);return;}

        String pos=input.substring(2,sp);

        String mac=input.substring(sp+1); mac.trim();

        if (mac.length()!=17){outLine("ERROR MAC xx:xx:xx:xx:xx:xx",toBle);return;}

        int idx=getIndexFromCommand(pos);

        if (idx==-1){outLine("ERROR posisi invalid",toBle);return;}

        if (!pairedMacs[idx].empty()){

            char buf[50]; snprintf(buf,sizeof(buf),"ERROR %s terisi, unpair dulu",tireLabels[idx]);

            outLine(buf,toBle); return;

        }

        if (isAlreadyPaired(std::string(mac.c_str()))){outLine("ERROR MAC di slot lain",toBle);return;}

        pairedMacs[idx]=std::string(mac.c_str());

        saveMacToFlash(idx,mac);

        tireData[idx]=TireData{}; lastSeenBLE[idx]=0;

        char buf[60]; snprintf(buf,sizeof(buf),"OK %s=%s",tireLabels[idx],mac.c_str());

        outLine(buf, toBle); return;

    }



    if (input=="scan_tpms") {

        scannedTpms.clear(); scanTpmsMode=true; scanTpmsDone=false;

        outLine("Scanning 120s...", toBle);

        NimBLEDevice::getScan()->stop();

        NimBLEDevice::getScan()->clearResults();

        NimBLEDevice::getScan()->start(120,onScanDone,false);

        return;

    }



    outLine("Unknown cmd. Type: help", toBle);

}



// ============================================================

// BLE SERVER CALLBACKS

// ============================================================

class ServerCallbacks : public NimBLEServerCallbacks {

    void onConnect(NimBLEServer* pSrv) override {

        bleConnected = true;

        Serial.println("[BLE-SRV] HP terhubung!");

        bleSend(String("TPMS ") + NODE_ID + " v4.2 OK. Type: help");

    }

    void onDisconnect(NimBLEServer* pSrv) override {

        bleConnected = false;

        Serial.println("[BLE-SRV] HP disconnect. Re-advertising...");

        NimBLEDevice::startAdvertising();

    }

};



class RxCallbacks : public NimBLECharacteristicCallbacks {

    void onWrite(NimBLECharacteristic* pChar) override {

        std::string val = pChar->getValue();

        if (val.empty()) return;

        String cmd = String(val.c_str()); cmd.trim();

        Serial.printf("[BLE<-HP] %s\n", cmd.c_str());

        handleCommand(cmd, true);

    }

};



// ============================================================

// INIT BLE SERVER

// ============================================================

void initBleServer() {

    pServer = NimBLEDevice::createServer();

    pServer->setCallbacks(new ServerCallbacks());



    NimBLEService* pSvc = pServer->createService(NUS_SERVICE_UUID);



    pTxChar = pSvc->createCharacteristic(NUS_TX_UUID, NIMBLE_PROPERTY::NOTIFY);

    pRxChar = pSvc->createCharacteristic(NUS_RX_UUID,

                  NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);

    pRxChar->setCallbacks(new RxCallbacks());



    pSvc->start();



    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();

    pAdv->addServiceUUID(NUS_SERVICE_UUID);

    pAdv->setScanResponse(true);

    pAdv->setMinPreferred(0x06);

    pAdv->setMaxPreferred(0x12);

    NimBLEDevice::startAdvertising();



    Serial.printf("[BLE-SRV] Advertising: \"%s\"\n", BLE_DEV_NAME);

}



// ============================================================

// BLE SCAN CALLBACK (baca sensor TPMS)

// ============================================================

class MyAdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks {

    void onResult(NimBLEAdvertisedDevice* advertisedDevice) {

        if (!advertisedDevice->isAdvertisingService(NimBLEUUID((uint16_t)0xA827))) return;



        std::string addr = advertisedDevice->getAddress().toString();

        int         rssi = advertisedDevice->getRSSI();



        // AUTO-PAIR mode

        if (apState == AP_SCANNING) {

            if (rssi<AUTOPAIR_RSSI_MIN || rssi>AUTOPAIR_RSSI_MAX) return;

            if (isAlreadyPaired(addr)) return;

            for (auto& c : apCandidates) {

                if (c.mac==addr) { if (rssi>c.rssi) c.rssi=rssi; return; }

            }

            apCandidates.push_back({addr,rssi});

            char buf[60]; snprintf(buf,sizeof(buf),"Cand:%s RSSI:%d",addr.c_str(),rssi);

            Serial.println(buf); if (apToBle) bleSend(String(buf));

            return;

        }



        // LEGACY SCAN mode

        if (scanTpmsMode) {

            if (isAlreadyPaired(addr)) return;

            for (auto& d : scannedTpms) if (d.mac==addr) return;

            scannedTpms.push_back({addr,rssi});

            Serial.printf("[SCAN] %s RSSI:%d\n",addr.c_str(),rssi);

            return;

        }



        // NORMAL mode

        int idx = findTireIndex(addr);

        if (idx==-1) return;



        std::string mfr = advertisedDevice->getManufacturerData();

        if (mfr.length()<6) return;



        uint8_t* d      = (uint8_t*)mfr.data();

        uint16_t stCode = ((uint16_t)d[1]<<8)|d[0];

        float    volt   = d[2]/60.0f;

        int      cel    = (int)d[3]-50;

        uint16_t kpaAbs = ((uint16_t)d[4]<<8)|d[5];



        float kpaG=0,psi=0;

        if (kpaAbs>101) {

            kpaG=kpaAbs-101.3f; psi=kpaG*0.145f;

            if (kpaG<0) kpaG=0; if (psi<0) psi=0;

        }



        tireData[idx].voltage    = volt;

        tireData[idx].celsius    = cel;

        tireData[idx].kpa        = kpaG;

        tireData[idx].psi        = psi;

        tireData[idx].statusStr  = getStatusText(stCode);

        tireData[idx].statusCode = stCode;

        tireData[idx].rssi       = rssi;

        tireData[idx].hasData    = true;

        tireData[idx].lastSeen   = millis();



        TpmsData pkt;

        pkt.timestamp=millis(); pkt.statusCode=stCode;

        pkt.volt_mV=(uint16_t)(volt*1000); pkt.kpa_x10=(uint16_t)(kpaG*10);

        pkt.tireId=(uint8_t)idx; pkt.temp=(int8_t)cel; pkt.bleRssi=(int8_t)rssi;

        uint8_t* pp=(uint8_t*)&pkt; uint8_t cs=0;

        for (int i=0;i<(int)sizeof(TpmsData)-1;i++) cs^=pp[i];

        pkt.chksum=cs;



        if (!txQueuePush(pkt)) { diag.queueDropped++; }



        unsigned long nm=millis();

        Serial.printf("[BLE] %-6s|%s|%.1fkPa|%dC|%.2fV|%d",

                      tireLabels[idx],tireData[idx].statusStr.c_str(),kpaG,cel,volt,rssi);

        if (lastSeenBLE[idx]>0) Serial.printf("|%.1fs\n",(nm-lastSeenBLE[idx])/1000.0f);

        else Serial.println("|first");

        lastSeenBLE[idx]=nm;

    }

};



// ============================================================

// SCAN MAINTENANCE

// ============================================================

unsigned long lastScanRestart = 0;



void maintainScan() {

    if (apState==AP_SCANNING||scanTpmsMode) return;

    unsigned long now=millis();

    if (now-lastScanRestart<SCAN_RESTART_MS) return;

    lastScanRestart=now;

    NimBLEDevice::getScan()->stop();

    NimBLEDevice::getScan()->clearResults();

    NimBLEDevice::getScan()->start(0,nullptr,false);

}



// ============================================================

// SETUP

// ============================================================

void setup() {

    Serial.begin(115200);

    delay(500);

    Serial.printf("\n=== TPMS NODE v4.2 | %s | BLE:\"%s\" ===\n", NODE_ID, BLE_DEV_NAME);



    initRadio();

    loadMacsFromFlash();



    // Init NimBLE — nama device adalah yang muncul di list Bluetooth HP

    NimBLEDevice::init(BLE_DEV_NAME);

    NimBLEDevice::setPower(ESP_PWR_LVL_P9);



    // BLE Server (GATT) — agar HP bisa connect seperti TWS

    initBleServer();



    // BLE Scanner — untuk baca sensor TPMS

    NimBLEScan* pScan = NimBLEDevice::getScan();

    pScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), false);

    pScan->setActiveScan(false);

    pScan->setInterval(625);

    pScan->setWindow(625);

    pScan->setDuplicateFilter(false);

    pScan->start(0, nullptr, false);



    diagStartTime   = millis();

    lastScanRestart = millis();



    bool ada = false;

    for (int i=0;i<TIRE_COUNT;i++) if (!pairedMacs[i].empty()){ada=true;break;}

    if (ada) {

        Serial.println("[INFO] Pairing tersimpan:");

        for (int i=0;i<TIRE_COUNT;i++)

            if (!pairedMacs[i].empty())

                Serial.printf("  [%2d] %-6s : %s\n",i,tireLabels[i],pairedMacs[i].c_str());

    }



    Serial.printf("[OK] Cari \"%s\" di Bluetooth HP Anda.\n", BLE_DEV_NAME);

    Serial.println("[APP] Rekomendasi: \"Serial Bluetooth Terminal\" atau \"nRF Connect\"");

}



// ============================================================

// LOOP

// ============================================================

void loop() {

    // Serial USB (untuk debug)

    if (Serial.available()) {

        String cmd = Serial.readStringUntil('\n');

        handleCommand(cmd, false);

    }



    processRadioQueue();

    processAutoPairResult();

    maintainScan();

    bleFlushTx();   // Kirim antrian response BLE ke HP



    unsigned long now = millis();

    if (now - lastPrintTime >= PRINT_INTERVAL_MS) {

        lastPrintTime = now;

        sendAllTires(false);  // Hanya ke Serial, tidak spam ke BLE

    }



    if (scanTpmsDone) {

        scanTpmsDone = false;

        if (scannedTpms.empty()) {

            outLine("No TPMS found.", bleConnected);

        } else {

            char buf[40];

            snprintf(buf,sizeof(buf),"Found %d sensor:",(int)scannedTpms.size());

            outLine(buf, bleConnected);

            for (auto& d : scannedTpms) {

                snprintf(buf,sizeof(buf),"  %s RSSI:%d",d.mac.c_str(),d.rssi);

                outLine(buf, bleConnected);

            }

            outLine("Use: P-<slot> to pair", bleConnected);

        }

        NimBLEDevice::getScan()->clearResults();

        NimBLEDevice::getScan()->start(0,nullptr,false);

    }

}

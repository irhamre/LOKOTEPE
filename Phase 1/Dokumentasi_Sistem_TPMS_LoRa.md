# Dokumentasi Sistem TPMS Berbasis LoRa
### Gateway v6 (ESP32C3 + SX1276) & Node v4.2 (ESP32U + LilyGo XY LoRa Shield)

---

## 1. Ringkasan Umum

Sistem ini adalah **TPMS (Tire Pressure Monitoring System) jarak jauh** yang terdiri dari dua firmware terpisah yang saling berkomunikasi lewat radio **LoRa P2P (point-to-point)**:

| Komponen | Peran | Hardware |
|---|---|---|
| **TPMS Node v4.2** | Terpasang di kendaraan (truk/trailer). Membaca sensor BLE TPMS di tiap ban, lalu mengirim data via LoRa ke Gateway. | ESP32U DevKit + LilyGo XY LoRa Shield (SX1276) |
| **TPMS Gateway v6** | Stasioner (kantor/pos/server room). Menerima data LoRa dari banyak Node, lalu mempublikasikannya ke broker **MQTT** sebagai JSON. | ESP32C3 Mini + SX1276 custom wiring |

Alasan dipisah jadi 2 link radio (BLE untuk sensor↔Node, LoRa untuk Node↔Gateway) adalah karena BLE sensor TPMS jangkauannya pendek (di sekitar kendaraan), sedangkan LoRa dipakai untuk jarak jauh antara kendaraan dan pos penerima/gateway yang terhubung internet.

Topologi:

```
[Sensor BLE TPMS x16] --BLE--> [TPMS NODE, di kendaraan] --LoRa P2P--> [TPMS GATEWAY, stasioner] --WiFi/MQTT--> [Broker MQTT / Dashboard]
                                       ^                                         |
                                  (HP via BLE NUS,                    (LittleFS store & forward
                                   app "Serial Bluetooth                bila WiFi/MQTT putus)
                                   Terminal"/"nRF Connect")
```

Satu Gateway dapat menangani hingga **10 Node** sekaligus (`MAX_NODES = 10`), dan setiap Node dapat menangani hingga **16 ban/sensor** (`TIRE_COUNT / MAX_TIRES = 16`).

---

## 2. Protokol Komunikasi LoRa (Wajib Sama di Kedua Sisi)

Kedua firmware menggunakan struct biner yang **harus identik byte-per-byte** (pakai `__attribute__((packed))`), karena dikirim mentah lewat `LoRa.write()`/`LoRa.read()`.

### 2.1. Header Paket

```c
#define PKT_MAGIC  0xA55A   // angka penanda paket valid
#define PKT_VER    0x04     // versi protokol

struct PktHeader {
    uint16_t magic;      // harus == 0xA55A
    uint8_t  ver;        // versi protokol
    char     nodeId[7];  // ID Node, contoh "TRL001"
    uint8_t  tireCount;  // jumlah ban yang dimiliki Node (umumnya 16)
};
```

### 2.2. Data Per-Ban

```c
struct TpmsData {
    uint32_t timestamp;   // millis() saat dibaca dari sensor BLE
    uint16_t statusCode;  // kode status mentah dari sensor TPMS
    uint16_t volt_mV;     // tegangan baterai sensor (mV)
    uint16_t kpa_x10;     // tekanan ban dalam kPa x10 (gauge, sudah dikurangi 1 atm)
    uint8_t  tireId;      // index ban 0..15
    int8_t   temp;        // suhu ban (°C)
    int8_t   bleRssi;     // RSSI BLE saat sensor terbaca oleh Node
    uint8_t  chksum;      // XOR checksum seluruh byte sebelum field ini
};
```

### 2.3. Paket Lengkap

```c
struct LoRaPacket {
    PktHeader hdr;
    TpmsData  data;
};
```

**Catatan penting:**
- Setiap paket LoRa hanya membawa data **1 ban saja** (bukan 16 ban sekaligus) — ini menjaga ukuran paket tetap kecil agar airtime LoRa singkat.
- Checksum dihitung dengan XOR semua byte `TpmsData` kecuali byte checksum-nya sendiri.
- Parameter radio (frekuensi, Spreading Factor, Bandwidth, Coding Rate, Sync Word) **harus identik** antara Node dan Gateway:

| Parameter | Nilai |
|---|---|
| Frekuensi | `922E6` Hz (komentar kode menyebut bisa diganti `868E6` sesuai regulasi region) |
| Spreading Factor | 7 |
| Bandwidth | 125 kHz |
| Coding Rate | 4/5 |
| Sync Word | `0x12` |
| CRC | Aktif (`enableCrc()`) |

---

## 3. PROGRAM 1 — TPMS GATEWAY v6

### 3.1. Daftar Fitur Utama

1. **Penerima LoRa multi-node** — bisa menerima dari banyak Node sekaligus, masing-masing diidentifikasi via `nodeId`.
2. **Sistem Pair/Unpair per Node** — Node yang belum di-*pair* datanya diterima tapi **diabaikan** (tidak dipublikasikan), mencegah data sampah/Node asing masuk ke MQTT.
3. **Mode Scan** — mendeteksi Node yang aktif di sekitar selama 30 detik tanpa langsung mem-pair-nya, supaya operator bisa memilih mana yang valid sebelum `pair`.
4. **Cache per-ban dengan deteksi “stale”** — setiap ban punya cache data terakhir; jika tidak ada update >3 menit (`DATA_STALE_MS = 180000`), statusnya otomatis berubah jadi `NO_PACKET` walau Node masih terdeteksi.
5. **Publish ke MQTT dalam bentuk bundel JSON per Node** (1 JSON berisi status 16 ban), dengan retained flag aktif.
6. **Publish dipicu 2 cara:**
   - **Event-driven**: setiap kali ada paket baru masuk dari Node yang sudah di-pair.
   - **Periodik**: setiap 60 detik (`PERIODIC_PUBLISH_MS`) untuk semua Node yang di-pair, walau tidak ada paket baru — supaya status `NO_PACKET` tetap terkirim ke dashboard.
7. **WiFi & broker MQTT TIDAK hardcode** — menggunakan **WiFiManager** (captive portal) sehingga kredensial WiFi dan alamat broker MQTT diisi lewat browser HP/laptop, tanpa reflash firmware.
8. **OTA (Over-The-Air) firmware update** — update firmware lewat jaringan (ArduinoOTA), tanpa kabel USB, setelah gateway online.
9. **Store & Forward via LittleFS** — bila MQTT/WiFi mati saat publish, data disimpan ke flash internal (`/queue.txt`) dan otomatis dikirim ulang saat koneksi kembali, sehingga **data tidak hilang**.
10. **Diagnostik radio & sistem lengkap** lewat serial: jumlah RX, error checksum, error magic, RSSI/SNR, status MQTT, status FS, dsb.
11. **Antarmuka kontrol penuh via Serial Monitor** (lihat tabel perintah di bawah).
12. **Penyimpanan konfigurasi persisten** di NVS (`Preferences`): daftar Node yang dipair, host/port broker MQTT.

### 3.2. Hardware & Pinout

| Item | Spesifikasi |
|---|---|
| MCU | ESP32C3 Mini |
| Modul Radio | SX1276 (LoRa), wiring custom |
| SCK | GPIO 4 |
| MISO | GPIO 5 |
| MOSI | GPIO 6 |
| CS (NSS) | GPIO 7 |
| RST | GPIO 2 |
| DIO0 | GPIO 3 |
| Frekuensi | 922 MHz (ganti ke 868 MHz jika region berbeda) |

### 3.3. Library yang Dibutuhkan

| Library | Sumber | Fungsi |
|---|---|---|
| `arduino-LoRa` | sandeepmistry | Driver radio SX1276 |
| `ArduinoJson` | v7 | Serialisasi payload MQTT |
| `PubSubClient` | — | Klien MQTT |
| `WiFiManager` | tzapu | Captive portal setup WiFi & parameter custom |
| `ArduinoOTA`, `Preferences`, `LittleFS` | bundled ESP32 core | OTA, penyimpanan NVS, filesystem flash |

### 3.4. WiFi & Konfigurasi MQTT via Captive Portal

- Saat pertama nyala (atau setelah `wifi_reset`), Gateway membuka Access Point **`TPMS-Gateway-Setup`** (password default `tpms12345`).
- Setelah konek ke AP itu, browser otomatis menampilkan halaman konfigurasi (captive portal) berisi pilihan WiFi rumah/kantor + password, **ditambah 2 field kustom**:
  - **MQTT Broker Host/IP**
  - **MQTT Broker Port** (default `1883`)
- Semua kredensial disimpan otomatis (WiFi oleh WiFiManager, broker oleh `Preferences` namespace `gwcfg`), dipakai ulang di boot berikutnya.
- **Timeout portal: 180 detik** — jika tidak ada yang melakukan setup, Gateway lanjut berjalan **offline** (data tetap masuk antrian LittleFS).
- ⚠️ **Selama portal terbuka, Gateway bersifat blocking** — tidak memproses paket LoRa/serial sampai portal ditutup (perilaku normal WiFiManager).
- Broker default sebelum dikonfigurasi: `public-mqtt-broker.bevywise.com:1883` (broker publik untuk uji coba — **harus diganti** untuk produksi).

### 3.5. OTA Firmware Update

- Memakai `ArduinoOTA`, aktif otomatis setelah WiFi tersambung.
- Hostname: **`tpms-gateway`** → muncul di Arduino IDE sebagai *Network Port* `tpms-gateway at x.x.x.x`, port `3232`.
- Password OTA default: `tpms12345` (⚠️ **ganti sebelum dipakai di lapangan**).
- Upload firmware baru cukup lewat port jaringan tersebut, tanpa kabel USB.

### 3.6. Store & Forward (LittleFS)

Mekanisme penyangga data agar **tidak ada data yang hilang** saat WiFi/MQTT putus:

| File | Fungsi |
|---|---|
| `/queue.txt` | Antrian pesan yang gagal terkirim. Format per baris: `topic<TAB>payload` |
| `/temp.txt` | Buffer sementara saat proses forward berlangsung |

Alur kerja:
1. `publishNodeBundle()` mencoba publish ke MQTT.
2. Jika WiFi/MQTT tidak terhubung, atau `publish()` gagal (misal payload kebesaran untuk buffer) → data ditulis ke `queue.txt` lewat `saveToFS()`.
3. Setiap **5 detik** (`FS_PROCESS_INTERVAL_MS`), selama MQTT terkoneksi, `processStoredData()` dipanggil:
   - Membaca `queue.txt` baris per baris.
   - Mencoba publish ulang tiap baris (retained=true), beri delay 50ms antar publish agar broker tidak overload.
   - Baris yang **sukses** dihapus dari antrian.
   - Baris yang **gagal** ditulis ulang ke `temp.txt`, lalu menjadi `queue.txt` baru untuk percobaan berikutnya.
4. Statistik tercatat di `GwDiag`: `fsSaved`, `fsForwarded`, `fsSaveFail`.

Perintah serial terkait: `queue` (lihat info antrian), `clearqueue` (hapus manual).

### 3.7. Manajemen Node (Pairing)

- Setiap Node terdeteksi otomatis ditambahkan ke tabel `nodes[]` (maks 10) berdasarkan `nodeId` di header paket.
- Node **baru tidak otomatis dipair** — datanya tetap diterima (untuk update `lastSeen`/RSSI) tapi **diabaikan untuk publish** sampai di-`pair` manual.
- Status pairing disimpan persisten di NVS (`Preferences` namespace `gwpair`), jadi tetap tersimpan setelah restart/listrik mati.
- Mode `scan` (30 detik) membantu operator melihat Node mana saja yang aktif di sekitar sebelum memutuskan mem-pair.
- Bisa juga pair Node yang belum pernah terdeteksi secara manual lewat `pair <ID>` (Node akan dibuat entry-nya, ditandai paired, menunggu data masuk).

### 3.8. Struktur Topic & Payload MQTT

- **Topic status gateway**: `tpms/gateway/status` → dikirim sekali saat MQTT connect: `{"status":"online","hw":"ESP32C3_SX1276"}`
- **Topic per Node**: `tpms/<nodeId>` (contoh: `tpms/TRL001`), berisi **1 JSON bundel berisi seluruh 16 ban** Node tersebut.

Contoh payload `tpms/TRL001`:

```json
{
  "node": "TRL001",
  "paired": true,
  "rx_count": 482,
  "rssi_rf": -67.5,
  "node_age_s": 4,
  "uptime_s": 18230,
  "ts": 18230412,
  "tires": [
    {
      "id": 0,
      "status": "OK",
      "kpa": 320.5,
      "temp_c": 34,
      "volt_v": 3.05,
      "rssi_ble": -42,
      "status_code": 2,
      "age_s": 7
    },
    {
      "id": 1,
      "status": "NO_PACKET"
    }
  ]
}
```

Field kunci:
- `status: "OK"` → data masih segar (< 3 menit).
- `status: "NO_PACKET"` → tidak ada data sama sekali, atau data terakhir sudah lebih dari 3 menit (stale).
- `rssi_rf` → kekuatan sinyal LoRa Node↔Gateway (bukan BLE sensor↔Node).
- `rssi_ble` (di level tire) → kekuatan sinyal BLE sensor↔Node, dikirim dari Node.

⚠️ **Catatan teknis penting**: buffer `PubSubClient` diperbesar manual ke **2560 byte** (`mqttClient.setBufferSize(2560)`) karena payload JSON 16 ban bisa mencapai ±2120 byte, jauh di atas default PubSubClient (256 byte). Jika buffer ini diturunkan/dilupakan saat modifikasi kode, publish akan **selalu gagal silently** untuk Node dengan banyak ban aktif.

### 3.9. Diagnostik (struct `GwDiag`)

| Counter | Arti |
|---|---|
| `rxTotal` | Total paket LoRa diterima (apapun hasilnya) |
| `rxOk` | Paket valid (magic + checksum benar) |
| `rxCsErr` | Paket gagal checksum |
| `rxMagicErr` | Paket dengan magic number salah |
| `rxIgnored` | Paket valid tapi dari Node yang belum di-pair |
| `rssiLast` / `rssiSum` / `snrLast` | Kualitas sinyal radio terakhir/rata-rata |
| `mqttSent` / `mqttFail` | Statistik publish MQTT |
| `fsSaved` / `fsForwarded` / `fsSaveFail` | Statistik antrian LittleFS |

### 3.10. Daftar Perintah Serial — Gateway

| Perintah | Fungsi |
|---|---|
| `help` | Tampilkan daftar perintah |
| `scan` | Scan Node aktif selama 30 detik |
| `pair <ID>` | Pair Node, contoh: `pair TRL001` |
| `unpair <ID>` | Unpair Node tertentu |
| `unpair_all` | Unpair semua Node |
| `list` | Tampilkan semua Node yang pernah terdeteksi + status pairing |
| `status` | Tampilkan data tekanan/suhu/volt per ban untuk semua Node yang dipair |
| `json` | Dump seluruh data Node+ban dalam format JSON ke serial (debug) |
| `diag` | Tampilkan diagnostik lengkap (radio, MQTT, WiFi, OTA, FS) |
| `diag_reset` | Reset semua counter diagnostik |
| `mqtt_status` | Cek status koneksi MQTT + daftar topic aktif |
| `queue` | Info isi antrian LittleFS (`queue.txt`) |
| `clearqueue` | Hapus antrian LittleFS secara manual |
| `show_config` | Tampilkan WiFi SSID/IP, broker MQTT, hostname OTA |
| `set_broker <host> [port]` | Ganti broker MQTT cepat lewat serial (auto restart) |
| `wifi_config` | Buka ulang portal konfigurasi WiFi/MQTT (akan restart) |
| `wifi_reset` | Hapus kredensial WiFi tersimpan (akan restart ke mode AP setup) |

### 3.11. Konstanta Penting yang Bisa Disesuaikan

| Konstanta | Default | Keterangan |
|---|---|---|
| `MAX_NODES` | 10 | Maks jumlah Node yang ditangani |
| `MAX_TIRES` | 16 | Maks ban per Node |
| `DATA_STALE_MS` | 180000 (3 menit) | Ambang waktu data dianggap basi → `NO_PACKET` |
| `SCAN_DURATION_MS` | 30000 | Durasi mode scan |
| `PERIODIC_PUBLISH_MS` | 60000 | Interval publish ulang otomatis semua Node terpasang |
| `FS_PROCESS_INTERVAL_MS` | 5000 | Interval coba forward antrian LittleFS |
| `MQTT_PING_MS` | 10000 | Interval cek/reconnect MQTT |
| `WIFI_RETRY_MS` | 15000 | Interval retry WiFi reconnect saat putus |
| `AP_PASSWORD`, `OTA_PASSWORD` | `tpms12345` | ⚠️ **Wajib diganti** sebelum produksi |

---

## 4. PROGRAM 2 — TPMS NODE v4.2

### 4.1. Daftar Fitur Utama

1. **Pembaca 16 sensor BLE TPMS** sekaligus, dipasang per posisi ban kendaraan multi-sumbu (truk/trailer).
2. **Pengirim LoRa P2P** ke Gateway, satu paket per pembacaan sensor.
3. **Server BLE GATT (Nordic UART Service)** — Node bisa diakses langsung dari HP seperti aksesoris Bluetooth biasa (terlihat di list Bluetooth HP, mis. mirip TWS/smartwatch), dikendalikan via app terminal BLE (`Serial Bluetooth Terminal`, `nRF Connect`).
4. **Auto-Pair sensor berbasis RSSI** — operator mendekatkan sensor ke Node, kirim perintah `P-<slot>`, Node otomatis memilih sensor terdekat (RSSI dalam rentang tertentu) untuk dipasangkan ke slot itu — tidak perlu mengetik MAC address manual.
5. **Pairing manual** juga didukung (`p-<posisi> <MAC>`), untuk kasus auto-pair gagal/ambigu.
6. **Mode scan legacy** (`scan_tpms`, 120 detik) untuk melihat semua sensor TPMS terdekat + MAC + RSSI tanpa langsung mem-pair.
7. **Decoding data mentah sensor BLE TPMS** dari *manufacturer data* advertisement (tegangan, suhu, tekanan absolut → dikonversi ke gauge kPa & PSI, status kode).
8. **Antrian TX LoRa (circular buffer)** dengan deduplikasi per `tireId` — kalau ada update baru sebelum data lama terkirim, data lama digantikan (mencegah antrian penuh dengan data basi).
9. **Diagnostik TX radio** lengkap per ban (`diag`).
10. **Penyimpanan pairing persisten** di NVS (`Preferences` namespace `tpms`), tahan restart.
11. **Auto-restart scan BLE tiap 2 menit** agar proses scanning BLE tidak macet/stale dalam waktu lama.

### 4.2. Hardware & Pinout

| Item | Spesifikasi |
|---|---|
| MCU | ESP32U DevKit |
| Shield Radio | LilyGo XY LoRa Shield (SX1276) |
| SCK | GPIO 18 |
| MISO | GPIO 19 |
| MOSI | GPIO 23 |
| SS (CS) | GPIO 5 |
| RST | GPIO 14 |
| DIO0 | GPIO 26 |
| Frekuensi | 922 MHz |
| TX Power | 20 (maks, lihat `LoRa.setTxPower(20)`) |

### 4.3. Library yang Dibutuhkan

| Library | Fungsi |
|---|---|
| `arduino-LoRa` (sandeepmistry) | Driver radio SX1276 |
| `NimBLE-Arduino` | Stack BLE ringan — dipakai untuk **server** (komunikasi ke HP) sekaligus **scanner** (membaca sensor TPMS) |
| `Preferences` | Penyimpanan pairing MAC sensor ke NVS |

### 4.4. Identitas Node & Penamaan

- `NODE_ID` (contoh `"TRL001"`) — **wajib diganti unik per unit**, digunakan sebagai identifier di paket LoRa dan dasar topic MQTT di Gateway (`tpms/TRL001`).
- `BLE_DEV_NAME` (contoh `"TPMS_TRL001"`) — nama yang muncul di daftar Bluetooth HP saat mencari device.

### 4.5. BLE GATT Server — Akses Langsung dari HP

Node mengimplementasikan **Nordic UART Service (NUS)** standar, sehingga aplikasi terminal BLE umum bisa langsung connect:

| UUID | Peran |
|---|---|
| `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` | Service utama |
| `6E400002-...` | Karakteristik **RX** (HP → Node), properti `WRITE` |
| `6E400003-...` | Karakteristik **TX** (Node → HP), properti `NOTIFY` |

- Saat HP connect, Node mengirim pesan sambutan: `TPMS <NODE_ID> v4.2 OK. Type: help`.
- Saat HP disconnect, Node otomatis kembali advertising agar bisa disambungkan lagi.
- Pengiriman balik ke HP memakai antrian string (`bleTxBuf`, 16 slot) yang di-flush per loop, dipecah jadi chunk 20 byte (sesuai MTU BLE minimum) dengan delay 10ms antar chunk — supaya tidak ada notify yang terpotong.
- Perintah serial USB **tidak** direlay ke BLE (`toBle=false`), sedangkan perintah dari BLE selalu dibalas ke BLE juga.

### 4.6. Pembacaan & Decoding Sensor TPMS (BLE Scan)

- Node melakukan **passive BLE scan kontinu** (interval=window=625, tanpa filter duplikat dimatikan agar update terus diterima).
- Filter: hanya advertisement yang membawa **service UUID `0xA827`** yang diproses (UUID standar umum dipakai sensor TPMS BLE komersial).
- Jika MAC address yang terdeteksi cocok dengan salah satu slot yang sudah dipair, *manufacturer data* (minimal 6 byte) didecode:

| Byte | Arti |
|---|---|
| `d[0..1]` | `statusCode` (little endian: `d[1]<<8 | d[0]`) |
| `d[2]` | Tegangan baterai → `volt = d[2] / 60.0` (Volt) |
| `d[3]` | Suhu → `celsius = d[3] - 50` (°C, offset 50) |
| `d[4..5]` | Tekanan absolut (big endian: `d[4]<<8 | d[5]`), dalam kPa |

- Tekanan **absolut** dikonversi ke **gauge** (relatif terhadap atmosfer) dengan mengurangi 101.3 kPa, lalu dikonversi juga ke PSI (`psi = kpaGauge * 0.145`). Nilai negatif diklem ke 0.
- Kode status sensor diterjemahkan via `getStatusText()`:

| Kode | Arti |
|---|---|
| `0x0002` | STABLE_FAST |
| `0x0003` | STABLE_SLOW |
| `0x0004` | DOWN (tekanan turun) |
| `0x0005` | UP (tekanan naik) |
| `0x0006` | TRANSITION |
| `0x0008` | IDLE |
| lainnya | `UNK_XXXX` (kode tidak dikenal, ditampilkan hex) |

- Setelah decode, data langsung dipush ke **TX Queue** untuk dikirim via LoRa ke Gateway.

### 4.7. TX Queue LoRa

- Circular buffer ukuran **64 slot** (`TX_QUEUE_SIZE`).
- `txQueuePush()`: jika sudah ada antrian dengan `tireId` yang sama → **ditimpa** (update in-place) supaya antrian tidak menumpuk data basi untuk ban yang sama; jika belum ada → ditambahkan baru.
- `processRadioQueue()` mengirim **maksimal 1 paket setiap 300ms** (`TX_INTERVAL_MS`) — membatasi duty cycle radio dan menghindari airtime berlebihan.
- Jika antrian penuh saat push gagal → dicatat sebagai `diag.queueDropped`.

### 4.8. Mekanisme Auto-Pair

Tujuan: memasangkan sensor BLE TPMS ke slot ban tertentu **tanpa mengetik MAC manual**, cukup mendekatkan sensor lalu kirim perintah.

Langkah:
1. Operator mengirim `P-<slot>` (contoh `P-0` untuk slot FL) lewat BLE/Serial.
2. Validasi: slot harus 0–15, belum ada proses auto-pair lain berjalan, slot belum terisi.
3. Node memulai BLE scan **10 detik** (`AUTOPAIR_SCAN_SEC`), hanya menerima sensor dengan RSSI dalam rentang **-50 dBm s/d -30 dBm** (`AUTOPAIR_RSSI_MIN/MAX`) — cukup dekat untuk dianggap sensor yang sedang dipasang, tapi tidak terlalu dekat hingga sinyal saturasi.
4. Semua kandidat (MAC unik, belum dipair di slot lain) dikumpulkan; jika MAC sama terdeteksi berulang, RSSI tertinggi yang disimpan.
5. Setelah scan selesai, kandidat dengan RSSI **terbaik (terdekat)** dipilih sebagai pasangan slot tersebut, disimpan ke flash.
6. Hasil dikirim dalam 2 format:
   - Baris manusiawi: `OK Slot0(FL)=aa:bb:cc:dd:ee:ff RSSI:-35dBm`
   - Baris **parseable untuk aplikasi companion**: `AP_RESULT:0,FL,aa:bb:cc:dd:ee:ff,-35`
7. Jika tidak ada kandidat ditemukan dalam rentang RSSI → `FAIL no sensor ...`.

### 4.9. Pairing Manual & Manajemen Slot

| Perintah | Fungsi |
|---|---|
| `p-<pos> <MAC>` | Pairing manual slot dengan MAC tertentu (format `xx:xx:xx:xx:xx:xx`, 17 karakter) |
| `unpair-<pos>` | Hapus pairing 1 slot |
| `unpair_all` | Hapus semua pairing |
| `status` | Tampilkan semua 16 slot beserta MAC terpasang (atau `EMPTY`) |

Validasi yang dilakukan saat pairing manual: slot harus kosong, MAC belum dipakai di slot lain, format MAC harus tepat 17 karakter.

### 4.10. Mapping Slot/Posisi Ban

Sistem ini didesain untuk **kendaraan besar bersumbu ganda** (truk/trailer), tiap sumbu memiliki ban kembar (dual tire) kiri & kanan:

| Slot | Label | Perintah |
|---|---|---|
| 0 | FL (Front Left) | `fl` |
| 1 | FL-2 (Front Left dual) | `fl-2` |
| 2 | FR (Front Right) | `fr` |
| 3 | FR-2 (Front Right dual) | `fr-2` |
| 4 | RL (Rear Left) | `rl` |
| 5 | RL-2 (Rear Left dual) | `rl-2` |
| 6 | RR (Rear Right) | `rr` |
| 7 | RR-2 (Rear Right dual) | `rr-2` |
| 8 | ML (Middle Left) | `ml` |
| 9 | ML-2 (Middle Left dual) | `ml-2` |
| 10 | MR (Middle Right) | `mr` |
| 11 | MR-2 (Middle Right dual) | `mr-2` |
| 12 | R2L (Rear-2 Left, sumbu belakang ke-2) | `r2l` |
| 13 | R2L-2 (dual) | `r2l-2` |
| 14 | R2R (Rear-2 Right, sumbu belakang ke-2) | `r2r` |
| 15 | R2R-2 (dual) | `r2r-2` |

→ Total mengakomodasi **4 sumbu** (Front, Rear, Middle, Rear-2) × 2 sisi × 2 ban kembar = 16 ban. Cocok untuk truk besar/trailer dengan banyak sumbu.

### 4.11. Diagnostik (struct `RadioDiag`)

| Counter | Arti |
|---|---|
| `txCount` | Total percobaan kirim LoRa |
| `txOk` | Sukses kirim (`LoRa.endPacket() == 1`) |
| `txErr` | Gagal kirim |
| `txPerTire[16]` | Jumlah kirim per ban (untuk melihat ban mana paling aktif/jarang update) |
| `queueDropped` | Data yang hilang karena antrian TX penuh |

### 4.12. Daftar Perintah — Node (Serial USB & BLE)

| Perintah | Fungsi |
|---|---|
| `help` | Daftar perintah + mapping slot |
| `P-<0..15>` | Mulai auto-pair untuk slot tersebut (contoh `P-0`, `P-7`) |
| `p-<pos> <MAC>` | Pairing manual (contoh `p-fl aa:bb:cc:dd:ee:ff`) |
| `status` | Daftar 16 slot + MAC terpasang |
| `report` | Data tekanan/suhu/volt semua ban yang sudah ada data |
| `unpair-<pos>` | Hapus pairing satu posisi (contoh `unpair-fl`) |
| `unpair_all` | Hapus semua pairing |
| `diag` | Statistik TX radio per ban |
| `diag_reset` | Reset statistik diagnostik |
| `scan_tpms` | Scan legacy 120 detik, menampilkan semua sensor TPMS terdeteksi (MAC+RSSI) tanpa pairing otomatis |

### 4.13. Konstanta Penting yang Bisa Disesuaikan

| Konstanta | Default | Keterangan |
|---|---|---|
| `TIRE_COUNT` | 16 | Jumlah slot ban |
| `TX_QUEUE_SIZE` | 64 | Ukuran antrian TX LoRa |
| `TX_INTERVAL_MS` | 300 | Jeda minimum antar pengiriman LoRa |
| `SCAN_RESTART_MS` | 120000 (2 menit) | Interval restart BLE scan kontinu |
| `PRINT_INTERVAL_MS` | 10000 | Interval cetak laporan ke Serial (debug, tidak ke BLE) |
| `AUTOPAIR_SCAN_SEC` | 10 | Durasi scan saat auto-pair |
| `AUTOPAIR_RSSI_MIN` / `MAX` | -50 / -30 dBm | Rentang RSSI yang diterima saat auto-pair |

---

## 5. Alur Kerja End-to-End

1. **Sensor TPMS** di tiap ban memancarkan BLE advertisement berisi data tekanan/suhu/voltase secara periodik (manufacturer data, service UUID `0xA827`).
2. **Node** terus melakukan BLE passive scan; ketika menerima advertisement dari MAC yang sudah terpasang di salah satu slot, data didecode dan disimpan ke cache lokal (`tireData[]`).
3. Data tersebut langsung dikemas jadi `TpmsData`, diberi checksum, dan **dimasukkan ke TX queue**.
4. Setiap 300ms, Node mengirim **1 paket LoRa** (1 ban) dari antrian ke Gateway.
5. **Gateway** menerima paket, validasi magic number & checksum, lalu cek apakah `nodeId` pengirim sudah **di-pair**.
   - Jika belum di-pair → diabaikan (hanya update RSSI/`lastSeen`).
   - Jika sudah di-pair → data ban tersebut disimpan ke cache Gateway, lalu **langsung dipublikasikan** sebagai JSON bundel 16-ban ke topic `tpms/<nodeId>`.
6. Jika MQTT/WiFi sedang putus saat publish → data disimpan ke **LittleFS** (`queue.txt`) dan dikirim ulang otomatis begitu koneksi pulih.
7. Setiap 60 detik, Gateway **mempublikasikan ulang** semua Node yang dipair walau tidak ada paket baru — supaya ban yang sudah lama tidak update otomatis berstatus `NO_PACKET` di dashboard (bukan diam tak terdeteksi).
8. Dashboard/aplikasi yang subscribe ke topic `tpms/#` menerima update kondisi seluruh ban kendaraan secara near-real-time.

Diagram alur singkat:

```
Sensor BLE  --advertise-->  Node (scan, decode, checksum)
                                   |
                            TX Queue (300ms/paket)
                                   |
                                LoRa TX  ---udara---> LoRa RX
                                                          |
                                                Gateway: validasi paket
                                                          |
                                          Sudah dipair? ---tidak---> diabaikan
                                                   |ya
                                          Update cache ban + publish JSON
                                                   |
                                     MQTT terhubung? ---tidak---> simpan LittleFS
                                                   |ya
                                          Publish ke topic tpms/<nodeId>
```

---

## 6. Mekanisme Ketahanan (Resilience)

| Risiko | Mitigasi pada sistem |
|---|---|
| WiFi Gateway putus | Auto-reconnect tiap 15 detik memakai kredensial tersimpan (tidak membuka portal otomatis agar tidak mengganggu operasi) |
| MQTT broker tidak terjangkau | Auto-reconnect tiap 10 detik; selama putus, semua data baru disimpan ke LittleFS |
| Publish gagal (buffer/ukuran payload) | Data otomatis dialihkan ke antrian LittleFS, dicoba ulang tiap 5 detik setelah MQTT terkoneksi kembali |
| Ban berhenti mengirim data | Terdeteksi via `DATA_STALE_MS` (3 menit) → status berubah ke `NO_PACKET`, tetap terkirim ke MQTT lewat publish periodik 60 detik |
| Antrian TX Node penuh | Deduplikasi per `tireId` (update in-place) mengurangi risiko penuh; jika tetap penuh, tercatat di `queueDropped` |
| Restart/listrik mati | Daftar Node yang dipair (Gateway) dan MAC sensor yang dipair (Node) tersimpan di NVS (`Preferences`), otomatis dimuat ulang saat boot |
| Operator ingin ganti WiFi/broker tanpa reflash | `wifi_config` (portal) atau `set_broker` (cepat lewat serial) di Gateway |
| Perlu update firmware di lapangan | OTA Gateway via `ArduinoOTA` (tanpa USB); Node masih perlu USB karena tidak diimplementasikan OTA |

---

## 7. Keamanan & Catatan Penting

⚠️ **Password default harus diganti sebelum dipakai di lapangan:**
- `AP_PASSWORD = "tpms12345"` (password masuk AP setup WiFi Gateway)
- `OTA_PASSWORD = "tpms12345"` (password upload firmware OTA)

⚠️ **Broker MQTT default** (`public-mqtt-broker.bevywise.com:1883`) adalah broker publik tanpa autentikasi/enkripsi — **hanya cocok untuk uji coba**, bukan untuk data produksi. Gunakan broker privat (idealnya dengan TLS + username/password) untuk penggunaan nyata.

⚠️ **MQTT client tidak menggunakan autentikasi** — `mqttClient.connect(MQTT_CLIENT_ID)` dipanggil tanpa username/password. Jika broker produksi memerlukan kredensial, kode perlu dimodifikasi untuk menambahkannya.

⚠️ **BLE Node tanpa pairing/bonding/enkripsi** — siapa pun yang tahu nama device (`TPMS_TRL001`) bisa connect ke Node lewat app BLE generik dan mengirim perintah (termasuk unpair). Tidak ada otentikasi di level GATT.

⚠️ **Selama config portal WiFi Gateway terbuka, sistem berhenti memproses data** (blocking). Sebaiknya hanya dilakukan saat kendaraan/Node tidak sedang beroperasi aktif, atau pastikan Node tetap menyimpan data sementara (Node sendiri tidak punya store & forward, hanya antrian TX 64 slot dengan dedup — data lama yang belum terkirim & tertimpa oleh data baru akan hilang).

ℹ️ **`NODE_ID` dan `BLE_DEV_NAME` wajib unik** per kendaraan/unit Node — jika dua Node memakai ID yang sama, Gateway akan menganggapnya sebagai 1 Node (data akan tercampur/saling menimpa di cache).

---

## 8. Troubleshooting

| Gejala | Kemungkinan Sebab | Solusi |
|---|---|---|
| Gateway tidak menerima paket sama sekali | Parameter radio tidak sama (frekuensi/SF/BW/CR/Sync word), atau wiring SX1276 salah | Cek `initRadio()` di kedua sisi, pastikan identik; cek pinout sesuai dokumentasi di atas |
| Node terdeteksi di `scan` tapi data tidak pernah update | Node belum di-`pair` di Gateway | Jalankan `pair <ID>` di serial Gateway |
| Status ban selalu `NO_PACKET` walau Node aktif | Sensor BLE belum terpasang ke slot tersebut di Node, atau MAC sensor salah | Cek `status` di Node, lakukan `P-<slot>` ulang mendekatkan sensor |
| Publish MQTT selalu gagal untuk Node tertentu | Buffer PubSubClient tidak sesuai (jika kode dimodifikasi dan `setBufferSize` dihapus/diturunkan) | Pastikan `mqttClient.setBufferSize(2560)` tetap ada di `setup()` |
| Auto-pair selalu `FAIL no sensor` | Sensor terlalu jauh/terlalu dekat dari rentang RSSI `-50..-30 dBm`, atau service UUID sensor bukan `0xA827` | Dekatkan sensor sesuai jarak wajar; jika sensor berbeda merk, cek UUID advertisement-nya |
| Data tertahan, tidak sampai ke dashboard saat WiFi mati lalu nyala lagi | Normal — proses forward LittleFS butuh waktu (tiap 5 detik, delay 50ms/pesan) | Tunggu beberapa saat; cek progres lewat perintah `queue` di Gateway |
| Tidak bisa OTA Gateway | WiFi belum konek, atau salah password OTA | Pastikan `show_config` menunjukkan IP valid; cek `OTA_PASSWORD` |

---

## 9. Cheatsheet Perintah Cepat

**Gateway (Serial Monitor, 115200 baud):**
```
scan                      # cari node di sekitar
pair TRL001                # pair node yang ditemukan
list                       # lihat semua node + status pairing
status                     # lihat data ban semua node terpasang
diag                       # diagnostik lengkap
set_broker 192.168.1.50 1883
wifi_config                # buka ulang portal setup WiFi/MQTT
```

**Node (Serial Monitor / App BLE Terminal):**
```
scan_tpms                  # cari semua sensor TPMS di sekitar (120s)
P-0                        # auto-pair slot FL (dekatkan sensor dulu)
P-4                        # auto-pair slot RL
status                     # lihat 16 slot & MAC terpasang
report                     # lihat data tekanan/suhu semua ban
unpair-fl                  # hapus pairing slot FL
diag                       # statistik pengiriman radio
```

---

## 10. Saran Pengembangan Lebih Lanjut

- **Enkripsi/otentikasi LoRa**: payload saat ini polos (plaintext), rawan spoofing jika ada pihak ketiga mengirim paket dengan `magic`+checksum yang valid. Bisa ditambah HMAC atau pre-shared key sederhana.
- **TLS untuk MQTT** dan autentikasi username/password broker.
- **OTA untuk Node** (saat ini hanya Gateway yang punya OTA; Node masih perlu USB untuk update).
- **Store & forward di sisi Node**: saat ini Node hanya punya antrian RAM 64 slot tanpa persistensi; jika LoRa link putus lama, data lama yang ditimpa akan hilang permanen (berbeda dengan Gateway yang punya LittleFS).
- **ACK/retry di link LoRa** Node→Gateway untuk memastikan pengiriman lebih reliable (saat ini bersifat *fire-and-forget*, tidak ada acknowledgement).
- **Multi-gateway / failover** untuk area dengan jangkauan LoRa terbatas.
- **Konfirmasi tertulis dari operator** sebelum auto-pair menimpa slot (saat ini sudah ada validasi slot kosong, jadi cukup aman, namun bisa ditambah konfirmasi 2 langkah).

---

*Dokumentasi ini disusun berdasarkan analisis langsung terhadap source code firmware "TPMS Gateway v6" dan "TPMS Node v4.2".*

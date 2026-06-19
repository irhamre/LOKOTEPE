# Lokotepe — Phase 1

**Sistem TPMS (Tire Pressure Monitoring System) nirkabel jarak jauh berbasis LoRa**, terdiri dari dua firmware ESP32 yang saling berkomunikasi:

| Firmware | Lokasi | Peran |
|---|---|---|
| **Gateway V1** | terpasang di kendaraan | Membaca 16 sensor BLE TPMS per ban, mengirim data via LoRa |
| **Edge V1** | stasioner (pos/kantor/server room) | Menerima data LoRa dari banyak Gateway, publish ke MQTT |

> ℹ️ **Catatan penamaan**: pada source code, perangkat di kendaraan disebut **`Gateway`** dan perangkat stasioner penerima disebut **`Edge`**. Penamaan ini sengaja dibuat berbeda dari istilah TPMS pada umumnya (di mana unit kendaraan biasa disebut "node" dan unit penerima disebut "gateway") agar konsisten dengan konvensi penamaan proyek **Lokotepe**.

---

## 📑 Daftar Isi

- [Arsitektur Sistem](#-arsitektur-sistem)
- [Struktur Repository](#-struktur-repository)
- [Gateway V1](#-gateway-v1)
- [Edge V1](#-edge-v1)
- [Protokol Komunikasi LoRa](#-protokol-komunikasi-lora)
- [Alur Kerja End-to-End](#-alur-kerja-end-to-end)
- [Memulai (Getting Started)](#-memulai-getting-started)
- [Topic & Payload MQTT](#-topic--payload-mqtt)
- [Troubleshooting](#-troubleshooting)
- [Roadmap](#-roadmap)
- [Lisensi](#-lisensi)

---

## 🏗 Arsitektur Sistem

```
 [Sensor BLE TPMS x16]
          │ BLE advertise
          ▼
 ┌─────────────────────┐        LoRa P2P        ┌─────────────────────┐       WiFi / MQTT      ┌────────────────────┐
 │     GATEWAY V1       │ ─────────────────────▶ │       EDGE V1        │ ──────────────────────▶│  Broker MQTT /     │
 │ (ESP32U + LilyGo XY  │                        │ (ESP32C3 + SX1276)   │                         │  Dashboard          │
 │  LoRa Shield SX1276) │ ◀───── BLE NUS ──────── │                      │ ◀── LittleFS S&F ─────  │                    │
 └─────────────────────┘   (HP via app BLE       └─────────────────────┘   (bila WiFi/MQTT putus) └────────────────────┘
                             terminal)
```

- **Gateway V1** dipasang di kendaraan (truk/trailer), membaca seluruh sensor TPMS via BLE, lalu mengirim 1 paket LoRa per ban ke Edge.
- **Edge V1** bersifat stasioner, menerima paket dari banyak Gateway sekaligus (maks. 10), menggabungkan data per ban menjadi 1 bundel JSON per Gateway, dan mempublikasikannya ke broker MQTT.
- Satu **Edge V1** dapat menangani hingga **10 unit Gateway V1**.
- Satu **Gateway V1** dapat menangani hingga **16 sensor ban**.

---

## 📂 Struktur Repository

```
Lokotepe/
└── Phase1/
    ├── GatewayV1/
    │   └── GatewayV1.ino      # firmware unit kendaraan (eks "Node")
    ├── EdgeV1/
    │   └── EdgeV1.ino         # firmware unit penerima/stasioner (eks "Gateway")
    └── README.md               # dokumen ini
```

---

## 🚛 Gateway V1

*(sebelumnya disebut "TPMS Node v4.2" — unit yang terpasang di kendaraan)*

### Fitur Utama

- Membaca **16 sensor BLE TPMS** sekaligus, satu per posisi ban.
- Mengirim data ke Edge via **LoRa P2P**, satu paket per pembacaan sensor.
- **BLE GATT Server (Nordic UART Service)** — Gateway dapat diakses langsung dari HP seperti aksesoris Bluetooth biasa, dikendalikan lewat aplikasi terminal BLE umum (`Serial Bluetooth Terminal`, `nRF Connect`).
- **Auto-Pair berbasis RSSI** — dekatkan sensor ke unit, kirim `P-<slot>`, sensor terdekat otomatis terpasang ke slot tersebut tanpa perlu mengetik MAC address.
- **Pairing manual** sebagai alternatif (`p-<posisi> <MAC>`).
- **Mode scan** (`scan_tpms`, 120 detik) untuk melihat semua sensor TPMS di sekitar tanpa langsung memasangkannya.
- **Decoding otomatis** data mentah sensor (tegangan, suhu, tekanan absolut → gauge kPa & PSI, kode status).
- **Antrian TX (circular buffer, 64 slot)** dengan deduplikasi per posisi ban, mencegah data basi menumpuk di antrian.
- **Diagnostik TX radio** per ban.
- **Penyimpanan pairing persisten** (NVS/`Preferences`), tahan restart/listrik mati.

### Hardware

| Item | Spesifikasi |
|---|---|
| MCU | ESP32U DevKit |
| Shield Radio | LilyGo XY LoRa Shield (SX1276) |
| SCK / MISO / MOSI | GPIO 18 / 19 / 23 |
| SS (CS) | GPIO 5 |
| RST | GPIO 14 |
| DIO0 | GPIO 26 |
| Frekuensi LoRa | 922 MHz |
| TX Power | 20 (maksimum) |

### Library

| Library | Fungsi |
|---|---|
| `arduino-LoRa` (sandeepmistry) | Driver radio SX1276 |
| `NimBLE-Arduino` | Stack BLE — sebagai **server** (ke HP) & **scanner** (membaca sensor TPMS) |
| `Preferences` (bundled ESP32 core) | Penyimpanan pairing MAC sensor |

### Konfigurasi Identitas (wajib diganti per unit)

```cpp
#define NODE_ID       "TRL001"        // ID unik unit, jadi identifier di paket LoRa & topic MQTT
#define BLE_DEV_NAME  "TPMS_TRL001"   // Nama yang muncul di daftar Bluetooth HP
```

> ⚠️ `NODE_ID` harus unik per unit. Jika dua unit Gateway V1 memakai ID yang sama, Edge akan menganggapnya sebagai satu unit yang sama (data tercampur).

### BLE UART Service (Nordic UART Standard)

| UUID | Peran |
|---|---|
| `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` | Service utama |
| `6E400002-...` | RX (HP → Unit), `WRITE` |
| `6E400003-...` | TX (Unit → HP), `NOTIFY` |

### Mapping Slot Ban

Didesain untuk kendaraan bersumbu ganda (4 sumbu × 2 sisi × 2 ban kembar = 16):

| Slot | Label | Perintah |
|---|---|---|
| 0–1 | FL, FL-2 | `fl`, `fl-2` |
| 2–3 | FR, FR-2 | `fr`, `fr-2` |
| 4–5 | RL, RL-2 | `rl`, `rl-2` |
| 6–7 | RR, RR-2 | `rr`, `rr-2` |
| 8–9 | ML, ML-2 | `ml`, `ml-2` |
| 10–11 | MR, MR-2 | `mr`, `mr-2` |
| 12–13 | R2L, R2L-2 | `r2l`, `r2l-2` |
| 14–15 | R2R, R2R-2 | `r2r`, `r2r-2` |

### Daftar Perintah (Serial USB & BLE)

| Perintah | Fungsi |
|---|---|
| `help` | Daftar perintah + mapping slot |
| `P-<0..15>` | Mulai auto-pair untuk slot (contoh `P-0`) |
| `p-<pos> <MAC>` | Pairing manual (contoh `p-fl aa:bb:cc:dd:ee:ff`) |
| `status` | Daftar 16 slot + MAC terpasang |
| `report` | Data tekanan/suhu/volt semua ban |
| `unpair-<pos>` | Hapus pairing satu slot |
| `unpair_all` | Hapus semua pairing |
| `diag` | Statistik TX radio per ban |
| `diag_reset` | Reset statistik |
| `scan_tpms` | Scan legacy 120s, lihat semua sensor TPMS di sekitar |

### Tuning Penting

| Konstanta | Default | Keterangan |
|---|---|---|
| `TX_INTERVAL_MS` | 300 | Jeda minimum antar kirim LoRa |
| `AUTOPAIR_SCAN_SEC` | 10 | Durasi scan saat auto-pair |
| `AUTOPAIR_RSSI_MIN/MAX` | -50 / -30 dBm | Rentang RSSI yang diterima auto-pair |
| `SCAN_RESTART_MS` | 120000 | Interval restart BLE scan kontinu |

---

## 📡 Edge V1

*(sebelumnya disebut "TPMS Gateway v6" — unit penerima/stasioner)*

### Fitur Utama

- Penerima LoRa **multi-Gateway** (hingga 10 unit), masing-masing diidentifikasi via ID.
- **Sistem Pair/Unpair per unit Gateway** — unit yang belum di-*pair* datanya diterima tapi diabaikan untuk publish (anti data sampah).
- **Mode Scan** (30 detik) untuk melihat unit Gateway aktif sebelum memutuskan pairing.
- **Cache per ban dengan deteksi stale** — tidak ada update >3 menit → status otomatis `NO_PACKET`.
- **Publish ke MQTT** dalam bentuk 1 JSON bundel (16 ban) per unit Gateway, dipicu oleh data baru **dan** secara periodik tiap 60 detik.
- **WiFi & broker MQTT tidak hardcode** — pakai **WiFiManager** (captive portal), kredensial diisi lewat browser HP/laptop tanpa reflash firmware.
- **OTA firmware update** (`ArduinoOTA`) lewat jaringan, tanpa kabel USB.
- **Store & Forward via LittleFS** — data yang gagal terkirim (WiFi/MQTT putus) disimpan di flash internal, otomatis dikirim ulang saat koneksi pulih.
- **Diagnostik lengkap** via serial (radio, MQTT, WiFi, OTA, filesystem).
- **Konfigurasi tersimpan persisten** di NVS: daftar Gateway yang dipair, host/port broker MQTT.

### Hardware

| Item | Spesifikasi |
|---|---|
| MCU | ESP32C3 Mini |
| Modul Radio | SX1276 (LoRa), wiring custom |
| SCK / MISO / MOSI | GPIO 4 / 5 / 6 |
| CS (NSS) | GPIO 7 |
| RST | GPIO 2 |
| DIO0 | GPIO 3 |
| Frekuensi LoRa | 922 MHz (ganti `868E6` jika region berbeda) |

### Library

| Library | Sumber | Fungsi |
|---|---|---|
| `arduino-LoRa` | sandeepmistry | Driver radio SX1276 |
| `ArduinoJson` | v7 | Serialisasi payload MQTT |
| `PubSubClient` | — | Klien MQTT |
| `WiFiManager` | tzapu | Captive portal WiFi + parameter custom |
| `ArduinoOTA`, `Preferences`, `LittleFS` | bundled ESP32 core | OTA, NVS, filesystem flash |

### Setup WiFi & MQTT (Captive Portal)

1. Nyalakan unit pertama kali (atau setelah `wifi_reset`).
2. Konek HP/laptop ke Access Point **`TPMS-Gateway-Setup`** (password default `tpms12345`).
3. Browser otomatis membuka halaman konfigurasi: pilih WiFi, isi password, isi **MQTT Broker Host/IP** dan **Port** (default `1883`).
4. Kredensial tersimpan otomatis, dipakai ulang di boot berikutnya.

> ⚠️ Timeout portal 180 detik. Selama portal terbuka, unit **tidak memproses paket LoRa/serial** (perilaku blocking bawaan WiFiManager).
> ⚠️ **Ganti `AP_PASSWORD` dan `OTA_PASSWORD`** (default `tpms12345`) sebelum dipakai di lapangan.

### OTA Firmware Update

- Hostname: `tpms-gateway` (muncul di Arduino IDE sebagai *Network Port*, port `3232`).
- Password OTA terpisah dari password AP, diatur lewat `OTA_PASSWORD`.

### Store & Forward (LittleFS)

| File | Fungsi |
|---|---|
| `/queue.txt` | Antrian pesan gagal kirim, format `topic<TAB>payload` per baris |
| `/temp.txt` | Buffer sementara saat proses forward |

Diproses otomatis setiap **5 detik** saat MQTT terkoneksi; baris sukses dihapus, baris gagal dicoba lagi di siklus berikutnya.

### Daftar Perintah (Serial)

| Perintah | Fungsi |
|---|---|
| `help` | Daftar perintah |
| `scan` | Scan unit Gateway aktif (30 detik) |
| `pair <ID>` | Pair unit Gateway, contoh `pair TRL001` |
| `unpair <ID>` | Unpair unit tertentu |
| `unpair_all` | Unpair semua |
| `list` | Daftar semua unit terdeteksi + status pairing |
| `status` | Data ban semua unit yang dipair |
| `json` | Dump semua data ke serial (debug) |
| `diag` / `diag_reset` | Diagnostik lengkap / reset statistik |
| `mqtt_status` | Status koneksi MQTT + topic aktif |
| `queue` / `clearqueue` | Info / hapus antrian LittleFS |
| `show_config` | Tampilkan konfigurasi aktif |
| `set_broker <host> [port]` | Ganti broker MQTT cepat via serial (auto restart) |
| `wifi_config` | Buka ulang portal setup WiFi/MQTT |
| `wifi_reset` | Hapus kredensial WiFi tersimpan |

### Tuning Penting

| Konstanta | Default | Keterangan |
|---|---|---|
| `MAX_NODES` | 10 | Maks unit Gateway yang ditangani |
| `MAX_TIRES` | 16 | Maks ban per unit Gateway |
| `DATA_STALE_MS` | 180000 (3 menit) | Ambang data dianggap basi → `NO_PACKET` |
| `PERIODIC_PUBLISH_MS` | 60000 | Interval publish ulang semua unit terpasang |
| `FS_PROCESS_INTERVAL_MS` | 5000 | Interval coba forward antrian LittleFS |

> ⚠️ Buffer `PubSubClient` diperbesar ke **2560 byte** (`mqttClient.setBufferSize(2560)`) karena payload JSON 16 ban bisa ±2120 byte — jangan dihapus/diturunkan saat modifikasi kode, atau publish akan gagal diam-diam.

---

## 🔗 Protokol Komunikasi LoRa

Struct biner berikut **wajib identik** di kedua firmware (dikirim mentah, `__attribute__((packed))`):

```cpp
#define PKT_MAGIC  0xA55A
#define PKT_VER    0x04

struct PktHeader {
    uint16_t magic;      // harus == 0xA55A
    uint8_t  ver;
    char     nodeId[7];  // ID unit Gateway, contoh "TRL001"
    uint8_t  tireCount;
};

struct TpmsData {
    uint32_t timestamp;
    uint16_t statusCode;
    uint16_t volt_mV;
    uint16_t kpa_x10;     // kPa gauge x10
    uint8_t  tireId;      // 0..15
    int8_t   temp;        // °C
    int8_t   bleRssi;
    uint8_t  chksum;      // XOR seluruh byte sebelumnya
};

struct LoRaPacket {
    PktHeader hdr;
    TpmsData  data;
};
```

Parameter radio (harus sama persis di Gateway V1 & Edge V1):

| Parameter | Nilai |
|---|---|
| Spreading Factor | 7 |
| Bandwidth | 125 kHz |
| Coding Rate | 4/5 |
| Sync Word | `0x12` |
| CRC | Aktif |

Setiap paket hanya membawa data **1 ban**, sehingga 1 unit Gateway V1 mengirim hingga 16 paket per siklus pembacaan penuh.

---

## 🔄 Alur Kerja End-to-End

1. Sensor BLE TPMS memancarkan advertisement (service UUID `0xA827`) berisi data tekanan/suhu/voltase.
2. **Gateway V1** men-scan BLE secara kontinu, mendecode data dari MAC yang sudah dipair, lalu memasukkannya ke antrian TX.
3. Setiap 300ms, **Gateway V1** mengirim 1 paket LoRa dari antrian.
4. **Edge V1** menerima paket, validasi magic & checksum, cek apakah ID pengirim sudah dipair.
   - Belum dipair → diabaikan (hanya update RSSI/waktu terakhir terlihat).
   - Sudah dipair → update cache ban, langsung publish bundel JSON ke `tpms/<ID>`.
5. Jika MQTT/WiFi putus → data masuk antrian LittleFS, dikirim ulang otomatis saat koneksi pulih.
6. Setiap 60 detik, **Edge V1** mempublikasikan ulang semua unit yang dipair (walau tanpa paket baru), agar ban yang berhenti update tetap terlihat sebagai `NO_PACKET` di dashboard, bukan diam tak terdeteksi.

---

## 🚀 Memulai (Getting Started)

### Prasyarat

- Arduino IDE (atau PlatformIO) dengan board package **ESP32** terpasang.
- Library: `arduino-LoRa`, `ArduinoJson` (v7), `PubSubClient`, `WiFiManager` (tzapu) — install lewat Library Manager.
- `NimBLE-Arduino` untuk Gateway V1.

### Langkah Flashing

1. Clone repository:
   ```bash
   git clone https://github.com/irhamre/Lokotepe.git
   cd Lokotepe/Phase1
   ```
2. **Gateway V1** (unit kendaraan):
   - Buka `GatewayV1/GatewayV1.ino`.
   - Ganti `NODE_ID` dan `BLE_DEV_NAME` agar unik per unit.
   - Pilih board ESP32U DevKit, upload via USB.
3. **Edge V1** (unit stasioner):
   - Buka `EdgeV1/EdgeV1.ino`.
   - (Opsional) ganti `AP_PASSWORD`, `OTA_PASSWORD` untuk keamanan.
   - Pilih board ESP32C3 Mini, upload via USB.
   - Setelah nyala, lakukan setup WiFi & broker MQTT lewat captive portal `TPMS-Gateway-Setup` (lihat [bagian Edge V1](#edge-v1)).
4. Pairing:
   - Di **Gateway V1**: kirim `P-<slot>` per sensor (auto-pair) atau `p-<pos> <MAC>` manual.
   - Di **Edge V1**: kirim `scan` untuk melihat unit Gateway aktif, lalu `pair <ID>` untuk mengaktifkan publish-nya ke MQTT.

---

## 📨 Topic & Payload MQTT

| Topic | Isi |
|---|---|
| `tpms/gateway/status` | Status online Edge V1: `{"status":"online","hw":"ESP32C3_SX1276"}` |
| `tpms/<ID>` | Bundel JSON 16 ban untuk satu unit Gateway V1 (contoh ID: `TRL001`) |

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
    { "id": 0, "status": "OK", "kpa": 320.5, "temp_c": 34, "volt_v": 3.05, "rssi_ble": -42, "status_code": 2, "age_s": 7 },
    { "id": 1, "status": "NO_PACKET" }
  ]
}
```

---

## 🛠 Troubleshooting

| Gejala | Kemungkinan Sebab | Solusi |
|---|---|---|
| Edge V1 tidak menerima paket sama sekali | Parameter radio tidak identik, atau wiring SX1276 salah | Samakan SF/BW/CR/Sync word; cek pinout |
| Unit Gateway terdeteksi tapi data tidak update | Belum di-`pair` di Edge V1 | Jalankan `pair <ID>` |
| Status ban selalu `NO_PACKET` | Sensor belum terpasang ke slot di Gateway V1 | Cek `status`, ulangi `P-<slot>` |
| Publish MQTT gagal terus | Buffer PubSubClient diturunkan/dihapus | Pastikan `setBufferSize(2560)` ada |
| Auto-pair selalu `FAIL` | Sensor di luar rentang RSSI -50..-30 dBm, atau UUID sensor berbeda | Atur jarak; cek UUID advertisement sensor |
| Tidak bisa OTA Edge V1 | WiFi belum konek / password OTA salah | Cek `show_config`, cek `OTA_PASSWORD` |

---

## 🗺 Roadmap

- [ ] Enkripsi/otentikasi payload LoRa (saat ini plaintext + checksum saja)
- [ ] TLS & autentikasi username/password untuk MQTT
- [ ] OTA untuk Gateway V1 (saat ini hanya Edge V1 yang punya OTA)
- [ ] Store & forward persisten di sisi Gateway V1 (saat ini hanya antrian RAM 64 slot)
- [ ] ACK/retry pada link LoRa Gateway→Edge
- [ ] Dukungan multi-Edge / failover untuk area jangkauan terbatas

---

## 📄 Lisensi

Belum ditentukan — tambahkan file `LICENSE` sesuai kebutuhan proyek **Lokotepe**.

# TPMS Monitoring System using ESP32, LilyGO T3, and Nordic nRF52840

## Overview

This repository contains source code and documentation for the development of a **Tire Pressure Monitoring System (TPMS)** monitoring platform using a combination of **ESP32**, **LilyGO T3 (LoRa)**, and **Nordic nRF52840 DK**.

The system is designed to collect TPMS sensor data via Bluetooth Low Energy (BLE), process the information, and transmit it through a wireless Point-to-Point (P2P) communication network for centralized monitoring.

---

# Repository Structure

## 1. Gateway & Node (P2P Communication System)

### Gate_Test_P2P

Gateway firmware for the **LilyGO T3** board.

**Features:**

* P2P communication receiver
* Radio diagnostic monitoring
* Displays data received from up to 12 TPMS sensors
* RSSI and communication quality monitoring

---

### Node_Test_P2P

Node firmware for **ESP32 + CC1101**.

**Features:**

* BLE TPMS data acquisition
* Supports up to 12 TPMS sensors
* Data processing and packaging
* Wireless P2P transmission to Gateway

---

### Gate V2

Second-generation Gateway firmware.

**Improvements:**

* Supports up to 30 TPMS sensors
* Enhanced radio diagnostic functions
* Improved packet handling
* Optimized monitoring interface

---

### Node V2

Second-generation Node firmware.

**Improvements:**

* Supports up to 30 TPMS sensors
* Enhanced BLE scanning and parsing
* Radio diagnostic functionality
* Improved communication reliability

---

## 2. TPMS Sensor Reader Module

### TPMS_Nordic

Firmware developed specifically for the **Nordic nRF52840 DK**.

**Features:**

* BLE scanning and TPMS detection
* TPMS packet decoding and parsing
* Simultaneous monitoring of up to 12 TPMS sensors
* Platform evaluation for future BLE-based TPMS systems

---

## 3. Documentation

### TPMS nRF on Arduino IDE.pdf

Step-by-step guide for programming the **Nordic nRF52840 DK** using Arduino IDE.

Contents include:

* Installing Board Support Package (BSP)
* Arduino IDE configuration
* Bootloader setup
* USB driver configuration
* Upload procedures
* Troubleshooting serial port issues
* Common development problems and solutions

---

# System Architecture

```text
TPMS Sensors
      │
      │ BLE
      ▼
+----------------+
| ESP32 / nRF52840 |
| TPMS Reader      |
+----------------+
      │
      │ P2P Radio
      ▼
+----------------+
| LilyGO T3      |
| Gateway        |
+----------------+
      │
      ▼
Monitoring System
```

---

# Hardware Summary

| Component          | Role          | Main Functions                                          |
| ------------------ | ------------- | ------------------------------------------------------- |
| LilyGO T3          | Gateway       | P2P Communication, Radio Diagnostic                     |
| ESP32 + CC1101     | Node          | BLE TPMS Acquisition, Data Processing, P2P Transmission |
| Nordic nRF52840 DK | Sensor Reader | BLE TPMS Scanning and Processing                        |

---

# Supported Features

* TPMS sensor monitoring
* Tire pressure acquisition
* Tire temperature acquisition
* BLE communication
* P2P wireless communication
* RSSI monitoring
* Radio diagnostics
* Multi-sensor support
* Expandable architecture

---

# Getting Started

## Nordic nRF52840 Development

Refer to:

```text
TPMS nRF on Arduino IDE.pdf
```

for complete setup instructions.

Topics covered:

* Arduino IDE setup
* Board installation
* USB driver configuration
* Upload process
* Debugging and troubleshooting

---

## ESP32 & LilyGO T3 Development

### Requirements

Install the following libraries:

* CC1101 Radio Library
* BLE Library (ESP32)
* SPI Library
* ArduinoJson (if required)

### Configuration

Before uploading:

1. Verify pin assignments in the source code.
2. Configure radio frequency parameters.
3. Configure TPMS sensor IDs if necessary.
4. Verify hardware connections.

---

# Applications

This project can be adapted for:

* Trailer TPMS monitoring
* Fleet monitoring systems
* Heavy-duty vehicle monitoring
* Industrial vehicle monitoring
* Smart transportation systems
* Wireless sensor network research
* BLE reverse engineering studies

---

# Development Status

Current focus areas:

* BLE TPMS reverse engineering
* Nordic nRF52840 evaluation
* Long-range P2P communication
* Radio link diagnostics
* Multi-node TPMS architecture
* Reliability and performance optimization

---

# License

This repository, including all source code, documentation, schematics, and related materials, is the intellectual property of Qimtronics.

Please review individual library licenses before commercial deployment.

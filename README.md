# Distributed Temperature Monitoring & Estimation System

**EE2120 – Second Year Engineering Project**  
**Group 25 – Department of Electrical & Electronic Engineering**  
**University of Peradeniya**

A distributed smart temperature monitoring system based on **ESP32**, heterogeneous temperature sensors, adaptive sensor fusion, GPS positioning, secure MQTT communication, and a **Node-RED SCADA dashboard**.

---

## Project Overview

This project develops an intelligent temperature-monitoring node using two different temperature sensors:

- **LM35DZ waterproof analog temperature sensor**
- **DS18B20 waterproof digital temperature sensor**

The sensor readings are processed using an ESP32 and combined using two sensor-fusion techniques:

1. **Adaptive Confidence-Weighted Fusion**
2. **Simple 1D Kalman Filter**

The system also integrates:

- GPS positioning
- OLED display
- microSD data logging
- Wi-Fi communication
- MQTT over TLS
- Mosquitto MQTT broker
- Node-RED based SCADA
- Alarm and diagnostic monitoring
- Distributed temperature estimation using IDW

---

## Main Objectives

The main objectives of the project are to:

- Measure temperature using two heterogeneous sensors.
- Characterize the actual installed sensor system experimentally.
- Improve measurement robustness using sensor fusion.
- Compare adaptive fusion with a simple Kalman filter.
- Display measurements locally using an OLED.
- Record experimental and operational data to a microSD card.
- Acquire GPS coordinates for distributed monitoring.
- Securely transmit sensor data through MQTT.
- Develop a Node-RED SCADA interface for monitoring and diagnostics.
- Support distributed temperature estimation using multiple GPS-enabled nodes.

---

## System Architecture

The basic measurement and processing path is:

```text
LM35DZ ───────┐
              │
              ├──> ESP32 ──> Sensor Fusion ──> OLED Display
DS18B20 ──────┘        │
                       ├──> microSD Logging
GPS NEO-6M ────────────┤
                       │
                       └──> Wi-Fi / MQTT over TLS
                                  │
                                  ▼
                           Mosquitto Broker
                                  │
                                  ▼
                              Node-RED
                                  │
                                  ▼
                           SCADA Dashboard
                                  │
                                  ▼
                               Browser

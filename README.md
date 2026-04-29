# 🛰️ Air Alarm Tracker — ESP32-S3 (ESP-IDF)


**Air Alarm Tracker** is an embedded system based on the **ESP32-S3** microcontroller. It performs real-time monitoring of air raid alerts in Kyiv and the surrounding region by fetching data directly from [alert.in.ua](http://alert.in.ua/).

---

## 📋 Features
* **Autonomous Monitoring:** The device connects to Wi-Fi and polls the server every 10 seconds.
* **Visual Alerts:** A WS2812B LED ring changes color based on the current threat level.
* **Informative UI:** An SSD1306 OLED display shows text statuses ("ALARM", "GOOD", "WARNING") and connection health.
* **Reliability:** Built on **FreeRTOS** to separate network synchronization from interface rendering tasks.

---

## 🚦 Visual Logic (LED & Display)

The system automatically transitions between states based on the API response:

| State | LED Color | OLED Display | Description |
| :--- | :---: | :--- | :--- |
| **Danger** | 🔴 Red | `Kiev: ALARM` | Active air raid alert in the city. |
| **Warning** | 🟡 Yellow | `Kiev: WARNING` | Alert in the region / heightened caution. |
| **Normal** | 🔵 Blue | `Kiev: GOOD` | All clear. Monitoring in progress. |
| **Offline** | ⚪ White | `Connecting...` | Initializing Wi-Fi or API request failed. |

---

## 🛠️ Technical Architecture

### Hardware
The project utilizes the **ESP32-S3** dual-core processor with the following peripheral configuration:

* **Display:** SSD1306 (128x64) I2C OLED.
    * `SDA` -> **GPIO 8**
    * `SCL` -> **GPIO 9**
* **LEDs:** WS2812B RGB Ring (12 LEDs).
    * `Data` -> **GPIO 16** (driven via the **RMT** peripheral).
* **Power:** 5V via USB-C.

### Software Implementation
The firmware is written in **C/C++** using the **ESP-IDF** framework:
* **Event Groups:** Manages Wi-Fi states (Connected/Fail) asynchronously.
* **HTTP Client:** Fetches raw data from `alert.in.ua` with custom string-parsing logic.
* **Custom Graphics:** A lightweight, custom implementation of 5x7 fonts for the SSD1306, rendered directly via the I2C Master driver.
* **FreeRTOS Tasks:** The main logic is encapsulated in a dedicated `main_loop_task` with a 100ms cycle.

---

## 📡 Project Diagram

### Hardware Signal Path
This diagram illustrates the physical connections between the ESP32-S3 and the external modules:

> **<img width="1199" height="1280" alt="image" src="https://github.com/user-attachments/assets/1c7b659e-0c63-4ed0-b6d4-63919bbbfd5d" />
**

---

## ⚙️ Setup & Installation

1.  **WiFi Configuration:**
    Locate the definitions in `main.cpp` (or `main.c`) and update your credentials:
    ```c
    #define WIFI_SSID "Your_SSID"
    #define WIFI_PASS "Your_Password"
    ```
2.  **Build & Flash:**
    The project uses the standard ESP-IDF build system:
    ```bash
    idf.py build
    idf.py -p [PORT] flash monitor
    ```
3.  **Polling Interval:** The default fetch interval is set to 10 seconds (`FETCH_INTERVAL_MS 10000`).

---

## 🔬 Technology Stack

* **Microcontroller:** Espressif ESP32-S3.
* **Protocol:** HTTP / JSON.
* **Libraries:** * `esp_http_client` for networking.
    * `led_strip` (RMT) for hardware-accelerated LED control.
    * Custom I2C SSD1306 driver.

---


**Developer:** Maksymenko Vladyslav  
**Embedded Systems Graduation Project — 2026**

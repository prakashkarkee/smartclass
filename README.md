![Dashboard Screenshot](/images/project_images/summer_programmer.png)


# Smart Classroom IoT-Based Monitoring System Project

## Table of Contents

- [Introduction](#introduction)  
- [Problem Statement](#problem-statement)  
- [Goals and Objectives](#goals-and-objectives)  
- [Features](#features)  
- [Tools and Methodology](#tools-and-methodology)  
- [System Architecture](#system-architecture)  
- [Circuit Diagram](#circuit-diagram)  
- [System Logic](#system-logic)  
- [Results](#results)  
- [Challenges & Future Works](#challenges--future-works)  
- [Conclusion](#conclusion)  
- [Team Members](#team-members)  
- [Project Code (ESP32)](#project-code-esp32)  
- [Dependencies](#dependencies)  
- [References](#references)  

## Introduction
Classroom environments directly impact student focus, comfort, and learning outcomes. Traditional classrooms lack automated monitoring of temperature, noise, and occupancy, making energy management and real-time supervision difficult. This project aims to solve this by creating a smart classroom system that monitors and automates classroom conditions, such as temperature, humidity, motion, and noise, to improve comfort and energy efficiency.

![Dashboard Screenshot](/images/project_images/dashboard.jpg)
![Dashboard Screenshot](/images/project_images/chart_info.jpg)

## Problem Statement
- Uncomfortable classroom environment due to temperature & noise.
- Energy wastage from inefficient lighting, air conditioning (AC), and heater use.
- No centralized system for real-time monitoring of environmental conditions.

## Goals and Objectives
- Develop a real-time IoT monitoring system for classroom environments.
- Automate classroom comfort by controlling temperature and noise levels.
- Improve energy efficiency through automated control of lighting, AC, and heater systems.
- Enable cloud-based data logging and visualization using Firebase.

## Features

- Real-time status display for sensors: AC, Heater, Humidity, Light, Motion, Noise, Temperature
- Historical data charts for each sensor
- Toggle between today's data and all historical data
- Responsive and modern UI

## Tools and Methodology
- **ESP-32 Microcontroller**: Controls the smart classroom system.
- **Firebase and HTML**: Used for real-time data storage and displaying historical statistics on a web-based dashboard.
- **Sensors**:
  - **DHT11**: Measures temperature and humidity.
  - **PIR**: Detects motion for occupancy monitoring.
  - **I2S Microphone**: Monitors noise levels in the classroom.
  - **LEDs**: Indicate the status of the lighting, AC, and heater.

## System Architecture
1. **ESP-32 Microcontroller**: Acts as the heart of the system, controlling the sensors and communicating with the cloud.
2. **Sensors**: Collect data about classroom conditions:
   - Temperature and humidity via DHT11.
   - Motion via PIR sensor.
   - Noise levels via I2S microphone.
3. **Firebase Database**: Stores the sensor data and allows for cloud-based monitoring and visualization.
4. **Actuators**: Control LEDs to indicate the operational status of lights, AC, and heater.

## Circuit Diagram
- The system uses an ESP-32 to interact with various sensors (PIR, DHT11, I2S microphone) and actuators (LEDs for AC, heater, and light status).
- **Wiring**: Detailed connections of the ESP-32 with the sensors and actuators will be provided.

![Dashboard Screenshot](/images/project_images/schematic_diagram.jpg)

## System Logic
- The system monitors classroom conditions in real-time.
- Based on data (e.g., motion detected, temperature threshold exceeded), it automatically controls lighting, AC, and heaters to optimize comfort and energy use.

![Dashboard Screenshot](/images/project_images/flowchart.jpg)

## Results
- **Real-Time Monitoring**: The system displays live data (temperature, humidity, motion, and noise) on a web dashboard.
- **Automated Control**: The system can automatically control AC and heater systems based on occupancy (motion detection) and temperature.
- **Cloud Logging**: Firebase logs the data for later analysis, enabling access to historical statistics.

## Challenges & Future Works
- **Memory Limitation**: Current integration is limited due to memory constraints on the ESP32.
- **Future Enhancements**:
  - Integration of machine learning for voice activation.
  - Teacher notification system for real-time alerts.
  - Improved BLE (Bluetooth Low Energy) integration for additional connectivity options.

## Conclusion
The Smart Classroom IoT-Based Monitoring System improves classroom comfort by automating temperature, noise, and occupancy control. It helps in energy conservation through efficient use of lighting, AC, and heating systems. This project aims to provide a more comfortable and energy-efficient environment for both students and teachers.


## Team Members

- **Prakash Karkee** – Computer Science and Engineering (CSE), ITEE  
- **Brian Kiprop** – Wireless Communication (CWC), ITEE  
- **Lihini Karunarathne** – Artificial Intelligence (AI), ITEE  

### Supervisors and Mentors
- **Hanna Saarela** – Development Manager, Faculty of Information Technology and Electrical Engineering, University of Oulu  
- **Christian Schuss** – University Lecturer, Faculty of Information Technology and Electrical Engineering, University of Oulu  
- **Anuradha Athukorala** – University Trainee, University of Oulu

## Project Code (ESP32)

```cpp
#include <Arduino.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include "time.h"
#include "DHT.h"
#include <math.h>
#include "driver/i2s.h"

// ========================
// Wi-Fi & Firebase
// ========================
#define WIFI_SSID "University"
#define WIFI_PASSWORD "someone2334"
#define API_KEY "ghteCMHPff049YeCmjaA2J-wab1dTrPS3fdsfdsf"
#define DATABASE_URL "https://smartclassoulu-default-rtdb.europe-west1.firebasedatabase.app"
#define USER_EMAIL "firebaseEmail@gmail.com"
#define USER_PASSWORD "password"

FirebaseData fbData;
FirebaseConfig fbConfig;
FirebaseAuth fbAuth;
FirebaseESP32 firebase;

// ========================
// Pins & Sensors
// ========================
#define PIR_PIN 14
#define LED_PIN 16
#define DHTPIN 21
#define DHTTYPE DHT11
#define HEATER_PIN 12
#define AC_PIN 27

// I2S pin configuration
#define I2S_WS      25
#define I2S_SD      33
#define I2S_SCK     32
#define I2S_PORT    I2S_NUM_0

#define SAMPLE_RATE     16000
#define SAMPLE_BITS     32
#define BUFFER_SIZE     512        // DMA read buffer size
#define AVERAGE_BUFFERS 4          // Number of buffers to average
#define SMOOTHING_ALPHA 0.2        // Moving average weight

DHT dht(DHTPIN, DHTTYPE);

// ========================
// Variables
// ========================
bool motionDetected = false;
unsigned long lastMotionTime = 0;
bool heaterOn = false;
bool acOn = false;

const unsigned long MOTION_TIMEOUT = 120000;   // 2 min
const unsigned long DHT_INTERVAL = 5000;       // 5 sec
const unsigned long NOISE_INTERVAL = 2000;     // 2 sec
const unsigned long STATUS_INTERVAL = 5000;    // 5 sec
const unsigned long FIREBASE_INTERVAL = 180000; // 3 min

unsigned long lastDHTRead = 0;
unsigned long lastNoiseRead = 0;
unsigned long lastStatusPrint = 0;
unsigned long lastFirebaseUpdate = 0;

float currentTemp = NAN;
float currentHum = NAN;
double currentNoiseDB = 0;
double smoothed_dB = 0; // For smoothing

// Hysteresis for AC/Heater
const float TEMP_LOWER = 20.0;
const float TEMP_UPPER = 22.0;

// ========================
// I2S configuration
// ========================
i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = i2s_bits_per_sample_t(SAMPLE_BITS),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = BUFFER_SIZE,
    .use_apll = false
};

i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
};

// ========================
// Setup
// ========================
void setup() {
    Serial.begin(115200);

    pinMode(PIR_PIN, INPUT);
    pinMode(HEATER_PIN, OUTPUT);
    pinMode(AC_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(HEATER_PIN, LOW);
    digitalWrite(AC_PIN, LOW);
    digitalWrite(LED_PIN, LOW);

    dht.begin();

    // I2S microphone
    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_PORT, &pin_config);
    i2s_zero_dma_buffer(I2S_PORT);

    // Wi-Fi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to Wi-Fi");

    // NTP time
    configTime(0, 0, "pool.ntp.org");
    setenv("TZ", "CET-1CEST,M3.5.0/02:00,M10.5.0/03:00", 1);
    tzset();
    time_t now = time(nullptr);
    while (now < 1000000000) {
        delay(500);
        Serial.print(".");
        now = time(nullptr);
    }
    Serial.println("\nTime synchronized!");

    // Firebase
    fbConfig.api_key = API_KEY;
    fbConfig.database_url = DATABASE_URL;
    fbAuth.user.email = USER_EMAIL;
    fbAuth.user.password = USER_PASSWORD;
    firebase.begin(&fbConfig, &fbAuth);
    firebase.reconnectWiFi(true);

    Serial.println("✅ System Started");
}

// ========================
// Functions
// ========================

// --- Motion handler ---
void handleMotion() {
    bool motion = digitalRead(PIR_PIN) == HIGH;
    if (motion) {
        if (!motionDetected) Serial.println("🚶 Motion detected!");
        motionDetected = true;
        lastMotionTime = millis();
        digitalWrite(LED_PIN, HIGH);
    }
    if (motionDetected && millis() - lastMotionTime > MOTION_TIMEOUT) {
        motionDetected = false;
        digitalWrite(LED_PIN, LOW);
        digitalWrite(HEATER_PIN, LOW);
        digitalWrite(AC_PIN, LOW);
        heaterOn = false;
        acOn = false;
        Serial.println("⏹ Motion timeout → Everything OFF");
    }
}

// --- DHT (non-blocking) ---
void readDHT_nonBlocking() {
    if (millis() - lastDHTRead >= DHT_INTERVAL) {
        lastDHTRead = millis();
        float t = dht.readTemperature();
        float h = dht.readHumidity();
        if (!isnan(t)) currentTemp = t;
        if (!isnan(h)) currentHum = h;
    }
}

// --- Noise (non-blocking) ---
void readNoise_nonBlocking() {
    if (millis() - lastNoiseRead >= NOISE_INTERVAL) {
        lastNoiseRead = millis();

        int32_t buffer[BUFFER_SIZE];
        size_t bytesRead = 0;
        if (i2s_read(I2S_PORT, (char*)buffer, sizeof(buffer), &bytesRead, 0) && bytesRead > 0) {
            double totalSum = 0;
            int samples = bytesRead / sizeof(int32_t);
            for (int i = 0; i < samples; i++) {
                double sample = buffer[i] / 2147483648.0;
                totalSum += sample * sample;
            }
            double rms = sqrt(totalSum / samples + 1e-10); // avoid 0
            double dB = 20 * log10(rms) + 94;
            smoothed_dB = SMOOTHING_ALPHA * dB + (1 - SMOOTHING_ALPHA) * smoothed_dB;
            currentNoiseDB = smoothed_dB;
        }
    }
}

// --- Heater/AC control ---
void controlHeaterAC(float temperature, bool motionActive) {
    if (!motionActive || isnan(temperature)) {
        digitalWrite(HEATER_PIN, LOW);
        digitalWrite(AC_PIN, LOW);
        heaterOn = false;
        acOn = false;
        return;
    }
    if (temperature < TEMP_LOWER && !heaterOn) {
        digitalWrite(HEATER_PIN, HIGH);
        digitalWrite(AC_PIN, LOW);
        heaterOn = true;
        acOn = false;
    } else if (temperature > TEMP_UPPER && !acOn) {
        digitalWrite(HEATER_PIN, LOW);
        digitalWrite(AC_PIN, HIGH);
        heaterOn = false;
        acOn = true;
    }
}

// --- Status printing ---
void printStatus() {
    if (millis() - lastStatusPrint >= STATUS_INTERVAL) {
        lastStatusPrint = millis();
        Serial.printf("Temp: %.2f°C, Hum: %.2f%%, Motion: %s, Heater: %s, AC: %s, Light: %s, Noise: %.2f dB, Timestamp: %s\n",
                      currentTemp, currentHum,
                      motionDetected ? "Yes" : "No",
                      heaterOn ? "ON" : "OFF",
                      acOn ? "ON" : "OFF",
                      digitalRead(LED_PIN) ? "ON" : "OFF",
                      currentNoiseDB,
                      getTimestampString().c_str());
    }
}

// --- Timestamp ---
String getTimestampString() {
    time_t now = time(nullptr);
    struct tm* timeinfo = localtime(&now);
    char buffer[30];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
    return String(buffer);
}

// --- Firebase push ---
void pushToFirebase() {
    FirebaseJson json;
    json.set("temperature", isnan(currentTemp) ? 0 : currentTemp);
    json.set("humidity", isnan(currentHum) ? 0 : currentHum);
    json.set("motion", motionDetected ? 1 : 0);
    json.set("heater", heaterOn ? 1 : 0);
    json.set("ac", acOn ? 1 : 0);
    json.set("light", digitalRead(LED_PIN) ? 1 : 0);
    json.set("noise_db", isnan(currentNoiseDB) || isinf(currentNoiseDB) ? 0 : currentNoiseDB);
    json.set("timestamp", getTimestampString());

    String path = "/smartclass/status";

    if (firebase.pushJSON(fbData, path, json)) {
        Serial.println("✅ Firebase updated");
    } else {
        Serial.print("❌ Firebase failed: ");
        Serial.println(fbData.errorReason());
    }
}

// ========================
// Main Loop
// ========================
void loop() {
    handleMotion();
    readDHT_nonBlocking();
    readNoise_nonBlocking();
    controlHeaterAC(currentTemp, motionDetected);
    printStatus();

    if (millis() - lastFirebaseUpdate >= FIREBASE_INTERVAL) {
        lastFirebaseUpdate = millis();
        pushToFirebase();
    }
}
```

## Dependencies
- [Firebase JS SDK](https://firebase.google.com/docs/web/setup)  
- [Chart.js](https://www.chartjs.org/)  
- [Arduino Core for ESP32](https://github.com/espressif/arduino-esp32)  
- [FirebaseESP32 Library](https://github.com/mobizt/Firebase-ESP32)  
- [DHT Sensor Library](https://github.com/adafruit/DHT-sensor-library)  
- [I2S Driver (ESP32)](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2s.html)  

## References
1. **ESP32 Documentation**: [ESP32 Docs](https://www.espressif.com/en/products/hardware/esp32/overview)
2. **Firebase**: [Firebase Docs](https://firebase.google.com/docs)
3. **DHT11 Sensor**: [DHT11 Datasheet](https://www.electronicwings.com)
4. **PIR Sensor**: [PIR Sensor Datasheet](https://www.adafruit.com)
5. **I2S Microphone**: [I2S Microphone Datasheet](https://www.adafruit.com)

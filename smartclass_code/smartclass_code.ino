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
// replace Wifi name, password, Firebase project API key, Database URL, useremail and password
#define WIFI_SSID     "Your_WiFi_Name"
#define WIFI_PASSWORD "Your_WiFi_Password"
#define API_KEY       "Your_Firebase_API_Key"
#define DATABASE_URL  "your-project-id.firebaseio.com"
#define USER_EMAIL    "your-firebase-user@email.com"
#define USER_PASSWORD "your-firebase-password"

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
#define BUFFER_SIZE     1024
#define AVERAGE_BUFFERS 4
#define SMOOTHING_ALPHA 0.2

DHT dht(DHTPIN, DHTTYPE);

// ========================
// Variables
// ========================
bool motionDetected = false;
unsigned long lastMotionTime = 0;
bool heaterOn = false;
bool acOn = false;

const unsigned long MOTION_TIMEOUT   = 360000;   //  6 minutes
const unsigned long DHT_INTERVAL     = 60000;    // 60 sec
const unsigned long NOISE_INTERVAL   = 60000;    // 60 sec
const unsigned long STATUS_INTERVAL  = 75000;    // 75 sec
const unsigned long FIREBASE_INTERVAL= 300000;   // 5 minutes

unsigned long lastDHTRead = 0;
unsigned long lastNoiseRead = 0;
unsigned long lastStatusPrint = 0;
unsigned long lastFirebaseUpdate = 0;

float currentTemp = NAN;
float currentHum = NAN;
double currentNoiseDB = 0;
double smoothed_dB = 0;

const float TEMP_LOWER = 20.00;
const float TEMP_UPPER = 25.00;

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

    // ========================
    // Wi-Fi
    // ========================
    Serial.print("Connecting to Wi-Fi");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 15000) {
        delay(500);
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected to Wi-Fi");
    } else {
        Serial.println("\nWi-Fi failed (offline mode)");
    }

    // ========================
    // NTP time
    // ========================
    if (WiFi.status() == WL_CONNECTED) {
        configTime(0, 0, "pool.ntp.org");
        setenv("TZ", "CET-1CEST,M3.5.0/02:00,M10.5.0/03:00", 1);
        tzset();
        unsigned long ntpStart = millis();
        time_t now = time(nullptr);
        while (now < 1000000000 && millis() - ntpStart < 10000) {
            delay(500);
            Serial.print(".");
            now = time(nullptr);
        }
        if (now < 1000000000) {
            Serial.println("\nNTP failed (using millis)");
        } else {
            Serial.println("\nTime synchronized!");
        }
    }

    // ========================
    // Firebase
    // ========================
    if (WiFi.status() == WL_CONNECTED) {
        fbConfig.api_key = API_KEY;
        fbConfig.database_url = DATABASE_URL;
        fbAuth.user.email = USER_EMAIL;
        fbAuth.user.password = USER_PASSWORD;
        firebase.begin(&fbConfig, &fbAuth);
        firebase.reconnectWiFi(true);
        Serial.println("Firebase ready");
    } else {
        Serial.println("Firebase skipped (no Wi-Fi)");
    }

    Serial.println("System Started");
}

// ========================
// Functions
// ========================

void handleMotionAndTemperature() {
    bool motion = digitalRead(PIR_PIN) == HIGH;
    if (motion) {
        if (!motionDetected) Serial.println("Motion detected!");
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
        Serial.println("Motion timeout → Everything OFF");
    }

    if (motionDetected && !isnan(currentTemp)) {
        if (currentTemp < TEMP_LOWER && !heaterOn) {
            digitalWrite(HEATER_PIN, HIGH);
            digitalWrite(AC_PIN, LOW);
            heaterOn = true; acOn = false;
        } else if (currentTemp > TEMP_UPPER && !acOn) {
            digitalWrite(HEATER_PIN, LOW);
            digitalWrite(AC_PIN, HIGH);
            heaterOn = false; acOn = true;
        }
    } else {
        digitalWrite(HEATER_PIN, LOW);
        digitalWrite(AC_PIN, LOW);
        heaterOn = false; acOn = false;
    }
}

void readDHT_nonBlocking() {
    if (millis() - lastDHTRead >= DHT_INTERVAL) {
        lastDHTRead = millis();
        float t = dht.readTemperature();
        float h = dht.readHumidity();
        if (!isnan(t)) currentTemp = t;
        if (!isnan(h)) currentHum = h;
    }
}

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
            double rms = sqrt(totalSum / samples + 1e-10);
            double dB = 20 * log10(rms) + 94;
            smoothed_dB = SMOOTHING_ALPHA * dB + (1 - SMOOTHING_ALPHA) * smoothed_dB;
            currentNoiseDB = smoothed_dB;
        }
    }
}

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

String getTimestampString() {
    time_t now = time(nullptr);
    struct tm* timeinfo = localtime(&now);
    char buffer[30];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
    return String(buffer);
}

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
        Serial.println("Firebase updated");
    } else {
        Serial.print("Firebase failed: ");
        Serial.println(fbData.errorReason());
    }
}

// ========================
// Reconnect
// ========================
const unsigned long RECONNECT_INTERVAL = 300000; // 5 min
unsigned long lastReconnectAttempt = 0;

void reconnectIfNeeded() {
    if (millis() - lastReconnectAttempt < RECONNECT_INTERVAL) return;
    lastReconnectAttempt = millis();

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Reconnecting Wi-Fi...");
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

        unsigned long startAttempt = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 5000) {
            delay(500);
            Serial.print(".");
        }
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nWi-Fi reconnected");
        } else {
            Serial.println("\nWi-Fi reconnect failed");
            return;
        }
    }

    if (!Firebase.ready()) {
        Serial.println("Reconnecting Firebase...");
        firebase.begin(&fbConfig, &fbAuth);
        firebase.reconnectWiFi(true);

        if (Firebase.ready()) {
            Serial.println("Firebase reconnected");
        } else {
            Serial.println("Firebase reconnect failed");
        }
    }
}

// ========================
// Main Loop
// ========================
void loop() {
    readDHT_nonBlocking();
    readNoise_nonBlocking();
    handleMotionAndTemperature();
    printStatus();

    if (millis() - lastFirebaseUpdate >= FIREBASE_INTERVAL) {
        lastFirebaseUpdate = millis();
        pushToFirebase();
    }

    reconnectIfNeeded();
}

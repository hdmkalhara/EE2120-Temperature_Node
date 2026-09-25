/*
  ================================================================
       ESP32 DISTRIBUTED TEMPERATURE MONITORING NODE
       FINAL DUAL SENSOR-FUSION VERSION

       Method 1: Adaptive Confidence-Weighted Fusion
       Method 2: Simple 1D Kalman Fusion (Q = 0.03 degC^2/s)

       + SH1106 OLED
       + NEO-6M GPS
       + MicroSD logging
       + Wi-Fi + authenticated MQTT/TLS telemetry for Node-RED SCADA
       + CA-verified TLS on Mosquitto port 8883
  ================================================================

  BOARD:
    ESP32

  ------------------------------------------------
  PIN CONNECTIONS
  ------------------------------------------------

  LM35:
    VOUT  -> GPIO34
    VCC   -> 5V
    GND   -> GND

    IMPORTANT:
    The characterized LM35 setup used 5V.
    Do NOT power the LM35 from 3.3V.

  DS18B20:
    DQ    -> GPIO4
    VCC   -> 3.3V
    GND   -> GND
    4.7k resistor between DQ and 3.3V

  SH1106 OLED:
    SDA   -> GPIO21
    SCL   -> GPIO22
    VCC   -> 3.3V
    GND   -> GND

  NEO-6M GPS:
    GPS TX -> GPIO16  (ESP32 UART2 RX)
    GPS RX -> GPIO17  (ESP32 UART2 TX)
    GPS GND -> GND
    GPS VCC -> appropriate supply for your NEO-6M breakout board

  MicroSD:
    SCK  -> GPIO18
    MISO -> GPIO19
    MOSI -> GPIO23
    CS   -> GPIO5
    VCC  -> 5V for the regulator-type SD module used in this project
    GND  -> GND

  ------------------------------------------------
  REQUIRED LIBRARIES
  ------------------------------------------------
    OneWire
    DallasTemperature
    Adafruit GFX Library
    Adafruit SH110X
    TinyGPSPlus
    SD
    SPI
    WiFi (included with ESP32 core)
    PubSubClient
    ArduinoJson
*/

// ================================================================
// NETWORK SECURITY MODE
// ================================================================
// CURRENT LAB MIGRATION:
//   0 = authenticated MQTT on port 1884 (username/password + broker ACL)
// FINAL DEPLOYMENT:
//   1 = MQTT over TLS on port 8883 (requires CA certificate below)
#define MQTT_USE_TLS 1

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#if MQTT_USE_TLS
#include <WiFiClientSecure.h>
#include <time.h>
#endif

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <TinyGPSPlus.h>
#include <SPI.h>
#include <SD.h>

// ================================================================
// SERIAL OUTPUT MODE
// ================================================================
// 1 = Arduino IDE Serial Plotter: ONLY 4 clean live curves are sent
//     on USB Serial: LM35, DS18B20, Kalman, Adaptive.
// 0 = Original diagnostic / CSV Serial Monitor output.
//
// Keep this at 1 while using Tools -> Serial Plotter.
#define SERIAL_PLOTTER_MODE 1

#if SERIAL_PLOTTER_MODE
  // Suppress status/debug text because non-numeric text can disturb the
  // Arduino IDE Serial Plotter.
  #define DEBUG_PRINT(...)   do { } while (0)
  #define DEBUG_PRINTLN(...) do { } while (0)
#else
  #define DEBUG_PRINT(...)   Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#endif

// ================================================================
// WI-FI + MQTT / NODE-RED SCADA SETTINGS
// ================================================================
// ESP32 does not connect directly to Node-RED. It publishes to Mosquitto;
// Node-RED subscribes to the same MQTT topic.
//
// SECURITY:
// - Use the dedicated broker user "esp32_node01".
// - Broker ACL should grant this user WRITE access only to
//   ee2120/node01/sensors (least privilege).
// - Never use the Node-RED editor/admin password here.
// - The MQTT password below must match the CURRENT password stored for
//   esp32_node01 in Mosquitto.
// - Do not publish this sketch publicly with real Wi-Fi/MQTT credentials.

const char* WIFI_SSID     = "Senuda Pixel 7 Pro";
const char* WIFI_PASSWORD = "12345678";

// Laptop / Mosquitto broker IPv4 address on the same Wi-Fi network.
const char* MQTT_BROKER = "10.33.91.8";

#if MQTT_USE_TLS
const uint16_t MQTT_PORT = 8883;   // MQTT over TLS
#else
const uint16_t MQTT_PORT = 1884;   // Temporary non-TLS fallback listener
#endif

const char* MQTT_USERNAME = "esp32_node01";
const char* MQTT_PASSWORD = "senudaesp123";

const char* NODE_ID            = "node01";
const char* MQTT_TOPIC_SENSORS = "ee2120/node01/sensors";
const char* MQTT_CLIENT_PREFIX = "ESP32_NODE_01";

// Retry intervals are deliberately rate-limited to avoid reconnect storms.
const uint32_t WIFI_RETRY_INTERVAL_MS = 10000;
const uint32_t MQTT_RETRY_INTERVAL_MS = 5000;

#if MQTT_USE_TLS
// TLS certificate validation needs a sensible system clock.
// The ESP32 synchronizes UTC time from NTP before attempting MQTT/TLS.
const char* NTP_SERVER_1 = "pool.ntp.org";
const char* NTP_SERVER_2 = "time.nist.gov";
const time_t TLS_MIN_VALID_EPOCH = 1700000000;  // sanity threshold (> 2023)
bool tlsClockSyncStarted = false;
uint32_t lastTLSClockStatusMs = 0;
#endif

// PubSubClient's default packet buffer is too small for the full SCADA JSON.
const uint16_t MQTT_BUFFER_SIZE = 2048;

#if MQTT_USE_TLS
// Replace the text below with the CA certificate that signs your Mosquitto
// server certificate BEFORE setting MQTT_USE_TLS to 1.
static const char MQTT_CA_CERT[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFuTCCA6GgAwIBAgIUIjQjX7wbZy8UFpuS5ysJx51M/U8wDQYJKoZIhvcNAQEL
BQAwbDELMAkGA1UEBhMCTEsxITAfBgNVBAoMGFVuaXZlcnNpdHkgb2YgUGVyYWRl
bml5YTEYMBYGA1UECwwPRUUyMTIwIEdyb3VwIDI1MSAwHgYDVQQDDBdFRTIxMjAg
R3JvdXAgMjUgTVFUVCBDQTAeFw0yNjA5MjQyMTAxMzhaFw0zNjA5MjEyMTAxMzha
MGwxCzAJBgNVBAYTAkxLMSEwHwYDVQQKDBhVbml2ZXJzaXR5IG9mIFBlcmFkZW5p
eWExGDAWBgNVBAsMD0VFMjEyMCBHcm91cCAyNTEgMB4GA1UEAwwXRUUyMTIwIEdy
b3VwIDI1IE1RVFQgQ0EwggIiMA0GCSqGSIb3DQEBAQUAA4ICDwAwggIKAoICAQDr
2OeAMqCfATJJFqeyoutHi5dseejSEkQ9+hqn82AtkQm0+HBiYUr73JiXPg91LukR
NrVxhSrvQaINvHBSGiHFYP7TR8wJOYbk2DkX0FSfzE6934vHIISVT0shz5VipT0H
BAL6rsbwFhDmnoQiCDFA5GgzbKZTjHkIvMtlgKwgT/hPT3ZSA3R0I1Ap4//t+Oqc
FJ1/9hH42Stbk0LOzdhrcXVgnmXy1W1a9Fr7j2W69KC1T7nTO2cvEFeKvgey+jpO
O3RIWFXLVVfAU/if8S5qTWJxm83JUaDQFdAdQ5zjnRv8KdgAOLsvbzONhpnAL8DP
Gl9XKpfyNeCLwHd29Q6aEZrFg8eTTCQwzblJmbAqdCSrTiSS2hQLTV/oSIE287W6
9ij8bhWrmQ+pyYjNzEukGWwCEBDzInSf6ylaxNZLqqN0fuCH2REIIFpzrZosfxUO
HWMjHjJKWAhmV+R2WMahMcar7b9ObMtD5QczXQWlrd4UiWN+uwwcqQUR26Nu3lC1
k8qPBj+AJuZKI8TDp9FCBooABz4lbQapLIQ0oDWsbSnfWrqhAfwai5CZwws5cf4q
clChAzNoKfVnYO48MpmTeCeVpUeTCCPG58qeh110INmLC/lbpuxn1GTU3EVzwaoF
Zjosj2jLsVjDX8fIBRCleTIaKAWTcvnHCqC0sQEXgwIDAQABo1MwUTAdBgNVHQ4E
FgQUNw+miQ3ecFKwRNEUq9aiHmELiYEwHwYDVR0jBBgwFoAUNw+miQ3ecFKwRNEU
q9aiHmELiYEwDwYDVR0TAQH/BAUwAwEB/zANBgkqhkiG9w0BAQsFAAOCAgEAnS63
ZIHeaC8XHb7mTzeG7YmcXRJ+oNvGmoqmZUSAHH35qAgNSPGXSG12MX5x5cC78eEH
G+HRYypEVOQ9LfKTWauk73fGTVUIlKqcHSmoPr2Q2Ts5mIF4I6oT+9Fa1YwRBIZc
dApkuOuuO8Vwg26snKyfKoVJ6TlxjHtGRHOrWsISqesDb0VxrngPXmVXjQp8koET
gt3VYJ8dkIFuNYl0KWMRWaWdARAIGIZLlhOiBmMoiuHpxupAazBjM9uY3eNq1LV4
DRB3G4XLBI0BaTSQnIN74UmPI1y0fcg6E12aQBe0P5AnA8O2MVYRvTJVAYRjwPa7
165adtxb9HL2GF0QSKP+ywOJDVG2nmbIwkk+XXJFzevQiBlziy84tRqK9S85e6Fe
e8aCunwSfISk27vCZx833whSIF1Pd0HciJY/9FvEbCgV5c/U8VRssPlG+YqaYdtS
HeNeNbekJ+OS7meGK8YGGUnQP63BmG8ezxoHcrxlMwCQZFvU2XFPJj/qSh1KSZ98
vT5eUaP8hePEcaaRoI0Vh7xlOinfAyhSgUgKO5AkkVv+XvCMS+Ti5ClfJkgUhrGr
JDJ/lCXebbEUQQdcJEVVCqknDK5DLE4Md0dfLcn2G6A9YY5bWjx4KYuCRVwgE7zd
Ann4HzpUN6VHWHBOm/db9esmpc0IdawBUoRqBX8=
-----END CERTIFICATE-----
)EOF";

WiFiClientSecure mqttNetworkClient;
#else
WiFiClient mqttNetworkClient;
#endif

PubSubClient mqttClient(mqttNetworkClient);

uint32_t lastWiFiReconnectAttempt = 0;
uint32_t lastMQTTReconnectAttempt = 0;
char mqttClientId[48] = {0};

// ================================================================
// PIN DEFINITIONS
// ================================================================

const int LM35_PIN    = 34;
const int DS18B20_PIN = 4;

const int GPS_RX_PIN  = 16;   // ESP32 RX <- GPS TX
const int GPS_TX_PIN  = 17;   // ESP32 TX -> GPS RX

const int SD_CS_PIN   = 5;

const long GPS_BAUD_RATE = 9600;

// ================================================================
// OLED SETTINGS
// ================================================================

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_I2C_ADDR 0x3C

Adafruit_SH1106G display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  OLED_RESET
);

bool oledReady = false;

// ================================================================
// LM35 CALIBRATION
// Experimentally obtained:
//   T_LM35 = 0.081567 * ADC + 14.5187
// ================================================================

const float LM35_SLOPE_M     = 0.081567f;
const float LM35_INTERCEPT_C = 14.5187f;

// ================================================================
// FINAL SENSOR PARAMETERS
// ================================================================

// LM35 measured steady-state variance
const float R_LM35 = 0.294331f;          // deg C^2

// DS18B20 effective non-zero 12-bit uncertainty floor
const float R_DS18B20 = 0.02810f;        // deg C^2

// Measured waterproof-probe response time constants
const float TAU_LM35    = 13.95f;         // seconds
const float TAU_DS18B20 = 8.04f;          // seconds

// ================================================================
// FINAL SIMPLE KALMAN PARAMETER
// ================================================================

// Experimentally selected process-noise rate.
// Tuned by replaying the recorded dataset through multiple Q values.
const float KALMAN_Q_PER_SECOND = 0.03f; // deg C^2 / s

// Initial Kalman uncertainty
const float KALMAN_INITIAL_P = 1.0f;

// ================================================================
// SENSOR TIMING
// ================================================================

const uint32_t LM35_PERIOD_MS    = 100;    // 10 Hz
const uint32_t DS18B20_PERIOD_MS = 1000;   // ~1 new result/s
const uint32_t DS18B20_CONV_MS   = 750;    // 12-bit max conversion time

// ================================================================
// OLED / GPS TIMING
// ================================================================

const unsigned long SCREEN_SWITCH_MS = 3000;
const unsigned long OLED_UPDATE_MS   = 250;  // 4 Hz OLED refresh

unsigned long lastScreenSwitchTime = 0;
unsigned long lastOLEDUpdateTime   = 0;

bool showingTemperatureScreen = true;

// ================================================================
// DS18B20
// ================================================================

OneWire oneWireBus(DS18B20_PIN);
DallasTemperature ds18b20(&oneWireBus);

// ================================================================
// GPS
// ================================================================

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

// ================================================================
// MICROSD
// ================================================================

const char* LOG_FILE_PATH = "/datalog.csv";
bool sdCardReady = false;

// ================================================================
// SENSOR / ADAPTIVE FUSION STATE
// ================================================================

uint32_t lastLM35Time       = 0;
uint32_t lastDS18B20Request = 0;
uint32_t ds18b20RequestTime = 0;

bool ds18b20Converting = false;

int lastADC = 0;

float lm35Temp    = NAN;
float ds18b20Temp = NAN;

// Final outputs from the two fusion methods
float adaptiveTemp = NAN;
float kalmanTemp   = NAN;

float previousDS18B20Temp = NAN;

// Estimated temperature rate from DS18B20
float tempRate = 0.0f;

// Adaptive confidence weights
float weightLM35    = 0.0f;
float weightDS18B20 = 0.0f;

// Sensor validity
bool lm35Valid    = false;
bool ds18b20Valid = false;

// ================================================================
// SIMPLE 1D KALMAN STATE
// ================================================================

bool kalmanInitialized = false;

float kalmanP = KALMAN_INITIAL_P;

// Time of the previous Kalman predict/update operation
uint32_t lastKalmanTime = 0;

// Diagnostic gains / innovations
float kalmanGainLM35    = 0.0f;
float kalmanGainDS18B20 = 0.0f;

float innovationLM35    = 0.0f;
float innovationDS18B20 = 0.0f;

// ================================================================
// GPS VALUES
// ================================================================

double latestLatitude  = 0.0;
double latestLongitude = 0.0;

unsigned long latestSatellites = 0;

float latestHDOP = 99.99;

bool latestGPSFix = false;

int latestYear   = 0;
int latestMonth  = 0;
int latestDay    = 0;

int latestHour   = 0;
int latestMinute = 0;
int latestSecond = 0;

// ================================================================
// GPS FIX QUALITY THRESHOLDS
// ================================================================

const unsigned long MIN_SATELLITES_FOR_FIX = 4;
const float MAX_HDOP_FOR_FIX = 5.0;

// Serial Plotter helper
void printSerialPlotterCurves();

// ================================================================
// SETUP
// ================================================================

void setup() {

  Serial.begin(115200);
  delay(1000);

  DEBUG_PRINTLN();
  DEBUG_PRINTLN("==================================================");
  DEBUG_PRINTLN(" ESP32 DUAL SENSOR-FUSION TEMPERATURE NODE");
  DEBUG_PRINTLN(" Adaptive Confidence Fusion + Simple 1D Kalman");
  DEBUG_PRINTLN(" Kalman Q = 0.03 degC^2/s");
  DEBUG_PRINTLN(" GPS + OLED + MicroSD + Secure MQTT");
  DEBUG_PRINTLN("==================================================");

  // ------------------------------------------------
  // ESP32 ADC
  // ------------------------------------------------

  analogReadResolution(12);
  analogSetPinAttenuation(LM35_PIN, ADC_11db);

  DEBUG_PRINTLN("ADC initialized.");

  // ------------------------------------------------
  // DS18B20
  // ------------------------------------------------

  ds18b20.begin();

  int deviceCount = ds18b20.getDeviceCount();

  DEBUG_PRINT("DS18B20 devices detected: ");
  DEBUG_PRINTLN(deviceCount);

  if (deviceCount == 0) {
    DEBUG_PRINTLN("WARNING: No DS18B20 detected. Check wiring.");
  }

  // Final deployment: 12-bit, asynchronous conversion
  ds18b20.setResolution(12);
  ds18b20.setWaitForConversion(false);

  ds18b20.requestTemperatures();

  ds18b20Converting = true;
  ds18b20RequestTime = millis();
  lastDS18B20Request = millis();

  // ------------------------------------------------
  // GPS
  // ------------------------------------------------

  gpsSerial.begin(
    GPS_BAUD_RATE,
    SERIAL_8N1,
    GPS_RX_PIN,
    GPS_TX_PIN
  );

  DEBUG_PRINTLN("NEO-6M GPS initialized.");

  // ------------------------------------------------
  // OLED
  // ------------------------------------------------

  Wire.begin(21, 22);

  oledReady = display.begin(OLED_I2C_ADDR, true);

  if (!oledReady) {
    // Do not freeze the sensing / network node just because the local display
    // is unavailable. The fault is reported to SCADA as oled:false.
    DEBUG_PRINTLN("WARNING: OLED initialization failed. Continuing without OLED.");
  } else {
    display.clearDisplay();
    display.setTextColor(SH110X_WHITE);
    display.setTextSize(1);

    display.setCursor(12, 18);
    display.println("Dual Fusion Node");

    display.setCursor(18, 34);
    display.println("Initializing...");

    display.display();
    delay(1200);
  }

  // ------------------------------------------------
  // MicroSD
  // ------------------------------------------------

  sdCardReady = initSDCard();

  // ------------------------------------------------
  // Wi-Fi + MQTT security / Node-RED telemetry
  // ------------------------------------------------

  initializeNetwork();

  // ------------------------------------------------
  // Serial CSV header
  // ------------------------------------------------

  DEBUG_PRINTLN();
  DEBUG_PRINTLN(
    "Time_ms,ADC,LM35_C,DS18B20_C,"
    "Adaptive_C,Kalman_C,"
    "Weight_LM35,Weight_DS18B20,"
    "Rate_C_per_s,"
    "K_LM35,K_DS18B20,Kalman_P,"
    "Innovation_LM35,Innovation_DS18B20,"
    "Fusion_Status,"
    "Latitude,Longitude,Satellites,HDOP,GPS_Fix,GPS_Date,GPS_Time"
  );

  DEBUG_PRINTLN();
  DEBUG_PRINT("MQTT telemetry topic: ");
  DEBUG_PRINTLN(MQTT_TOPIC_SENSORS);
  DEBUG_PRINT("MQTT authentication: ENABLED as user ");
  DEBUG_PRINTLN(MQTT_USERNAME);
#if MQTT_USE_TLS
  DEBUG_PRINTLN("MQTT transport: TLS ENABLED");
#else
  DEBUG_PRINTLN("MQTT transport: AUTHENTICATED, TLS NOT YET ENABLED");
#endif
  DEBUG_PRINTLN("System ready.");
}

// ================================================================
// MAIN LOOP
// ================================================================

void loop() {

  // GPS must be parsed continuously
  feedGPSParser();

  uint32_t now = millis();

  // Maintain Wi-Fi and authenticated MQTT without blocking sensor timing.
  maintainNetwork(now);

  // ------------------------------------------------
  // LM35 at 10 Hz
  // ------------------------------------------------

  if (now - lastLM35Time >= LM35_PERIOD_MS) {

    lastLM35Time = now;

    lm35Temp = readLM35();
    lm35Valid = validLM35(lm35Temp);

    // Adaptive method uses the latest available values
    updateAdaptiveFusion();

    // IMPORTANT:
    // Kalman gets ONE update for this genuinely new LM35 sample.
    if (lm35Valid) {
      kalmanMeasurementUpdate(
        lm35Temp,
        R_LM35,
        now,
        true
      );
    }

    // Arduino IDE Serial Plotter output at the LM35 sampling rate (10 Hz).
    // DS18B20 updates about once per second; its latest valid value is held
    // between conversions, so all four curves remain visible continuously.
    printSerialPlotterCurves();
  }

  // ------------------------------------------------
  // Finish a genuinely new DS18B20 conversion
  // ------------------------------------------------

  if (
    ds18b20Converting &&
    now - ds18b20RequestTime >= DS18B20_CONV_MS
  ) {

    float newDS18B20Temp = ds18b20.getTempCByIndex(0);

    ds18b20Converting = false;

    ds18b20Valid = validDS18B20(newDS18B20Temp);

    if (ds18b20Valid) {

      ds18b20Temp = newDS18B20Temp;

      // --------------------------------------------
      // Temperature-rate estimate for adaptive fusion
      // --------------------------------------------

      if (isfinite(previousDS18B20Temp)) {

        float rawRate =
          ds18b20Temp - previousDS18B20Temp;

        const float RATE_ALPHA = 0.5f;

        tempRate =
          RATE_ALPHA * rawRate +
          (1.0f - RATE_ALPHA) * tempRate;
      }

      previousDS18B20Temp = ds18b20Temp;

      // IMPORTANT:
      // Kalman uses the DS18B20 only HERE, once per new conversion.
      kalmanMeasurementUpdate(
        ds18b20Temp,
        R_DS18B20,
        now,
        false
      );
    }

    // Update adaptive output after receiving the new DS value
    updateAdaptiveFusion();

    // ----------------------------------------------
    // GPS + Serial + SD logging once per new DS result
    // ----------------------------------------------

    updateGPSValues();

    String logLine = buildLogLine(now);

    DEBUG_PRINTLN(logLine);

    if (sdCardReady) {
      logToSDCard(logLine);
    }

    // Publish one complete telemetry packet for each new DS18B20 result.
    // The dashboard's "fused" field is the adaptive confidence-weighted
    // temperature; Kalman data is also included as extra diagnostics.
    publishTelemetry(now);
  }

  // ------------------------------------------------
  // Start next DS18B20 conversion
  // ------------------------------------------------

  if (
    !ds18b20Converting &&
    now - lastDS18B20Request >= DS18B20_PERIOD_MS
  ) {

    ds18b20.requestTemperatures();

    ds18b20Converting = true;

    ds18b20RequestTime = now;
    lastDS18B20Request = now;
  }

  // ------------------------------------------------
  // Switch OLED screen
  // ------------------------------------------------

  if (now - lastScreenSwitchTime >= SCREEN_SWITCH_MS) {

    lastScreenSwitchTime = now;
    showingTemperatureScreen = !showingTemperatureScreen;
  }

  // ------------------------------------------------
  // OLED refresh limited to 4 Hz
  // ------------------------------------------------

  if (oledReady && now - lastOLEDUpdateTime >= OLED_UPDATE_MS) {

    lastOLEDUpdateTime = now;

    if (showingTemperatureScreen) {
      updateTempScreen();
    } else {
      updateGPSScreen();
    }
  }
}

// ================================================================
// WI-FI + AUTHENTICATED MQTT / NODE-RED SCADA
// ================================================================

void initializeNetwork() {

  // Do not store Wi-Fi credentials in ESP32 flash using the legacy
  // persistent WiFi config mechanism. Credentials are supplied by this
  // sketch at runtime instead.
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);

  // A stable station MAC-based MQTT client ID prevents accidental duplicate
  // client IDs when more than one physical node is deployed.
  uint64_t chipId = ESP.getEfuseMac();
  uint32_t shortId = (uint32_t)(chipId & 0xFFFFFFULL);

  snprintf(
    mqttClientId,
    sizeof(mqttClientId),
    "%s_%06lX",
    MQTT_CLIENT_PREFIX,
    (unsigned long)shortId
  );

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setBufferSize(MQTT_BUFFER_SIZE);
  mqttClient.setKeepAlive(30);
  mqttClient.setSocketTimeout(5);

#if MQTT_USE_TLS
  mqttNetworkClient.setCACert(MQTT_CA_CERT);
#endif

  DEBUG_PRINTLN();
  DEBUG_PRINTLN("---------------- NETWORK ----------------");
  DEBUG_PRINT("Wi-Fi SSID: ");
  DEBUG_PRINTLN(WIFI_SSID);
  DEBUG_PRINT("MQTT broker: ");
  DEBUG_PRINT(MQTT_BROKER);
  DEBUG_PRINT(":");
  DEBUG_PRINTLN(MQTT_PORT);
  DEBUG_PRINT("MQTT client ID: ");
  DEBUG_PRINTLN(mqttClientId);
  DEBUG_PRINTLN("MQTT password: [HIDDEN]");
  DEBUG_PRINTLN("-----------------------------------------");

  startWiFiConnection();
}

void startWiFiConnection() {

  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  DEBUG_PRINT("Connecting to Wi-Fi: ");
  DEBUG_PRINTLN(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWiFiReconnectAttempt = millis();
}

void maintainNetwork(uint32_t now) {

  // ------------------------------------------------
  // Wi-Fi recovery
  // ------------------------------------------------
  if (WiFi.status() != WL_CONNECTED) {

    if (now - lastWiFiReconnectAttempt >= WIFI_RETRY_INTERVAL_MS) {
      DEBUG_PRINTLN("Wi-Fi disconnected. Reconnecting...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      lastWiFiReconnectAttempt = now;
    }

    return;
  }

  // ------------------------------------------------
  // MQTT recovery
  // ------------------------------------------------
  if (!mqttClient.connected()) {

    if (now - lastMQTTReconnectAttempt >= MQTT_RETRY_INTERVAL_MS) {
      lastMQTTReconnectAttempt = now;
      connectMQTT();
    }

    return;
  }

  // Required by PubSubClient to maintain keep-alive traffic.
  mqttClient.loop();
}

#if MQTT_USE_TLS
bool ensureTLSClockReady() {

  time_t currentTime = time(nullptr);

  if (currentTime >= TLS_MIN_VALID_EPOCH) {
    return true;
  }

  if (!tlsClockSyncStarted) {
    DEBUG_PRINTLN("TLS: Synchronizing system clock with NTP...");
    configTime(0, 0, NTP_SERVER_1, NTP_SERVER_2);
    tlsClockSyncStarted = true;
    lastTLSClockStatusMs = millis();
    return false;
  }

  if (millis() - lastTLSClockStatusMs >= 5000) {
    lastTLSClockStatusMs = millis();
    DEBUG_PRINTLN("TLS: Waiting for valid system time before certificate verification...");
  }

  return false;
}
#endif

bool connectMQTT() {

  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

#if MQTT_USE_TLS
  if (!ensureTLSClockReady()) {
    return false;
  }
#endif

  DEBUG_PRINT("Connecting to authenticated MQTT broker ");
  DEBUG_PRINT(MQTT_BROKER);
  DEBUG_PRINT(":");
  DEBUG_PRINT(MQTT_PORT);
  DEBUG_PRINT(" as ");
  DEBUG_PRINT(MQTT_USERNAME);
  DEBUG_PRINT(" ... ");

  // SECURITY:
  // MQTT username/password authentication runs inside the TLS tunnel.
  // The broker certificate is verified against MQTT_CA_CERT before MQTT
  // authentication succeeds. The password is never printed to Serial.
  bool connected = mqttClient.connect(
    mqttClientId,
    MQTT_USERNAME,
    MQTT_PASSWORD
  );

  if (connected) {
    DEBUG_PRINTLN("CONNECTED");
#if MQTT_USE_TLS
    DEBUG_PRINTLN("MQTT security: TLS + CA verification + username/password ENABLED");
#endif
    DEBUG_PRINT("ESP32 IP: ");
    DEBUG_PRINTLN(WiFi.localIP());
    DEBUG_PRINT("Wi-Fi RSSI: ");
    DEBUG_PRINT(WiFi.RSSI());
    DEBUG_PRINTLN(" dBm");
    return true;
  }

  DEBUG_PRINT("FAILED, MQTT state = ");
  DEBUG_PRINTLN(mqttClient.state());
  return false;
}

bool publishTelemetry(uint32_t now) {

  if (WiFi.status() != WL_CONNECTED || !mqttClient.connected()) {
    return false;
  }

  // The field names below intentionally match the current Node-RED SCADA
  // core. Additional Kalman fields are included; older dashboards simply
  // ignore fields they do not use.
#if ARDUINOJSON_VERSION_MAJOR >= 7
  JsonDocument doc;
#else
  StaticJsonDocument<1536> doc;
#endif

  doc["node_id"] = NODE_ID;

  if (lm35Valid && isfinite(lm35Temp)) {
    doc["lm35"] = lm35Temp;
  } else {
    doc["lm35"] = nullptr;
  }

  doc["lm35_adc"] = lastADC;
  doc["lm35_valid"] = lm35Valid;

  if (ds18b20Valid && isfinite(ds18b20Temp)) {
    doc["ds18b20"] = ds18b20Temp;
  } else {
    doc["ds18b20"] = nullptr;
  }

  doc["ds18b20_valid"] = ds18b20Valid;

  // Existing dashboard uses "fused" as its authoritative final value.
  if (isfinite(adaptiveTemp)) {
    doc["fused"] = adaptiveTemp;
  } else {
    doc["fused"] = nullptr;
  }

  doc["fusion_source"] = "ESP32_ADAPTIVE";
  doc["weight_lm35"] = weightLM35;
  doc["weight_ds18b20"] = weightDS18B20;
  doc["temp_rate"] = tempRate;
  doc["fusion_status"] = fusionStatus();

  if (lm35Valid && ds18b20Valid && isfinite(lm35Temp) && isfinite(ds18b20Temp)) {
    doc["sensor_difference"] = fabsf(lm35Temp - ds18b20Temp);
  } else {
    doc["sensor_difference"] = nullptr;
  }

  // Extra Method-2 / Kalman diagnostics.
  if (kalmanInitialized && isfinite(kalmanTemp)) {
    doc["kalman"] = kalmanTemp;
  } else {
    doc["kalman"] = nullptr;
  }
  doc["kalman_p"] = kalmanP;
  doc["kalman_gain_lm35"] = kalmanGainLM35;
  doc["kalman_gain_ds18b20"] = kalmanGainDS18B20;
  doc["innovation_lm35"] = innovationLM35;
  doc["innovation_ds18b20"] = innovationDS18B20;

  // GPS fields expected by the Node-RED dashboard.
  if (latestGPSFix) {
    doc["latitude"] = latestLatitude;
    doc["longitude"] = latestLongitude;
  } else {
    doc["latitude"] = nullptr;
    doc["longitude"] = nullptr;
  }

  doc["satellites"] = latestSatellites;
  doc["hdop"] = latestHDOP;
  doc["gps_fix"] = latestGPSFix;

  char dateBuffer[11] = "INVALID";
  if (gps.date.isValid()) {
    snprintf(
      dateBuffer,
      sizeof(dateBuffer),
      "%04d-%02d-%02d",
      latestYear,
      latestMonth,
      latestDay
    );
  }
  doc["date"] = dateBuffer;

  char timeBuffer[9] = "INVALID";
  if (gps.time.isValid()) {
    snprintf(
      timeBuffer,
      sizeof(timeBuffer),
      "%02d:%02d:%02d",
      latestHour,
      latestMinute,
      latestSecond
    );
  }
  doc["time"] = timeBuffer;

  // Communication / hardware health expected by SCADA.
  doc["wifi"] = true;
  doc["mqtt"] = true;
  doc["rssi"] = WiFi.RSSI();
  doc["sd"] = sdCardReady;
  doc["oled"] = oledReady;
  doc["uptime_ms"] = now;

  // Explicit security diagnostics. These are informational and do not
  // replace broker-side authentication/ACL enforcement.
  doc["mqtt_authenticated"] = true;
  doc["mqtt_tls"] = (MQTT_USE_TLS != 0);

  char payload[1800];
  size_t payloadLength = serializeJson(doc, payload, sizeof(payload));

  if (payloadLength == 0 || payloadLength >= sizeof(payload)) {
    DEBUG_PRINTLN("ERROR: MQTT JSON serialization failed or overflowed.");
    return false;
  }

  // Retain=false prevents stale measurements from being mistaken for live
  // measurements after a broker/client restart.
  bool ok = mqttClient.publish(
    MQTT_TOPIC_SENSORS,
    reinterpret_cast<const uint8_t*>(payload),
    payloadLength,
    false
  );

  if (!ok) {
    DEBUG_PRINTLN("WARNING: MQTT telemetry publish failed.");
  }

  return ok;
}

// ================================================================
// GPS PARSER
// ================================================================

void feedGPSParser() {

  while (gpsSerial.available() > 0) {

    char c = gpsSerial.read();
    gps.encode(c);
  }
}

// ================================================================
// LM35
// ================================================================

float readLM35() {

  lastADC = analogRead(LM35_PIN);

  return
    LM35_SLOPE_M * ((float)lastADC)
    + LM35_INTERCEPT_C;
}

bool validLM35(float t) {

  if (!isfinite(t)) {
    return false;
  }

  // Broad electrical sanity limits.
  // Calibration was formally characterized over a narrower range.
  if (t < -10.0f || t > 100.0f) {
    return false;
  }

  return true;
}

// ================================================================
// DS18B20
// ================================================================

bool validDS18B20(float t) {

  if (!isfinite(t)) {
    return false;
  }

  if (t <= -126.0f) {
    return false;
  }

  if (t < -10.0f || t > 100.0f) {
    return false;
  }

  return true;
}

// ================================================================
// METHOD 1: ADAPTIVE CONFIDENCE-WEIGHTED FUSION
// ================================================================

void updateAdaptiveFusion() {

  // ------------------------------------------------
  // Both sensors valid
  // ------------------------------------------------

  if (lm35Valid && ds18b20Valid) {

    float dynamicLM35 =
      TAU_LM35 * fabsf(tempRate);

    float dynamicDS18B20 =
      TAU_DS18B20 * fabsf(tempRate);

    float R_LM35_eff =
      R_LM35 +
      dynamicLM35 * dynamicLM35;

    float R_DS18B20_eff =
      R_DS18B20 +
      dynamicDS18B20 * dynamicDS18B20;

    float confidenceLM35 =
      1.0f / R_LM35_eff;

    float confidenceDS18B20 =
      1.0f / R_DS18B20_eff;

    float confidenceSum =
      confidenceLM35 + confidenceDS18B20;

    weightLM35 =
      confidenceLM35 / confidenceSum;

    weightDS18B20 =
      confidenceDS18B20 / confidenceSum;

    adaptiveTemp =
      weightLM35 * lm35Temp +
      weightDS18B20 * ds18b20Temp;
  }

  // ------------------------------------------------
  // LM35 only
  // ------------------------------------------------

  else if (lm35Valid && !ds18b20Valid) {

    weightLM35    = 1.0f;
    weightDS18B20 = 0.0f;

    adaptiveTemp = lm35Temp;
  }

  // ------------------------------------------------
  // DS18B20 only
  // ------------------------------------------------

  else if (!lm35Valid && ds18b20Valid) {

    weightLM35    = 0.0f;
    weightDS18B20 = 1.0f;

    adaptiveTemp = ds18b20Temp;
  }

  // ------------------------------------------------
  // Both invalid
  // ------------------------------------------------

  else {

    weightLM35    = 0.0f;
    weightDS18B20 = 0.0f;

    // Hold previous adaptive output.
  }
}

// ================================================================
// METHOD 2: SIMPLE 1D KALMAN FUSION
// ================================================================

void kalmanMeasurementUpdate(
  float measurement,
  float measurementVariance,
  uint32_t now,
  bool isLM35Measurement
) {

  if (!isfinite(measurement)) {
    return;
  }

  // ------------------------------------------------
  // Initialize from first valid sensor measurement
  // ------------------------------------------------

  if (!kalmanInitialized) {

    kalmanTemp = measurement;
    kalmanP = measurementVariance;

    kalmanInitialized = true;
    lastKalmanTime = now;

    if (isLM35Measurement) {
      kalmanGainLM35 = 1.0f;
      innovationLM35 = 0.0f;
    } else {
      kalmanGainDS18B20 = 1.0f;
      innovationDS18B20 = 0.0f;
    }

    return;
  }

  // ------------------------------------------------
  // Prediction
  // ------------------------------------------------

  float dt =
    (now - lastKalmanTime) / 1000.0f;

  // Protect against unexpected very large timing gaps
  if (dt < 0.0f) {
    dt = 0.0f;
  }

  if (dt > 5.0f) {
    dt = 5.0f;
  }

  kalmanP +=
    KALMAN_Q_PER_SECOND * dt;

  lastKalmanTime = now;

  // ------------------------------------------------
  // Measurement update
  // ------------------------------------------------

  float innovation =
    measurement - kalmanTemp;

  float gain =
    kalmanP /
    (kalmanP + measurementVariance);

  kalmanTemp =
    kalmanTemp +
    gain * innovation;

  kalmanP =
    (1.0f - gain) * kalmanP;

  // Keep diagnostic values separately
  if (isLM35Measurement) {

    kalmanGainLM35 = gain;
    innovationLM35 = innovation;

  } else {

    kalmanGainDS18B20 = gain;
    innovationDS18B20 = innovation;
  }
}

// ================================================================
// FUSION STATUS
// ================================================================

const char* fusionStatus() {

  if (lm35Valid && ds18b20Valid) {
    return "FUSION_OK";
  }

  if (lm35Valid && !ds18b20Valid) {
    return "LM35_ONLY";
  }

  if (!lm35Valid && ds18b20Valid) {
    return "DS18B20_ONLY";
  }

  return "BOTH_FAULT";
}

// ================================================================
// GPS VALUES
// ================================================================

void updateGPSValues() {

  if (gps.satellites.isValid()) {
    latestSatellites = gps.satellites.value();
  } else {
    latestSatellites = 0;
  }

  if (gps.hdop.isValid()) {
    latestHDOP = gps.hdop.hdop();
  } else {
    latestHDOP = 99.99;
  }

  bool fixIsRecent =
    gps.location.isValid() &&
    gps.location.age() < 5000;

  bool fixMeetsQuality =
    (latestSatellites >= MIN_SATELLITES_FOR_FIX) &&
    (latestHDOP <= MAX_HDOP_FOR_FIX);

  if (fixIsRecent && fixMeetsQuality) {

    latestGPSFix = true;

    latestLatitude  = gps.location.lat();
    latestLongitude = gps.location.lng();

  } else {

    latestGPSFix = false;
  }

  if (gps.date.isValid()) {

    latestYear  = gps.date.year();
    latestMonth = gps.date.month();
    latestDay   = gps.date.day();
  }

  if (gps.time.isValid()) {

    latestHour   = gps.time.hour();
    latestMinute = gps.time.minute();
    latestSecond = gps.time.second();
  }
}

// ================================================================
// MICROSD
// ================================================================

bool initSDCard() {

  DEBUG_PRINTLN("Initializing MicroSD card...");

  if (!SD.begin(SD_CS_PIN)) {

    DEBUG_PRINTLN("ERROR: MicroSD card mount failed.");
    DEBUG_PRINTLN("Check wiring and FAT32 format.");

    return false;
  }

  bool fileAlreadyExists =
    SD.exists(LOG_FILE_PATH);

  File logFile =
    SD.open(LOG_FILE_PATH, FILE_APPEND);

  if (!logFile) {

    DEBUG_PRINTLN("ERROR: Could not open datalog.csv");
    return false;
  }

  if (!fileAlreadyExists) {

    logFile.println(
      "Time_ms,"
      "ADC,"
      "LM35_C,"
      "DS18B20_C,"
      "Adaptive_C,"
      "Kalman_C,"
      "Weight_LM35,"
      "Weight_DS18B20,"
      "Rate_C_per_s,"
      "K_LM35,"
      "K_DS18B20,"
      "Kalman_P,"
      "Innovation_LM35,"
      "Innovation_DS18B20,"
      "Fusion_Status,"
      "Latitude,"
      "Longitude,"
      "Satellites,"
      "HDOP,"
      "GPS_Fix,"
      "GPS_Date,"
      "GPS_Time"
    );
  }

  logFile.close();

  DEBUG_PRINTLN("MicroSD card ready.");
  DEBUG_PRINT("Logging to: ");
  DEBUG_PRINTLN(LOG_FILE_PATH);

  return true;
}

// ================================================================
// ARDUINO IDE SERIAL PLOTTER: 4 LIVE TEMPERATURE CURVES
// ================================================================

void printSerialPlotterCurves() {

#if SERIAL_PLOTTER_MODE

  // Wait until all four quantities contain valid numeric values.
  // This prevents NaN / startup text from disturbing Serial Plotter.
  if (
    !lm35Valid ||
    !ds18b20Valid ||
    !kalmanInitialized ||
    !isfinite(lm35Temp) ||
    !isfinite(ds18b20Temp) ||
    !isfinite(kalmanTemp) ||
    !isfinite(adaptiveTemp)
  ) {
    return;
  }

  // Arduino Serial Plotter recognizes "label:value" series separated by tabs.
  // One complete line = one sample containing all four curves.
  Serial.print("LM35:");
  Serial.print(lm35Temp, 3);
  Serial.print('\t');

  Serial.print("DS18B20:");
  Serial.print(ds18b20Temp, 3);
  Serial.print('\t');

  Serial.print("Kalman:");
  Serial.print(kalmanTemp, 3);
  Serial.print('\t');

  Serial.print("Adaptive:");
  Serial.println(adaptiveTemp, 3);

#endif
}

// ================================================================
// BUILD CSV LOG LINE
// ================================================================

String buildLogLine(unsigned long timestampMs) {

  String line = "";

  line += String(timestampMs);
  line += ",";

  line += String(lastADC);
  line += ",";

  // LM35
  if (lm35Valid) {
    line += String(lm35Temp, 3);
  } else {
    line += "NaN";
  }
  line += ",";

  // DS18B20
  if (ds18b20Valid) {
    line += String(ds18b20Temp, 3);
  } else {
    line += "NaN";
  }
  line += ",";

  // Adaptive fusion
  if (isfinite(adaptiveTemp)) {
    line += String(adaptiveTemp, 3);
  } else {
    line += "NaN";
  }
  line += ",";

  // Kalman fusion
  if (kalmanInitialized && isfinite(kalmanTemp)) {
    line += String(kalmanTemp, 3);
  } else {
    line += "NaN";
  }
  line += ",";

  // Adaptive weights
  line += String(weightLM35, 4);
  line += ",";
  line += String(weightDS18B20, 4);
  line += ",";

  // Rate
  line += String(tempRate, 3);
  line += ",";

  // Kalman diagnostics
  line += String(kalmanGainLM35, 4);
  line += ",";
  line += String(kalmanGainDS18B20, 4);
  line += ",";
  line += String(kalmanP, 6);
  line += ",";
  line += String(innovationLM35, 3);
  line += ",";
  line += String(innovationDS18B20, 3);
  line += ",";

  // Status
  line += fusionStatus();
  line += ",";

  // GPS position
  if (latestGPSFix) {

    line += String(latestLatitude, 6);
    line += ",";
    line += String(latestLongitude, 6);

  } else {

    line += "NO_FIX";
    line += ",";
    line += "NO_FIX";
  }

  line += ",";

  line += String(latestSatellites);
  line += ",";

  line += String(latestHDOP, 2);
  line += ",";

  if (latestGPSFix) {
    line += "FIX";
  } else {
    line += "NO_FIX";
  }

  line += ",";

  if (gps.date.isValid()) {

    char dateBuffer[11];

    sprintf(
      dateBuffer,
      "%04d-%02d-%02d",
      latestYear,
      latestMonth,
      latestDay
    );

    line += dateBuffer;

  } else {

    line += "INVALID";
  }

  line += ",";

  if (gps.time.isValid()) {

    char timeBuffer[9];

    sprintf(
      timeBuffer,
      "%02d:%02d:%02d",
      latestHour,
      latestMinute,
      latestSecond
    );

    line += timeBuffer;

  } else {

    line += "INVALID";
  }

  return line;
}

// ================================================================
// WRITE TO SD
// ================================================================

void logToSDCard(const String& line) {

  File logFile =
    SD.open(LOG_FILE_PATH, FILE_APPEND);

  if (logFile) {

    logFile.println(line);
    logFile.close();

  } else {

    DEBUG_PRINTLN("ERROR: Could not write to MicroSD.");
  }
}

// ================================================================
// OLED: TEMPERATURE / BOTH FUSION OUTPUTS
// ================================================================

void updateTempScreen() {

  if (!oledReady) return;

  display.clearDisplay();
  display.setTextSize(1);

  // Title
  display.setCursor(22, 2);
  display.print("FUSION OUTPUTS");

  display.drawFastHLine(
    0, 12, SCREEN_WIDTH, SH110X_WHITE
  );

  // ------------------------------------------------
  // Adaptive output
  // ------------------------------------------------

  display.setCursor(0, 17);
  display.print("Adaptive:");

  display.setCursor(66, 17);

  if (isfinite(adaptiveTemp)) {
    display.print(adaptiveTemp, 1);
    display.print(" C");
  } else {
    display.print("ERR");
  }

  // ------------------------------------------------
  // Kalman output
  // ------------------------------------------------

  display.setCursor(0, 29);
  display.print("Kalman:");

  display.setCursor(66, 29);

  if (kalmanInitialized && isfinite(kalmanTemp)) {
    display.print(kalmanTemp, 1);
    display.print(" C");
  } else {
    display.print("WAIT");
  }

  // Divider
  display.drawFastHLine(
    0, 40, SCREEN_WIDTH, SH110X_WHITE
  );

  // ------------------------------------------------
  // Raw sensor values
  // ------------------------------------------------

  display.setCursor(0, 44);
  display.print("LM:");

  if (lm35Valid) {
    display.print(lm35Temp, 1);
  } else {
    display.print("ERR");
  }

  display.setCursor(66, 44);
  display.print("DS:");

  if (ds18b20Valid) {
    display.print(ds18b20Temp, 1);
  } else {
    display.print("ERR");
  }

  // Short health / SD indication
  display.setCursor(0, 55);

  if (lm35Valid && ds18b20Valid) {
    display.print("Sensors:OK");
  } else {
    display.print("Sensors:ERR");
  }

  display.setCursor(88, 55);
  display.print("SD:");

  if (sdCardReady) {
    display.print("OK");
  } else {
    display.print("X");
  }

  display.display();
}

// ================================================================
// OLED: GPS
// ================================================================

void updateGPSScreen() {

  if (!oledReady) return;

  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(28, 2);
  display.print("GPS STATUS");

  display.drawFastHLine(
    0, 12, SCREEN_WIDTH, SH110X_WHITE
  );

  display.setCursor(0, 17);
  display.print("Status: ");

  if (latestGPSFix) {
    display.print("FIX");
  } else {
    display.print("SEARCHING");
  }

  display.setCursor(75, 17);
  display.print("S:");
  display.print(latestSatellites);

  display.setCursor(0, 30);
  display.print("Lat:");

  if (latestGPSFix) {
    display.print(latestLatitude, 5);
  } else {
    display.print("---");
  }

  display.setCursor(0, 42);
  display.print("Lng:");

  if (latestGPSFix) {
    display.print(latestLongitude, 5);
  } else {
    display.print("---");
  }

  display.setCursor(0, 54);
  display.print("SD:");

  if (sdCardReady) {
    display.print("OK");
  } else {
    display.print("FAIL");
  }

  display.display();
}

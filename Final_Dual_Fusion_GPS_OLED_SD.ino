/*
  ================================================================
       ESP32 DISTRIBUTED TEMPERATURE MONITORING NODE
       FINAL DUAL SENSOR-FUSION VERSION

       Method 1: Adaptive Confidence-Weighted Fusion
       Method 2: Simple 1D Kalman Fusion (Q = 0.03 degC^2/s)

       + SH1106 OLED
       + NEO-6M GPS
       + MicroSD logging
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
*/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <TinyGPSPlus.h>
#include <SPI.h>
#include <SD.h>

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

// ================================================================
// SETUP
// ================================================================

void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("==================================================");
  Serial.println(" ESP32 DUAL SENSOR-FUSION TEMPERATURE NODE");
  Serial.println(" Adaptive Confidence Fusion + Simple 1D Kalman");
  Serial.println(" Kalman Q = 0.03 degC^2/s");
  Serial.println(" GPS + OLED + MicroSD");
  Serial.println("==================================================");

  // ------------------------------------------------
  // ESP32 ADC
  // ------------------------------------------------

  analogReadResolution(12);
  analogSetPinAttenuation(LM35_PIN, ADC_11db);

  Serial.println("ADC initialized.");

  // ------------------------------------------------
  // DS18B20
  // ------------------------------------------------

  ds18b20.begin();

  int deviceCount = ds18b20.getDeviceCount();

  Serial.print("DS18B20 devices detected: ");
  Serial.println(deviceCount);

  if (deviceCount == 0) {
    Serial.println("WARNING: No DS18B20 detected. Check wiring.");
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

  Serial.println("NEO-6M GPS initialized.");

  // ------------------------------------------------
  // OLED
  // ------------------------------------------------

  Wire.begin(21, 22);

  if (!display.begin(OLED_I2C_ADDR, true)) {

    Serial.println("ERROR: OLED initialization failed!");

    while (true) {
      delay(500);
    }
  }

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);

  display.setCursor(12, 18);
  display.println("Dual Fusion Node");

  display.setCursor(18, 34);
  display.println("Initializing...");

  display.display();

  delay(1200);

  // ------------------------------------------------
  // MicroSD
  // ------------------------------------------------

  sdCardReady = initSDCard();

  // ------------------------------------------------
  // Serial CSV header
  // ------------------------------------------------

  Serial.println();
  Serial.println(
    "Time_ms,ADC,LM35_C,DS18B20_C,"
    "Adaptive_C,Kalman_C,"
    "Weight_LM35,Weight_DS18B20,"
    "Rate_C_per_s,"
    "K_LM35,K_DS18B20,Kalman_P,"
    "Innovation_LM35,Innovation_DS18B20,"
    "Fusion_Status,"
    "Latitude,Longitude,Satellites,HDOP,GPS_Fix,GPS_Date,GPS_Time"
  );

  Serial.println("System ready.");
}

// ================================================================
// MAIN LOOP
// ================================================================

void loop() {

  // GPS must be parsed continuously
  feedGPSParser();

  uint32_t now = millis();

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

    Serial.println(logLine);

    if (sdCardReady) {
      logToSDCard(logLine);
    }
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

  if (now - lastOLEDUpdateTime >= OLED_UPDATE_MS) {

    lastOLEDUpdateTime = now;

    if (showingTemperatureScreen) {
      updateTempScreen();
    } else {
      updateGPSScreen();
    }
  }
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

  Serial.println("Initializing MicroSD card...");

  if (!SD.begin(SD_CS_PIN)) {

    Serial.println("ERROR: MicroSD card mount failed.");
    Serial.println("Check wiring and FAT32 format.");

    return false;
  }

  bool fileAlreadyExists =
    SD.exists(LOG_FILE_PATH);

  File logFile =
    SD.open(LOG_FILE_PATH, FILE_APPEND);

  if (!logFile) {

    Serial.println("ERROR: Could not open datalog.csv");
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

  Serial.println("MicroSD card ready.");
  Serial.print("Logging to: ");
  Serial.println(LOG_FILE_PATH);

  return true;
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

    Serial.println("ERROR: Could not write to MicroSD.");
  }
}

// ================================================================
// OLED: TEMPERATURE / BOTH FUSION OUTPUTS
// ================================================================

void updateTempScreen() {

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

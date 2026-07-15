#include <Arduino.h>
#include <ESPmDNS.h>
#include <LovyanGFX.hpp>
#include <Preferences.h>
#include <SdFat.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <Wire.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iterator>
#include <time.h>

namespace config {

constexpr int kBacklightPin = 21;
constexpr int kBootButtonPin = 0;
constexpr int kI2cSdaPin = 27;
constexpr int kI2cSclPin = 22;
constexpr uint8_t kScd4xAddress = 0x62;
constexpr uint32_t kReadingIntervalMs = 10000;
constexpr uint32_t kCo2StabilizationMs = 30000;
constexpr uint32_t kEnvironmentalStabilizationMs = 180000;
constexpr size_t kShortHistorySize = 60;
constexpr size_t kLongHistorySize = 720;
constexpr uint8_t kReadingsPerMinute = 6;
constexpr uint8_t kMinutesPerSave = 10;
constexpr int kSdCsPin = 5;
constexpr int kSdMosiPin = 23;
constexpr int kSdMisoPin = 19;
constexpr int kSdSckPin = 18;
constexpr uint32_t kSdRetryIntervalMs = 60000;
constexpr uint32_t kSensorWarmupPollMs = 2000;
constexpr uint8_t kSensorMissesBeforeRestart = 6;
constexpr char kUnsyncedLogFileName[] = "co2-unsynced.csv";
constexpr char kMonthlyLogPrefix[] = "co2-";
constexpr size_t kWebHistoryPoints = 600;
constexpr char kConfigPortalName[] = "CO2-Dashboard-Setup";
constexpr char kDefaultTimezone[] = "GMT0BST,M3.5.0/1,M10.5.0";
constexpr char kNtpServer1[] = "pool.ntp.org";
constexpr char kNtpServer2[] = "time.cloudflare.com";
constexpr size_t kLongTermPlotPoints = 280;
constexpr size_t kSdReadBlockSize = 512;
constexpr int kSwipeThreshold = 60;
constexpr uint32_t kBootButtonDebounceMs = 30;
constexpr uint32_t kBootButtonLongPressMs = 900;

}  // namespace config

class CydDisplay : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341 panel_;
  lgfx::Bus_SPI bus_;
  lgfx::Touch_XPT2046 touch_;

 public:
  CydDisplay() {
    {
      auto cfg = bus_.config();
      cfg.spi_host = VSPI_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 27000000;
      cfg.freq_read = 16000000;
      cfg.spi_3wire = true;
      cfg.use_lock = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = 14;
      cfg.pin_mosi = 13;
      cfg.pin_miso = 12;
      cfg.pin_dc = 2;
      bus_.config(cfg);
      panel_.setBus(&bus_);
    }

    {
      auto cfg = panel_.config();
      cfg.pin_cs = 15;
      cfg.pin_rst = -1;
      cfg.pin_busy = -1;
      cfg.panel_width = 320;
      cfg.panel_height = 240;
      cfg.memory_width = 320;
      cfg.memory_height = 240;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      // This panel is mounted in a reflected 320x240 native orientation.
      cfg.offset_rotation = 6;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits = 1;
      cfg.readable = true;
      cfg.invert = false;
      cfg.rgb_order = false;
      cfg.dlen_16bit = false;
      cfg.bus_shared = true;
      panel_.config(cfg);
    }

    {
      auto cfg = touch_.config();
      cfg.spi_host = HSPI_HOST;
      cfg.freq = 2500000;
      cfg.pin_sclk = 25;
      cfg.pin_mosi = 32;
      cfg.pin_miso = 39;
      cfg.pin_cs = 33;
      cfg.pin_int = 36;
      cfg.x_min = 300;
      cfg.x_max = 3900;
      cfg.y_min = 300;
      cfg.y_max = 3900;
      cfg.bus_shared = false;
      cfg.offset_rotation = 1;
      touch_.config(cfg);
      panel_.setTouch(&touch_);
    }

    setPanel(&panel_);
  }

  // Some CYD boards ship with the ILI9341 wired for RGB order instead of
  // BGR, which swaps red and blue on screen. Rather than requiring a
  // rebuild, this flips the panel's MADCTL color-order bit at runtime.
  void setColorOrderRgb(bool rgb_order) {
    auto cfg = panel_.config();
    if (cfg.rgb_order == rgb_order) return;
    cfg.rgb_order = rgb_order;
    panel_.config(cfg);
    setRotation(getRotation());
  }

  bool colorOrderRgb() const { return panel_.config().rgb_order; }
};

class Scd4x {
 public:
  bool begin() {
    Wire.begin(config::kI2cSdaPin, config::kI2cSclPin);
    Wire.setClock(100000);

    // Stop a measurement that may have survived a soft reset.
    sendCommand(0x3F86);
    delay(500);
    sendCommand(0x3646);
    delay(30);
    return sendCommand(0x21B1);
  }

  bool factoryReset() {
    if (!stopMeasurement() || !sendCommand(0x3632)) return false;
    delay(1200);
    if (!sendCommand(0x3646)) return false;
    delay(30);
    return sendCommand(0x21B1);
  }

  bool readIfReady(uint16_t& co2, float& temperature, float& humidity) {
    uint16_t ready = 0;
    if (!readWords(0xE4B8, &ready, 1, 1) || (ready & 0x07FF) == 0) {
      return false;
    }

    uint16_t words[3] = {};
    if (!readWords(0xEC05, words, 3, 1)) {
      return false;
    }

    co2 = words[0];
    temperature = -45.0f + 175.0f * static_cast<float>(words[1]) / 65535.0f;
    humidity = 100.0f * static_cast<float>(words[2]) / 65535.0f;
    return co2 != 0;
  }

  bool readSettings(bool& asc_enabled, float& temperature_offset,
                    uint16_t& altitude) {
    if (!stopMeasurement()) return false;
    uint16_t asc = 0;
    uint16_t raw_offset = 0;
    const bool ok = readWords(0x2313, &asc, 1, 1) &&
                    readWords(0x2318, &raw_offset, 1, 1) &&
                    readWords(0x2322, &altitude, 1, 1);
    const bool restarted = sendCommand(0x21B1);
    if (!ok || !restarted) return false;
    asc_enabled = asc != 0;
    temperature_offset =
        175.0f * static_cast<float>(raw_offset) / 65535.0f;
    return true;
  }

  bool setAutomaticSelfCalibration(bool enabled) {
    return updateSetting(0x2416, enabled ? 1 : 0);
  }

  bool setTemperatureOffset(float offset) {
    const uint16_t raw = static_cast<uint16_t>(
        std::round(offset * 65535.0f / 175.0f));
    return updateSetting(0x241D, raw);
  }

  bool setAltitude(uint16_t altitude) {
    return updateSetting(0x2427, altitude);
  }

  bool saveSettings(bool asc_enabled, float temperature_offset,
                    uint16_t altitude) {
    const uint16_t raw_offset = static_cast<uint16_t>(
        std::round(temperature_offset * 65535.0f / 175.0f));
    if (!stopMeasurement()) return false;

    bool written = sendCommandWithWord(0x2416, asc_enabled ? 1 : 0);
    delay(1);
    written = written && sendCommandWithWord(0x241D, raw_offset);
    delay(1);
    written = written && sendCommandWithWord(0x2427, altitude);
    delay(1);
    const bool persisted = written && sendCommand(0x3615);
    if (persisted) delay(800);
    const bool restarted = sendCommand(0x21B1);
    return written && persisted && restarted;
  }

  bool performForcedRecalibration(uint16_t target, int16_t& correction) {
    if (!stopMeasurement() || !sendCommandWithWord(0x362F, target)) {
      sendCommand(0x21B1);
      return false;
    }
    delay(400);
    uint16_t raw_correction = 0;
    const bool ok = receiveWords(&raw_correction, 1);
    const bool restarted = sendCommand(0x21B1);
    if (!ok || !restarted || raw_correction == 0xFFFF) return false;
    correction = static_cast<int16_t>(raw_correction - 0x8000);
    return true;
  }

 private:
  static uint8_t crc8(const uint8_t* data, size_t length) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < length; ++i) {
      crc ^= data[i];
      for (uint8_t bit = 0; bit < 8; ++bit) {
        crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x31)
                           : static_cast<uint8_t>(crc << 1);
      }
    }
    return crc;
  }

  static bool sendCommand(uint16_t command) {
    Wire.beginTransmission(config::kScd4xAddress);
    Wire.write(static_cast<uint8_t>(command >> 8));
    Wire.write(static_cast<uint8_t>(command));
    return Wire.endTransmission() == 0;
  }

  static bool sendCommandWithWord(uint16_t command, uint16_t value) {
    const uint8_t bytes[2] = {static_cast<uint8_t>(value >> 8),
                              static_cast<uint8_t>(value)};
    Wire.beginTransmission(config::kScd4xAddress);
    Wire.write(static_cast<uint8_t>(command >> 8));
    Wire.write(static_cast<uint8_t>(command));
    Wire.write(bytes, sizeof(bytes));
    Wire.write(crc8(bytes, sizeof(bytes)));
    return Wire.endTransmission() == 0;
  }

  static bool receiveWords(uint16_t* words, size_t count) {
    const size_t bytes_needed = count * 3;
    if (Wire.requestFrom(config::kScd4xAddress,
                         static_cast<uint8_t>(bytes_needed)) != bytes_needed) {
      return false;
    }
    for (size_t i = 0; i < count; ++i) {
      uint8_t bytes[2] = {static_cast<uint8_t>(Wire.read()),
                          static_cast<uint8_t>(Wire.read())};
      const uint8_t received_crc = static_cast<uint8_t>(Wire.read());
      if (crc8(bytes, 2) != received_crc) return false;
      words[i] = static_cast<uint16_t>(bytes[0] << 8) | bytes[1];
    }
    return true;
  }

  static bool stopMeasurement() {
    if (!sendCommand(0x3F86)) return false;
    delay(500);
    return true;
  }

  static bool updateSetting(uint16_t command, uint16_t value) {
    if (!stopMeasurement()) return false;
    const bool written = sendCommandWithWord(command, value);
    delay(1);
    const bool persisted = written && sendCommand(0x3615);
    if (persisted) delay(800);
    const bool restarted = sendCommand(0x21B1);
    return written && persisted && restarted;
  }

  static bool readWords(uint16_t command, uint16_t* words, size_t count,
                        uint16_t delay_ms) {
    if (!sendCommand(command)) {
      return false;
    }
    delay(delay_ms);

    const size_t bytes_needed = count * 3;
    if (Wire.requestFrom(config::kScd4xAddress,
                         static_cast<uint8_t>(bytes_needed)) != bytes_needed) {
      return false;
    }

    for (size_t i = 0; i < count; ++i) {
      uint8_t bytes[2] = {static_cast<uint8_t>(Wire.read()),
                          static_cast<uint8_t>(Wire.read())};
      const uint8_t received_crc = static_cast<uint8_t>(Wire.read());
      if (crc8(bytes, 2) != received_crc) {
        return false;
      }
      words[i] = static_cast<uint16_t>(bytes[0] << 8) | bytes[1];
    }
    return true;
  }
};

namespace {

CydDisplay display;
CydDisplay& canvas = display;
Scd4x sensor;
Preferences preferences;
WiFiManager wifi_manager;
WebServer web_server(80);
SoftSpiDriver<config::kSdMisoPin, config::kSdMosiPin, config::kSdSckPin>
    sd_spi;
SdFs sd;

struct Reading {
  uint16_t co2 = 0;
  float temperature = NAN;
  float humidity = NAN;
  bool valid = false;
};

Reading current;
uint16_t short_history[config::kShortHistorySize] = {};
float short_temperature_history[config::kShortHistorySize] = {};
float short_humidity_history[config::kShortHistorySize] = {};
size_t short_history_count = 0;
size_t short_history_head = 0;
uint16_t long_history[config::kLongHistorySize] = {};
float long_temperature_history[config::kLongHistorySize] = {};
float long_humidity_history[config::kLongHistorySize] = {};
size_t long_history_count = 0;
size_t long_history_head = 0;
uint32_t minute_sum = 0;
float minute_temperature_sum = 0.0f;
float minute_humidity_sum = 0.0f;
uint8_t minute_sample_count = 0;
uint8_t minute_environment_sample_count = 0;
uint8_t minutes_since_save = 0;
unsigned long last_reading_ms = 0;
unsigned long last_sensor_attempt_ms = 0;
unsigned long co2_ready_ms = 0;
unsigned long environmental_logging_ready_ms = 0;
unsigned long last_sd_attempt_ms = 0;
int last_clock_minute = -1;
uint32_t boot_id = 0;
bool sensor_ready = false;
bool co2_reading_valid = false;
bool environmental_reading_valid = false;
uint8_t consecutive_sensor_misses = 0;
uint8_t sensor_recovery_attempts = 0;
bool sd_ready = false;
bool time_configured = false;
bool web_server_started = false;
bool touch_was_down = false;
uint16_t touch_start_x = 0;
uint16_t touch_start_y = 0;
uint16_t last_touch_x = 0;
uint16_t last_touch_y = 0;
char timezone_rule[65] = {};
WiFiManagerParameter timezone_parameter(
    "timezone", "Timezone (POSIX rule)", config::kDefaultTimezone, 64,
    "placeholder=\"GMT0BST,M3.5.0/1,M10.5.0\"");

enum class DisplayPage : uint8_t {
  kDashboard,
  kLongTerm,
  kCalibration,
};

enum class TrendRange : uint8_t {
  kTenMinutes,
  kOneHour,
  kSixHours,
  kTwelveHours,
  kCount,
};

enum class LongTermRange : uint8_t {
  kDay,
  kWeek,
  kMonth,
  kCount,
};

enum class DisplayMetric : uint8_t {
  kCo2,
  kTemperature,
  kHumidity,
  kCount,
};

enum class CalibrationItem : uint8_t {
  kAutomaticCalibration,
  kTemperatureOffset,
  kReference,
  kForcedCalibration,
  kExit,
  kCount,
};

DisplayPage display_page = DisplayPage::kDashboard;
TrendRange trend_range = TrendRange::kTenMinutes;
LongTermRange long_term_range = LongTermRange::kDay;
DisplayMetric display_metric = DisplayMetric::kCo2;
uint16_t long_term_plot[config::kLongTermPlotPoints] = {};
size_t long_term_plot_count = 0;
uint16_t long_term_min = 0;
uint16_t long_term_max = 0;
uint16_t long_term_average = 0;
char long_term_last_timestamp[24] = {};
bool long_term_dirty = true;
uint32_t web_history_co2_sum[config::kWebHistoryPoints] = {};
float web_history_temperature_sum[config::kWebHistoryPoints] = {};
float web_history_humidity_sum[config::kWebHistoryPoints] = {};
uint16_t web_history_bucket_count[config::kWebHistoryPoints] = {};
bool calibration_loaded = false;
bool calibration_busy = false;
bool asc_enabled = true;
float calibration_temperature_offset = 0.0f;
uint16_t calibration_altitude = 0;
uint16_t calibration_target = 420;
bool calibration_armed = false;
CalibrationItem calibration_item =
    CalibrationItem::kAutomaticCalibration;
char calibration_status[48] = "Swipe here from long-term history";
bool boot_button_raw_down = false;
bool boot_button_down = false;
bool boot_button_long_action = false;
unsigned long boot_button_changed_ms = 0;
unsigned long boot_button_pressed_ms = 0;

struct PersistedHistory {
  uint32_t magic;
  uint16_t version;
  uint16_t count;
  uint16_t head;
  uint16_t samples[config::kLongHistorySize];
};

constexpr uint32_t kHistoryMagic = 0x434F3248;
constexpr uint16_t kHistoryVersion = 1;

constexpr uint32_t kBackground = 0x07111F;
constexpr uint32_t kPanel = 0x0E2034;
constexpr uint32_t kPanelLight = 0x15304B;
constexpr uint32_t kWhite = 0xF4F7FB;
constexpr uint32_t kMuted = 0x7794AC;
constexpr uint32_t kGreen = 0x42D392;
constexpr uint32_t kAmber = 0xFFB547;
constexpr uint32_t kRed = 0xFF5D73;
constexpr uint32_t kBlue = 0x50B8FF;

void configureNetworkTime() {
  if (WiFi.status() != WL_CONNECTED) return;
  configTzTime(timezone_rule, config::kNtpServer1, config::kNtpServer2);
  time_configured = true;
  Serial.printf("Wi-Fi connected: %s, timezone: %s\n",
                WiFi.SSID().c_str(), timezone_rule);
}

void savePortalSettings() {
  const char* configured_timezone = timezone_parameter.getValue();
  if (configured_timezone == nullptr || configured_timezone[0] == '\0') {
    return;
  }
  snprintf(timezone_rule, sizeof(timezone_rule), "%s", configured_timezone);
  preferences.putString("timezone", timezone_rule);
  time_configured = false;
  configureNetworkTime();
}

void setupWifiAndTime() {
  const String saved_timezone =
      preferences.getString("timezone", config::kDefaultTimezone);
  snprintf(timezone_rule, sizeof(timezone_rule), "%s",
           saved_timezone.c_str());

  timezone_parameter.setValue(timezone_rule, sizeof(timezone_rule) - 1);
  wifi_manager.addParameter(&timezone_parameter);
  wifi_manager.setTitle("CO2 Dashboard Setup");
  wifi_manager.setHostname("co2-dashboard");
  wifi_manager.setConnectTimeout(20);
  // Keep the non-blocking portal available until Wi-Fi is configured.
  wifi_manager.setConfigPortalTimeout(0);
  wifi_manager.setConfigPortalBlocking(false);
  wifi_manager.setWiFiAutoReconnect(true);
  wifi_manager.setSaveParamsCallback(savePortalSettings);

  Serial.printf("Saved Wi-Fi credentials: %s\n",
                wifi_manager.getWiFiIsSaved() ? "YES" : "NO");
  Serial.printf("Connecting to Wi-Fi or keeping portal \"%s\" active\n",
                config::kConfigPortalName);
  wifi_manager.autoConnect(config::kConfigPortalName);
  configureNetworkTime();
}

void serviceWifiAndTime() {
  wifi_manager.process();
  if (!time_configured && WiFi.status() == WL_CONNECTED) {
    configureNetworkTime();
  } else if (time_configured && WiFi.status() != WL_CONNECTED) {
    time_configured = false;
  }
}

void resetStabilizationWindows() {
  const unsigned long now = millis();
  co2_ready_ms = now + config::kCo2StabilizationMs;
  environmental_logging_ready_ms =
      now + config::kEnvironmentalStabilizationMs;
  co2_reading_valid = false;
  environmental_reading_valid = false;
  minute_sum = 0;
  minute_temperature_sum = 0.0f;
  minute_humidity_sum = 0.0f;
  minute_sample_count = 0;
  minute_environment_sample_count = 0;
}

void startSensor() {
  last_sensor_attempt_ms = millis();
  const bool factory_reset = sensor_recovery_attempts >= 3;
  sensor_ready =
      factory_reset ? sensor.factoryReset() : sensor.begin();
  if (factory_reset && sensor_ready) {
    preferences.remove("cal_defaults");
    sensor_recovery_attempts = 0;
    Serial.println("SCD4x factory reset completed");
  }
  consecutive_sensor_misses = 0;
  last_reading_ms = millis();
  resetStabilizationWindows();
  Serial.printf("SCD4x init: %s\n", sensor_ready ? "OK" : "FAILED");
}

void initializeCalibrationDefaults() {
  if (!sensor_ready) return;

  if (preferences.isKey("cal_defaults")) {
    calibration_busy = true;
    calibration_loaded =
        sensor.readSettings(asc_enabled, calibration_temperature_offset,
                            calibration_altitude);
    calibration_busy = false;
    last_reading_ms = millis();
    resetStabilizationWindows();
    Serial.printf("SCD4x calibration settings cache: %s\n",
                  calibration_loaded ? "OK" : "FAILED");
    return;
  }

  if (sensor.saveSettings(asc_enabled, 0.0f, 0)) {
    preferences.putBool("cal_defaults", true);
    calibration_temperature_offset = 0.0f;
    calibration_altitude = 0;
    calibration_loaded = true;
    last_reading_ms = millis();
    resetStabilizationWindows();
    Serial.println("SCD4x temperature offset and altitude set to zero");
  } else {
    Serial.println("Could not initialize SCD4x calibration defaults");
  }
}

bool formatLocalTimestamp(char* output, size_t output_size) {
  struct tm local_time = {};
  if (!getLocalTime(&local_time, 0)) return false;
  return strftime(output, output_size, "%Y-%m-%dT%H:%M:%S",
                  &local_time) > 0;
}

bool activeLogFileName(char* output, size_t output_size) {
  struct tm local_time = {};
  if (!getLocalTime(&local_time, 0)) {
    snprintf(output, output_size, "%s", config::kUnsyncedLogFileName);
    return false;
  }
  return strftime(output, output_size, "co2-%Y-%m.csv", &local_time) > 0;
}

bool ensureLogFile(const char* file_name) {
  FsFile log_file;
  if (!log_file.open(file_name, O_WRONLY | O_CREAT | O_APPEND)) {
    return false;
  }
  if (log_file.fileSize() == 0) {
    log_file.println(
        "boot_id,uptime_seconds,co2_ppm,temperature_c,humidity_percent,"
        "local_timestamp");
  }
  return log_file.close();
}

bool beginSdCard() {
  last_sd_attempt_ms = millis();
  sd_ready = sd.begin(
      SdSpiConfig(config::kSdCsPin, DEDICATED_SPI, SD_SCK_MHZ(0), &sd_spi));
  if (!sd_ready) {
    Serial.println("SD card unavailable; logging will retry in one minute");
    return false;
  }

  char file_name[20] = {};
  activeLogFileName(file_name, sizeof(file_name));
  if (!ensureLogFile(file_name)) {
    Serial.printf("Could not open %s\n", file_name);
    sd_ready = false;
    return false;
  }
  Serial.printf("SD logging ready: %s\n", file_name);
  return true;
}

void logMinuteReading(uint16_t co2, float temperature, float humidity) {
  if (!sd_ready &&
      millis() - last_sd_attempt_ms >= config::kSdRetryIntervalMs) {
    beginSdCard();
  }
  if (!sd_ready) return;

  char file_name[20] = {};
  activeLogFileName(file_name, sizeof(file_name));
  FsFile log_file;
  if (!log_file.open(file_name,
                     O_WRONLY | O_CREAT | O_APPEND)) {
    Serial.println("SD log open failed; logging paused");
    sd_ready = false;
    return;
  }
  if (log_file.fileSize() == 0) {
    log_file.println(
        "boot_id,uptime_seconds,co2_ppm,temperature_c,humidity_percent,"
        "local_timestamp");
  }

  log_file.print(boot_id);
  log_file.print(',');
  log_file.print(millis() / 1000UL);
  log_file.print(',');
  log_file.print(co2);
  log_file.print(',');
  log_file.print(temperature, 2);
  log_file.print(',');
  log_file.print(humidity, 2);
  log_file.print(',');
  char timestamp[24] = {};
  if (formatLocalTimestamp(timestamp, sizeof(timestamp))) {
    log_file.println(timestamp);
  } else {
    log_file.println("unsynced");
  }
  if (!log_file.close()) {
    Serial.println("SD log write failed; logging paused");
    sd_ready = false;
    return;
  }
  long_term_dirty = true;
  Serial.printf("Saved one-minute average to %s\n", file_name);
}

uint32_t qualityColor(uint16_t co2) {
  if (co2 < 800) return kGreen;
  if (co2 < 1200) return kAmber;
  return kRed;
}

const char* qualityLabel(uint16_t co2) {
  if (co2 < 800) return "FRESH";
  if (co2 < 1200) return "VENTILATE SOON";
  return "VENTILATE NOW";
}

const char* metricTitle(DisplayMetric metric) {
  switch (metric) {
    case DisplayMetric::kCo2:
      return "CO2";
    case DisplayMetric::kTemperature:
      return "TEMPERATURE";
    case DisplayMetric::kHumidity:
      return "HUMIDITY";
    default:
      return "";
  }
}

const char* metricTrendTitle(DisplayMetric metric) {
  switch (metric) {
    case DisplayMetric::kCo2:
      return "CO2 TREND";
    case DisplayMetric::kTemperature:
      return "TEMP TREND";
    case DisplayMetric::kHumidity:
      return "RH TREND";
    default:
      return "";
  }
}

const char* metricUnit(DisplayMetric metric) {
  switch (metric) {
    case DisplayMetric::kCo2:
      return "ppm";
    case DisplayMetric::kTemperature:
      return "C";
    case DisplayMetric::kHumidity:
      return "%";
    default:
      return "";
  }
}

uint32_t metricColor(DisplayMetric metric, float value) {
  switch (metric) {
    case DisplayMetric::kCo2:
      return qualityColor(static_cast<uint16_t>(value));
    case DisplayMetric::kTemperature:
      return kAmber;
    case DisplayMetric::kHumidity:
      return kBlue;
    default:
      return kWhite;
  }
}

bool metricReady(DisplayMetric metric) {
  switch (metric) {
    case DisplayMetric::kCo2:
      return co2_reading_valid;
    case DisplayMetric::kTemperature:
    case DisplayMetric::kHumidity:
      return environmental_reading_valid;
    default:
      return false;
  }
}

float currentMetricValue(DisplayMetric metric) {
  switch (metric) {
    case DisplayMetric::kCo2:
      return current.co2;
    case DisplayMetric::kTemperature:
      return current.temperature;
    case DisplayMetric::kHumidity:
      return current.humidity;
    default:
      return NAN;
  }
}

uint16_t scaledMetricValue(DisplayMetric metric, float value) {
  if (!isfinite(value)) return 0;
  if (metric == DisplayMetric::kCo2) {
    return static_cast<uint16_t>(std::max(0.0f, std::round(value)));
  }
  return static_cast<uint16_t>(std::max(0.0f, std::round(value * 10.0f)));
}

float unscaledMetricValue(DisplayMetric metric, uint16_t value) {
  return metric == DisplayMetric::kCo2 ? static_cast<float>(value)
                                       : static_cast<float>(value) / 10.0f;
}

void formatMetricValue(DisplayMetric metric, float value, char* output,
                       size_t output_size) {
  if (!isfinite(value)) {
    snprintf(output, output_size, metric == DisplayMetric::kCo2 ? "---"
                                                                : "--.-");
    return;
  }
  if (metric == DisplayMetric::kCo2) {
    snprintf(output, output_size, "%u",
             static_cast<unsigned>(std::round(value)));
  } else {
    snprintf(output, output_size, "%.1f", value);
  }
}

DisplayMetric leftBottomMetric() {
  return display_metric == DisplayMetric::kTemperature ? DisplayMetric::kCo2
                                                       : DisplayMetric::kTemperature;
}

DisplayMetric rightBottomMetric() {
  return display_metric == DisplayMetric::kHumidity ? DisplayMetric::kCo2
                                                    : DisplayMetric::kHumidity;
}

uint32_t currentMetricColor(DisplayMetric metric) {
  return metricColor(metric, metricReady(metric) ? currentMetricValue(metric)
                                                : 0.0f);
}

void saveHistory() {
  PersistedHistory stored = {};
  stored.magic = kHistoryMagic;
  stored.version = kHistoryVersion;
  stored.count = static_cast<uint16_t>(long_history_count);
  stored.head = static_cast<uint16_t>(long_history_head);
  std::copy(std::begin(long_history), std::end(long_history),
            std::begin(stored.samples));
  preferences.putBytes("history", &stored, sizeof(stored));
}

void loadHistory() {
  std::fill(std::begin(long_temperature_history),
            std::end(long_temperature_history), NAN);
  std::fill(std::begin(long_humidity_history),
            std::end(long_humidity_history), NAN);
  PersistedHistory stored = {};
  if (preferences.getBytesLength("history") != sizeof(stored) ||
      preferences.getBytes("history", &stored, sizeof(stored)) !=
          sizeof(stored) ||
      stored.magic != kHistoryMagic || stored.version != kHistoryVersion ||
      stored.count > config::kLongHistorySize ||
      stored.head >= config::kLongHistorySize) {
    return;
  }

  long_history_count = stored.count;
  long_history_head = stored.head;
  std::copy(std::begin(stored.samples), std::end(stored.samples),
            std::begin(long_history));
  Serial.printf("Restored %u minutes of CO2 history\n",
                static_cast<unsigned>(long_history_count));
}

void addShortHistory(uint16_t co2, float temperature, float humidity) {
  short_history[short_history_head] = co2;
  short_temperature_history[short_history_head] = temperature;
  short_humidity_history[short_history_head] = humidity;
  short_history_head =
      (short_history_head + 1) % config::kShortHistorySize;
  short_history_count =
      std::min(short_history_count + 1, config::kShortHistorySize);
}

void addLongHistory(uint16_t co2, float temperature, float humidity) {
  long_history[long_history_head] = co2;
  long_temperature_history[long_history_head] = temperature;
  long_humidity_history[long_history_head] = humidity;
  long_history_head = (long_history_head + 1) % config::kLongHistorySize;
  long_history_count =
      std::min(long_history_count + 1, config::kLongHistorySize);
  logMinuteReading(co2, temperature, humidity);
  if (++minutes_since_save >= config::kMinutesPerSave) {
    minutes_since_save = 0;
    saveHistory();
  }
}

void addHistory(uint16_t co2, float temperature, float humidity) {
  addShortHistory(co2, environmental_reading_valid ? temperature : NAN,
                  environmental_reading_valid ? humidity : NAN);
  minute_sum += co2;
  if (static_cast<int32_t>(millis() - environmental_logging_ready_ms) >= 0) {
    minute_temperature_sum += temperature;
    minute_humidity_sum += humidity;
    ++minute_environment_sample_count;
  }
  if (++minute_sample_count >= config::kReadingsPerMinute) {
    if (minute_environment_sample_count > 0) {
      addLongHistory(
          static_cast<uint16_t>(minute_sum / config::kReadingsPerMinute),
          minute_temperature_sum / minute_environment_sample_count,
          minute_humidity_sum / minute_environment_sample_count);
    } else {
      Serial.println(
          "Skipped CSV minute while temperature and humidity stabilized");
    }
    minute_sum = 0;
    minute_temperature_sum = 0.0f;
    minute_humidity_sum = 0.0f;
    minute_sample_count = 0;
    minute_environment_sample_count = 0;
  }
}

uint16_t ringValueAt(const uint16_t* values, size_t capacity, size_t count,
                     size_t head, size_t index) {
  const size_t oldest =
      (head + capacity - count) % capacity;
  return values[(oldest + index) % capacity];
}

float ringValueAt(const float* values, size_t capacity, size_t count,
                  size_t head, size_t index) {
  const size_t oldest = (head + capacity - count) % capacity;
  return values[(oldest + index) % capacity];
}

const char* trendRangeLabel() {
  switch (trend_range) {
    case TrendRange::kTenMinutes:
      return "10 min";
    case TrendRange::kOneHour:
      return "1 hour";
    case TrendRange::kSixHours:
      return "6 hours";
    case TrendRange::kTwelveHours:
      return "12 hours";
    default:
      return "";
  }
}

const char kDashboardHtml[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="theme-color" content="#07111f">
<title>CO2 Dashboard</title>
<style>
:root{color-scheme:dark;--bg:#07111f;--panel:#0e2034;--line:#15304b;--text:#f4f7fb;--muted:#7794ac;--green:#42d392;--amber:#ffb547;--red:#ff5d73;--blue:#50b8ff}
*{box-sizing:border-box}
body{margin:0;background:radial-gradient(circle at top,#102943 0,var(--bg) 48%);color:var(--text);font:16px/1.45 system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;min-height:100vh}
main{width:min(1120px,calc(100% - 28px));margin:auto;padding:28px 0 40px}
header{display:flex;align-items:flex-end;justify-content:space-between;gap:18px;margin-bottom:18px}
h1{font-size:clamp(1.2rem,3vw,1.65rem);letter-spacing:.08em;margin:0}
.header-actions{display:flex;align-items:center;gap:12px}
.connection{color:var(--muted);font-size:.88rem;text-align:right}
.gear{width:40px;height:40px;display:grid;place-items:center;padding:0;border-radius:50%;color:var(--muted)}
.gear:hover,.gear:focus-visible{color:var(--blue);border-color:var(--blue)}
.gear svg{width:20px;height:20px}
.grid{display:grid;grid-template-columns:minmax(250px,.78fr) minmax(0,1.5fr);gap:16px}
.card{background:rgba(14,32,52,.94);border:1px solid var(--line);border-radius:18px;box-shadow:0 18px 45px rgba(0,0,0,.2)}
.reading{padding:24px;display:flex;min-height:340px;flex-direction:column;justify-content:space-between}
.reading-value{font-size:clamp(4.5rem,12vw,8rem);font-weight:750;line-height:.85;letter-spacing:-.07em;margin:24px 0 8px}
.eyebrow{color:var(--muted);font-size:.78rem;font-weight:700;letter-spacing:.14em;text-transform:uppercase}
.unit{color:var(--muted);font-size:1.05rem;font-weight:650;letter-spacing:.08em}
.quality{font-weight:750;letter-spacing:.08em;margin-top:16px}
.live{display:flex;align-items:center;gap:8px;color:var(--muted);font-size:.88rem}
.dot{width:9px;height:9px;border-radius:50%;background:var(--amber);box-shadow:0 0 16px currentColor}
.trend{padding:20px;min-height:340px}
.trend-head{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-bottom:14px}
.ranges{display:flex;flex-wrap:wrap;gap:6px;justify-content:flex-end}
button{border:1px solid var(--line);border-radius:999px;background:#0a192a;color:var(--muted);padding:7px 11px;font:inherit;font-size:.78rem;cursor:pointer}
button.active{color:var(--blue);border-color:var(--blue);background:rgba(80,184,255,.09)}
.chart-wrap{height:260px;position:relative}
canvas{width:100%;height:100%;display:block}
.metrics{display:grid;grid-template-columns:repeat(3,1fr);gap:16px;margin-top:16px}
.metric{padding:20px;cursor:pointer;transition:border-color .15s,transform .15s}
.metric:hover{transform:translateY(-1px)}
.metric.active{border-color:var(--blue)}
.metric-value{font-size:clamp(2rem,6vw,3.3rem);font-weight:700;margin-top:8px}
.metric[data-live-metric=co2] .metric-value{color:var(--green)}
.metric[data-live-metric=temperature] .metric-value{color:var(--amber)}
.metric[data-live-metric=humidity] .metric-value{color:var(--blue)}
.monthly{margin-top:16px;padding:20px}
.monthly-head{display:flex;align-items:flex-end;justify-content:space-between;gap:16px;margin-bottom:18px}
.monthly-controls{display:flex;align-items:center;flex-wrap:wrap;gap:8px}
.monthly-controls select{width:auto;min-width:150px;padding:7px 34px 7px 11px;border-radius:999px;font-size:.82rem}
.monthly-chart{height:300px}
.summary{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin-top:14px}
.summary div{background:#0a192a;border:1px solid var(--line);border-radius:12px;padding:12px}
.summary span{display:block;color:var(--muted);font-size:.72rem;font-weight:700;letter-spacing:.1em;text-transform:uppercase}
.summary strong{display:block;font-size:1.3rem;margin-top:3px}
.settings{position:fixed;inset:0;z-index:10;display:none;align-items:center;justify-content:center;padding:18px;background:rgba(3,9,16,.78);backdrop-filter:blur(10px)}
.settings.open{display:flex}
.settings-panel{width:min(520px,100%);max-height:calc(100vh - 36px);overflow:auto;padding:24px}
.settings-head{display:flex;align-items:center;justify-content:space-between;gap:16px;margin-bottom:22px}
.settings-head h2{margin:0;font-size:1.25rem;letter-spacing:.06em}
.close{width:36px;height:36px;padding:0;font-size:1.35rem}
.field{display:grid;gap:8px;margin:18px 0}
.field-row{display:flex;align-items:center;justify-content:space-between;gap:16px}
label{color:var(--text);font-weight:650}
.help{color:var(--muted);font-size:.82rem}
input[type=number],select{width:100%;border:1px solid var(--line);border-radius:10px;background:#0a192a;color:var(--text);padding:11px 12px;font:inherit}
input[type=checkbox]{width:42px;height:22px;accent-color:var(--green)}
.primary{width:100%;border-color:var(--blue);color:var(--blue);padding:11px 16px;border-radius:10px;font-weight:700}
.danger-zone{border-top:1px solid var(--line);margin-top:24px;padding-top:20px}
.danger{width:100%;border-color:var(--red);color:var(--red);padding:11px 16px;border-radius:10px;font-weight:700}
.message{min-height:22px;margin-top:12px;color:var(--muted);font-size:.86rem}
footer{color:var(--muted);font-size:.78rem;margin-top:16px;text-align:right}
@media(max-width:720px){main{padding-top:18px}header{align-items:center}.connection{display:none}.grid{grid-template-columns:1fr}.reading{min-height:270px}.trend{min-height:330px}.reading-value{font-size:5.8rem}.trend-head,.monthly-head{align-items:flex-start;flex-direction:column}.ranges{justify-content:flex-start}.metrics{gap:10px}.metric{padding:14px}.metric-value{font-size:clamp(1.4rem,7vw,2.2rem)}.monthly-chart{height:260px}.monthly-controls select{width:100%}.settings-panel{padding:20px}}
</style>
</head>
<body>
<main>
<header><div><div class="eyebrow">Indoor air quality</div><h1>CO2 DASHBOARD</h1></div><div class="header-actions"><div class="connection" id="connection">Connecting...</div><button class="gear" id="settings-open" aria-label="Open settings" title="Settings"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" aria-hidden="true"><circle cx="12" cy="12" r="3"></circle><path d="M19.4 15a1.7 1.7 0 0 0 .3 1.9l.1.1-2.8 2.8-.1-.1a1.7 1.7 0 0 0-1.9-.3 1.7 1.7 0 0 0-1 1.6v.2h-4V21a1.7 1.7 0 0 0-1-1.6 1.7 1.7 0 0 0-1.9.3l-.1.1L4.2 17l.1-.1a1.7 1.7 0 0 0 .3-1.9A1.7 1.7 0 0 0 3 14H2.8v-4H3a1.7 1.7 0 0 0 1.6-1 1.7 1.7 0 0 0-.3-1.9L4.2 7 7 4.2l.1.1A1.7 1.7 0 0 0 9 4.6a1.7 1.7 0 0 0 1-1.6v-.2h4V3a1.7 1.7 0 0 0 1 1.6 1.7 1.7 0 0 0 1.9-.3l.1-.1L19.8 7l-.1.1a1.7 1.7 0 0 0-.3 1.9 1.7 1.7 0 0 0 1.6 1h.2v4H21a1.7 1.7 0 0 0-1.6 1Z"></path></svg></button></div></header>
<section class="grid">
<article class="card reading">
<div><div class="eyebrow" id="primary-label">Carbon dioxide</div><div class="reading-value" id="primary-value">---</div><div class="unit" id="primary-unit">PPM</div><div class="quality" id="quality">WARMING UP</div></div>
<div class="live"><span class="dot" id="dot"></span><span id="live">Waiting for sensor</span></div>
</article>
<article class="card trend">
<div class="trend-head"><div class="eyebrow" id="trend-label">CO2 trend</div><div class="ranges">
<button data-range="10" class="active">10 min</button><button data-range="60">1 hour</button><button data-range="360">6 hours</button><button data-range="720">12 hours</button><button data-range="1440">1 day</button><button data-range="10080">1 week</button>
</div></div>
<div class="chart-wrap"><canvas id="chart"></canvas></div>
</article>
</section>
<section class="metrics">
<article class="card metric active" data-live-metric="co2"><div class="eyebrow">CO2</div><div class="metric-value"><span id="co2-small">---</span> <small>ppm</small></div></article>
<article class="card metric" data-live-metric="temperature"><div class="eyebrow">Temperature</div><div class="metric-value"><span id="temperature">--.-</span> <small>&deg;C</small></div></article>
<article class="card metric" data-live-metric="humidity"><div class="eyebrow">Humidity</div><div class="metric-value"><span id="humidity">--.-</span> <small>%</small></div></article>
</section>
<section class="card monthly">
<div class="monthly-head"><div><div class="eyebrow">SD card archive</div><h2>MONTHLY HISTORY</h2></div><div class="monthly-controls"><select id="month" aria-label="History month"><option value="">Loading months...</option></select><button data-month-metric="co2" class="active">CO2</button><button data-month-metric="temperature">Temperature</button><button data-month-metric="humidity">Humidity</button></div></div>
<div class="chart-wrap monthly-chart"><canvas id="monthly-chart"></canvas></div>
<div class="summary"><div><span>Minimum</span><strong id="history-min">--</strong></div><div><span>Average</span><strong id="history-average">--</strong></div><div><span>Maximum</span><strong id="history-max">--</strong></div></div>
<div class="message" id="history-message">Select a stored month.</div>
</section>
<footer id="updated">Not updated yet</footer>
</main>
<section class="settings" id="settings" role="dialog" aria-modal="true" aria-labelledby="settings-title">
<div class="card settings-panel">
<div class="settings-head"><div><div class="eyebrow">Sensor configuration</div><h2 id="settings-title">SETTINGS</h2></div><button class="close" id="settings-close" aria-label="Close settings">&times;</button></div>
<form id="settings-form">
<div class="field field-row"><div><label for="asc">Automatic self-calibration</label><div class="help">Best for regularly occupied, well-ventilated rooms.</div></div><input id="asc" type="checkbox"></div>
<div class="field"><label for="offset">Temperature offset</label><input id="offset" type="number" min="0" max="10" step="0.1" inputmode="decimal"><div class="help">Compensates for sensor self-heating, from 0.0 to 10.0 &deg;C.</div></div>
<button class="primary" id="settings-save" type="submit">Save sensor settings</button>
</form>
<form id="display-settings-form">
<div class="field"><label for="color-order">Screen colour order</label><select id="color-order"><option value="bgr">BGR</option><option value="rgb">RGB</option></select><div class="help">If red and blue appear swapped on screen, switch this to match your panel's wiring.</div></div>
<button class="primary" id="display-settings-save" type="submit">Save display setting</button>
</form>
<div class="danger-zone">
<div class="field"><label for="reference">Forced recalibration reference</label><select id="reference"><option value="400">400 ppm</option><option value="420" selected>420 ppm</option><option value="450">450 ppm</option></select><div class="help">Only recalibrate after at least three minutes in stable air at the selected concentration.</div></div>
<button class="danger" id="frc" type="button">Run forced recalibration</button>
</div>
<div class="message" id="settings-message"></div>
</div>
</section>
<script>
const $=id=>document.getElementById(id);
const colors={green:"#42d392",amber:"#ffb547",red:"#ff5d73",blue:"#50b8ff",grid:"#15304b",muted:"#7794ac"};
const metricInfo={co2:{label:"Carbon dioxide",trend:"CO2 trend",unit:"PPM",accent:colors.green,digits:0},temperature:{label:"Temperature",trend:"Temperature trend",unit:"\u00b0C",accent:colors.amber,digits:1},humidity:{label:"Humidity",trend:"Humidity trend",unit:"%",accent:colors.blue,digits:1}};
let liveStatus=null,liveMetric="co2",range=10,trend={values:[]};
let monthly=null,monthlyMetric="co2";
function quality(ppm){return ppm<800?["FRESH",colors.green]:ppm<1200?["VENTILATE SOON",colors.amber]:["VENTILATE NOW",colors.red]}
async function getJson(url){const response=await fetch(url,{cache:"no-store"}),data=await response.json();if(!response.ok)throw Error(data.message||`Request failed (${response.status})`);return data}
function renderPrimary(){
 const info=metricInfo[liveMetric],valid=liveStatus?.valid&&(liveMetric==="co2"||liveStatus.environmental_valid),value=valid?liveStatus[liveMetric]:null;$("primary-label").textContent=info.label;$("trend-label").textContent=info.trend;$("primary-unit").textContent=info.unit;
 if(!valid){$("primary-value").textContent="---";$("quality").textContent="WARMING UP";$("quality").style.color=colors.amber;$("dot").style.background=colors.amber;return}
 $("primary-value").textContent=Number(value).toFixed(info.digits);
 if(liveMetric==="co2"){const q=quality(value);$("quality").textContent=q[0];$("quality").style.color=q[1];$("dot").style.background=q[1]}else{$("quality").textContent=liveMetric==="temperature"?"ROOM TEMPERATURE":"RELATIVE HUMIDITY";$("quality").style.color=info.accent;$("dot").style.background=info.accent}
}
function parseLocalTimestamp(value){if(!value)return null;const m=value.match(/^(\d{4})-(\d{2})-(\d{2})T(\d{2}):(\d{2}):(\d{2})$/);return m?new Date(+m[1],+m[2]-1,+m[3],+m[4],+m[5],+m[6]):null}
function formatTime(d){return d.toLocaleTimeString([], {hour:"2-digit",minute:"2-digit"})}
function formatDateTime(d){return d.toLocaleDateString([], {month:"short",day:"numeric"})+" "+formatTime(d)}
function trendTicks(now, minutes){
 const end=now?new Date(now):new Date(),start=new Date(end.getTime()-minutes*60000),ticks=[];
 if(minutes===10||minutes===60){const step=minutes===10?5:30;for(let t=new Date(Math.ceil(start.getTime()/(step*60000))*step*60000);t<=end;t=new Date(t.getTime()+step*60000))ticks.push(t)}
 if(minutes===360||minutes===720||minutes===1440){const step=minutes===1440?6:minutes===720?3:2;const t=new Date(start);t.setMinutes(0,0,0);if(t<start)t.setHours(t.getHours()+1);for(;t<=end;t.setHours(t.getHours()+step))ticks.push(new Date(t))}
 if(minutes===10080){const t=new Date(start);t.setHours(0,0,0,0);if(t<start)t.setDate(t.getDate()+1);for(;t<=end;t.setDate(t.getDate()+1))ticks.push(new Date(t))}
 return {start,end,ticks}
}
function drawTimeAxis(c,p,gw,gh,h,minutes){
 const now=parseLocalTimestamp(liveStatus?.timestamp),axis=trendTicks(now,minutes);c.font="11px system-ui";c.textBaseline="alphabetic";c.textAlign="center";c.fillStyle=colors.muted;c.strokeStyle=colors.grid;
 axis.ticks.forEach(t=>{const ratio=(t-axis.start)/(axis.end-axis.start);if(ratio<0||ratio>1)return;const x=p.l+ratio*gw;c.beginPath();c.moveTo(x,p.t);c.lineTo(x,p.t+gh);c.stroke();c.fillText(minutes>=1440?formatDateTime(t):formatTime(t),x,h-8)})
}
async function updateStatus(){
 try{const d=await getJson("/api/status");$("connection").textContent=d.connected?`${d.host} | ${d.ip}`:"Wi-Fi disconnected";
 $("updated").textContent=`Last update: ${d.timestamp||"time not synchronized"}`;
 liveStatus=d;$("co2-small").textContent=d.valid?d.co2:"---";$("temperature").textContent=d.environmental_valid?d.temperature.toFixed(1):"--.-";$("humidity").textContent=d.environmental_valid?d.humidity.toFixed(1):"--.-";$("live").textContent=d.valid?(d.environmental_valid?"Live sensor reading":"Temperature and humidity stabilizing"):"Sensor warming up";renderPrimary()}
 catch(e){$("connection").textContent="Dashboard offline";$("live").textContent="Connection lost"}
}
async function loadTrend(){try{trend=await getJson(`/api/trend?metric=${liveMetric}&minutes=${range}`);draw()}catch(e){trend={values:[],message:e.message};draw()}}
function monthLabel(value){const [year,month]=value.split("-").map(Number);return new Intl.DateTimeFormat(undefined,{month:"long",year:"numeric"}).format(new Date(year,month-1,1))}
async function loadMonths(){
 try{const d=await getJson("/api/months"),select=$("month");select.innerHTML="";
 if(!d.months.length){select.innerHTML='<option value="">No monthly data</option>';$("history-message").textContent="No synchronized monthly CSV files found.";return}
 d.months.forEach(value=>select.add(new Option(monthLabel(value),value)));await loadMonth(select.value)}
 catch(e){$("history-message").textContent=e.message||"Could not read the SD card archive."}
}
async function loadMonth(value){
 if(!value)return;$("history-message").textContent=`Loading ${monthLabel(value)}...`;
 try{monthly=await getJson(`/api/month-history?month=${encodeURIComponent(value)}`);$("history-message").textContent=monthly.samples?`${monthly.samples.toLocaleString()} one-minute readings`:"No readings in this month.";drawMonthly()}
 catch(e){monthly=null;$("history-message").textContent="Could not load the selected month.";drawMonthly()}
}
async function postSettings(values){const response=await fetch("/api/settings",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:new URLSearchParams(values)});const data=await response.json();if(!response.ok)throw Error(data.message||"Request failed");return data}
async function postDisplaySettings(values){const response=await fetch("/api/display-settings",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:new URLSearchParams(values)});const data=await response.json();if(!response.ok)throw Error(data.message||"Request failed");return data}
async function openSettings(){
 $("settings").classList.add("open");$("settings-message").textContent="Reading sensor settings...";
 try{const d=await getJson("/api/settings");$("asc").checked=d.asc;$("offset").value=d.temperature_offset.toFixed(1);$("reference").value=String(d.reference);$("settings-message").textContent=d.sensor_ready?"":"Sensor is not ready."}
 catch(e){$("settings-message").textContent="Could not read sensor settings."}
 try{const d=await getJson("/api/display-settings");$("color-order").value=d.color_order}
 catch(e){}
}
function closeSettings(){$("settings").classList.remove("open")}
function draw(){
 const canvas=$("chart"),rect=canvas.getBoundingClientRect(),scale=devicePixelRatio||1;canvas.width=Math.round(rect.width*scale);canvas.height=Math.round(rect.height*scale);
 const c=canvas.getContext("2d");c.scale(scale,scale);const w=rect.width,h=rect.height,p={l:48,r:10,t:12,b:25},gw=w-p.l-p.r,gh=h-p.t-p.b,values=trend.values||[],present=values.filter(v=>v!==null),info=metricInfo[liveMetric];
 c.clearRect(0,0,w,h);if(!present.length){c.fillStyle=colors.muted;c.font="12px system-ui";c.textAlign="center";c.fillText(trend.message||"Collecting history",w/2,h/2);return}let min=Math.min(...present),max=Math.max(...present);const padding=Math.max((max-min)*.12,liveMetric==="co2"?50:2);min=Math.floor(min-padding);max=Math.ceil(max+padding);if(max<=min)max=min+1;
 c.font="11px system-ui";c.fillStyle=colors.muted;c.textAlign="right";c.textBaseline="middle";
 for(let i=0;i<=4;i++){const y=p.t+i*gh/4,val=max-i*(max-min)/4;c.strokeStyle=colors.grid;c.beginPath();c.moveTo(p.l,y);c.lineTo(w-p.r,y);c.stroke();c.fillText(liveMetric==="co2"?Math.round(val):val.toFixed(1),p.l-7,y)}
 drawTimeAxis(c,p,gw,gh,h,range);
 c.lineWidth=2;c.lineJoin="round";c.lineCap="round";let previous=null;values.forEach((value,i)=>{if(value===null){previous=null;return}const point={x:p.l+i*gw/(values.length-1),y:p.t+gh-(value-min)*gh/(max-min)};if(previous){c.strokeStyle=liveMetric==="co2"?quality(value)[1]:info.accent;c.beginPath();c.moveTo(previous.x,previous.y);c.lineTo(point.x,point.y);c.stroke()}previous=point});
}
function drawMonthlyAxis(c,p,gw,gh,h){
 const start=parseLocalTimestamp(monthly?.first_timestamp),end=parseLocalTimestamp(monthly?.last_timestamp);if(!start||!end)return;c.font="11px system-ui";c.textBaseline="alphabetic";c.textAlign="center";c.fillStyle=colors.muted;c.strokeStyle=colors.grid;const ticks=[start];if(end>start)ticks.push(new Date((start.getTime()+end.getTime())/2),end);ticks.forEach(t=>{const ratio=end>start?(t-start)/(end-start):1;const x=p.l+ratio*gw;c.beginPath();c.moveTo(x,p.t);c.lineTo(x,p.t+gh);c.stroke();c.fillText(formatDateTime(t),x,h-9)})
}
function drawMonthly(){
 const canvas=$("monthly-chart"),rect=canvas.getBoundingClientRect(),scale=devicePixelRatio||1;canvas.width=Math.round(rect.width*scale);canvas.height=Math.round(rect.height*scale);
 const c=canvas.getContext("2d");c.scale(scale,scale);const w=rect.width,h=rect.height,p={l:50,r:12,t:14,b:28},gw=w-p.l-p.r,gh=h-p.t-p.b,values=monthly?.[monthlyMetric]||[],present=values.filter(v=>v!==null);
 c.clearRect(0,0,w,h);if(!present.length){c.fillStyle=colors.muted;c.font="12px system-ui";c.textAlign="center";c.fillText("No monthly data",w/2,h/2);["history-min","history-average","history-max"].forEach(id=>$(id).textContent="--");return}
 let min=Math.min(...present),max=Math.max(...present);const padding=Math.max((max-min)*.12,monthlyMetric==="co2"?50:2);min=Math.floor(min-padding);max=Math.ceil(max+padding);if(max<=min)max=min+1;
 c.font="11px system-ui";c.fillStyle=colors.muted;c.textAlign="right";c.textBaseline="middle";for(let i=0;i<=4;i++){const y=p.t+i*gh/4,val=max-i*(max-min)/4;c.strokeStyle=colors.grid;c.beginPath();c.moveTo(p.l,y);c.lineTo(w-p.r,y);c.stroke();c.fillText(monthlyMetric==="co2"?Math.round(val):val.toFixed(1),p.l-7,y)}
 drawMonthlyAxis(c,p,gw,gh,h);
 const color=monthlyMetric==="co2"?colors.green:monthlyMetric==="temperature"?colors.amber:colors.blue;c.strokeStyle=color;c.fillStyle=color;c.lineWidth=2;c.lineJoin="round";c.lineCap="round";let active=false,lastPoint=null;c.beginPath();values.forEach((value,i)=>{if(value===null){active=false;return}const x=p.l+(values.length>1?i*gw/(values.length-1):gw),y=p.t+gh-(value-min)*gh/(max-min);if(active)c.lineTo(x,y);else c.moveTo(x,y);active=true;lastPoint={x,y}});c.stroke();if(values.length===1&&lastPoint){c.beginPath();c.arc(lastPoint.x,lastPoint.y,3,0,Math.PI*2);c.fill()}
 const stats=monthly.stats?.[monthlyMetric],unit=monthlyMetric==="co2"?" ppm":monthlyMetric==="temperature"?" \u00b0C":" %",digits=monthlyMetric==="co2"?0:1;$("history-min").textContent=stats?stats.min.toFixed(digits)+unit:"--";$("history-average").textContent=stats?stats.average.toFixed(digits)+unit:"--";$("history-max").textContent=stats?stats.max.toFixed(digits)+unit:"--";
}
document.querySelectorAll("button[data-range]").forEach(b=>b.addEventListener("click",()=>{range=Number(b.dataset.range);document.querySelectorAll("button[data-range]").forEach(x=>x.classList.toggle("active",x===b));loadTrend()}));
document.querySelectorAll("[data-live-metric]").forEach(card=>card.addEventListener("click",()=>{liveMetric=card.dataset.liveMetric;document.querySelectorAll("[data-live-metric]").forEach(x=>x.classList.toggle("active",x===card));renderPrimary();loadTrend()}));
document.querySelectorAll("button[data-month-metric]").forEach(b=>b.addEventListener("click",()=>{monthlyMetric=b.dataset.monthMetric;document.querySelectorAll("button[data-month-metric]").forEach(x=>x.classList.toggle("active",x===b));drawMonthly()}));$("month").addEventListener("change",e=>loadMonth(e.target.value));
$("settings-open").addEventListener("click",openSettings);$("settings-close").addEventListener("click",closeSettings);$("settings").addEventListener("click",e=>{if(e.target===$("settings"))closeSettings()});
$("settings-form").addEventListener("submit",async e=>{e.preventDefault();const button=$("settings-save");button.disabled=true;$("settings-message").textContent="Saving settings...";try{const d=await postSettings({action:"save",asc:$("asc").checked?"1":"0",offset:$("offset").value});$("settings-message").textContent=d.message}catch(error){$("settings-message").textContent=error.message}finally{button.disabled=false}});
$("display-settings-form").addEventListener("submit",async e=>{e.preventDefault();const button=$("display-settings-save");button.disabled=true;$("settings-message").textContent="Saving display setting...";try{const d=await postDisplaySettings({color_order:$("color-order").value});$("settings-message").textContent=d.message}catch(error){$("settings-message").textContent=error.message}finally{button.disabled=false}});
$("frc").addEventListener("click",async()=>{const reference=$("reference").value;if(!confirm(`Run forced recalibration at ${reference} ppm? The sensor must have been in stable reference air for at least three minutes.`))return;const button=$("frc");button.disabled=true;$("settings-message").textContent="Recalibrating sensor...";try{const d=await postSettings({action:"frc",reference});$("settings-message").textContent=d.message}catch(error){$("settings-message").textContent=error.message}finally{button.disabled=false}});
addEventListener("keydown",e=>{if(e.key==="Escape")closeSettings()});
addEventListener("resize",()=>{draw();drawMonthly()});updateStatus();loadTrend();loadMonths();setInterval(updateStatus,5000);setInterval(loadTrend,10000);
</script>
</body>
</html>
)rawliteral";

void addJsonString(String& json, const char* value) {
  json += '"';
  for (const char* cursor = value; *cursor != '\0'; ++cursor) {
    if (*cursor == '"' || *cursor == '\\') json += '\\';
    if (static_cast<uint8_t>(*cursor) >= 0x20) json += *cursor;
  }
  json += '"';
}

void handleWebStatus() {
  String json;
  json.reserve(320);
  json = F("{\"valid\":");
  json += co2_reading_valid ? F("true") : F("false");
  json += F(",\"environmental_valid\":");
  json += environmental_reading_valid ? F("true") : F("false");
  json += F(",\"co2\":");
  json += current.co2;
  json += F(",\"temperature\":");
  json += current.valid ? String(current.temperature, 2) : F("0");
  json += F(",\"humidity\":");
  json += current.valid ? String(current.humidity, 2) : F("0");
  json += F(",\"connected\":");
  json += WiFi.status() == WL_CONNECTED ? F("true") : F("false");
  json += F(",\"host\":\"co2-dashboard.local\",\"ip\":");
  addJsonString(json, WiFi.localIP().toString().c_str());
  json += F(",\"timestamp\":");
  char timestamp[24] = {};
  if (formatLocalTimestamp(timestamp, sizeof(timestamp))) {
    addJsonString(json, timestamp);
  } else {
    json += F("\"\"");
  }
  json += F(",\"uptime_seconds\":");
  json += millis() / 1000UL;
  json += '}';
  web_server.sendHeader(F("Cache-Control"), F("no-store"));
  web_server.send(200, F("application/json"), json);
}

void appendHistoryArray(String& json, const uint16_t* values, size_t capacity,
                        size_t count, size_t head) {
  json += '[';
  for (size_t i = 0; i < count; ++i) {
    if (i > 0) json += ',';
    json += ringValueAt(values, capacity, count, head, i);
  }
  json += ']';
}

void handleWebHistory() {
  String json;
  json.reserve(6400);
  json = F("{\"short\":");
  appendHistoryArray(json, short_history, config::kShortHistorySize,
                     short_history_count, short_history_head);
  json += F(",\"long\":");
  appendHistoryArray(json, long_history, config::kLongHistorySize,
                     long_history_count, long_history_head);
  json += '}';
  web_server.sendHeader(F("Cache-Control"), F("no-store"));
  web_server.send(200, F("application/json"), json);
}

bool validMonthName(const String& month) {
  if (month.length() != 7 || month[4] != '-') return false;
  for (size_t i = 0; i < month.length(); ++i) {
    if (i == 4) continue;
    if (month[i] < '0' || month[i] > '9') return false;
  }
  const int year = month.substring(0, 4).toInt();
  const int month_number = month.substring(5, 7).toInt();
  return year >= 2000 && year <= 2099 &&
         month_number >= 1 && month_number <= 12;
}

bool monthlyLogName(const char* file_name, char* month,
                    size_t month_size) {
  constexpr size_t kExpectedLength = 15;
  if (strlen(file_name) != kExpectedLength ||
      strncasecmp(file_name, config::kMonthlyLogPrefix,
                  strlen(config::kMonthlyLogPrefix)) != 0 ||
      strcasecmp(file_name + 11, ".csv") != 0) {
    return false;
  }
  char candidate[8] = {};
  memcpy(candidate, file_name + 4, 7);
  if (!validMonthName(String(candidate))) return false;
  snprintf(month, month_size, "%s", candidate);
  return true;
}

void handleWebMonths() {
  if (!sd_ready && !beginSdCard()) {
    web_server.send(503, F("application/json"),
                    F("{\"months\":[],\"message\":\"SD card unavailable\"}"));
    return;
  }

  constexpr size_t kMaxMonths = 48;
  char months[kMaxMonths][8] = {};
  size_t month_count = 0;
  FsFile directory = sd.open("/", O_RDONLY);
  FsFile entry;
  if (!directory) {
    Serial.println("Web archive: could not open SD root directory");
    web_server.send(
        500, F("application/json"),
        F("{\"months\":[],\"message\":\"Could not open SD root directory\"}"));
    return;
  }

  while (month_count < kMaxMonths &&
         entry.openNext(&directory, O_RDONLY)) {
    if (!entry.isDir()) {
      char file_name[32] = {};
      char month[8] = {};
      entry.getName(file_name, sizeof(file_name));
      if (monthlyLogName(file_name, month, sizeof(month))) {
        bool duplicate = false;
        for (size_t i = 0; i < month_count; ++i) {
          duplicate = duplicate || strcmp(months[i], month) == 0;
        }
        if (!duplicate) {
          snprintf(months[month_count++], sizeof(months[0]), "%s", month);
        }
      }
    }
    entry.close();
  }
  directory.close();

  for (size_t i = 0; i < month_count; ++i) {
    for (size_t j = i + 1; j < month_count; ++j) {
      if (strcmp(months[i], months[j]) < 0) {
        char temporary[8] = {};
        memcpy(temporary, months[i], sizeof(temporary));
        memcpy(months[i], months[j], sizeof(months[i]));
        memcpy(months[j], temporary, sizeof(months[j]));
      }
    }
  }

  String json;
  json.reserve(16 + month_count * 11);
  json = F("{\"months\":[");
  for (size_t i = 0; i < month_count; ++i) {
    if (i > 0) json += ',';
    addJsonString(json, months[i]);
  }
  json += F("]}");
  Serial.printf("Web archive: found %u monthly file(s)\n",
                static_cast<unsigned>(month_count));
  web_server.sendHeader(F("Cache-Control"), F("no-store"));
  web_server.send(200, F("application/json"), json);
}

bool parseMonthlyRow(char* line, uint16_t& co2, float& temperature,
                     float& humidity, char*& timestamp) {
  char* fields[6] = {};
  fields[0] = line;
  for (size_t i = 1; i < 6; ++i) {
    char* separator = strchr(fields[i - 1], ',');
    if (separator == nullptr) return false;
    *separator = '\0';
    fields[i] = separator + 1;
  }

  char* end = nullptr;
  const unsigned long parsed_co2 = strtoul(fields[2], &end, 10);
  if (end == fields[2] || *end != '\0' || parsed_co2 == 0 ||
      parsed_co2 > UINT16_MAX) {
    return false;
  }
  co2 = static_cast<uint16_t>(parsed_co2);

  temperature = strtof(fields[3], &end);
  if (end == fields[3] || *end != '\0' || !isfinite(temperature)) {
    return false;
  }
  humidity = strtof(fields[4], &end);
  if (end == fields[4] || *end != '\0' || !isfinite(humidity)) {
    return false;
  }
  timestamp = fields[5];
  timestamp[strcspn(timestamp, "\r\n")] = '\0';
  return true;
}

int daysInMonth(int year, int month) {
  static constexpr uint8_t kDays[] = {
      31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (month != 2) return kDays[month - 1];
  const bool leap = (year % 4 == 0 && year % 100 != 0) ||
                    year % 400 == 0;
  return leap ? 29 : 28;
}

void appendWebHistoryValues(String& json, const char* name,
                            const uint32_t* integer_sums,
                            const float* decimal_sums,
                            size_t point_count) {
  json += '"';
  json += name;
  json += F("\":[");
  for (size_t i = 0; i < point_count; ++i) {
    if (i > 0) json += ',';
    if (web_history_bucket_count[i] == 0) {
      json += F("null");
    } else if (integer_sums != nullptr) {
      json += static_cast<float>(integer_sums[i]) /
              web_history_bucket_count[i];
    } else {
      json += String(decimal_sums[i] / web_history_bucket_count[i], 2);
    }
  }
  json += ']';
}

void handleWebMonthHistory() {
  const String month = web_server.arg("month");
  if (!validMonthName(month)) {
    web_server.send(400, F("application/json"),
                    F("{\"message\":\"Invalid month\"}"));
    return;
  }
  if (!sd_ready && !beginSdCard()) {
    web_server.send(503, F("application/json"),
                    F("{\"message\":\"SD card unavailable\"}"));
    return;
  }

  char file_name[20] = {};
  snprintf(file_name, sizeof(file_name), "co2-%s.csv", month.c_str());
  FsFile log_file;
  if (!log_file.open(file_name, O_RDONLY)) {
    web_server.send(404, F("application/json"),
                    F("{\"message\":\"Month not found\"}"));
    return;
  }

  const int year = month.substring(0, 4).toInt();
  const int month_number = month.substring(5, 7).toInt();
  uint32_t sample_count = 0;
  uint64_t co2_total = 0;
  double temperature_total = 0.0;
  double humidity_total = 0.0;
  uint16_t co2_min = UINT16_MAX;
  uint16_t co2_max = 0;
  float temperature_min = INFINITY;
  float temperature_max = -INFINITY;
  float humidity_min = INFINITY;
  float humidity_max = -INFINITY;
  char first_timestamp[24] = {};
  char last_timestamp[24] = {};
  uint32_t scanned_rows = 0;
  char line[128] = {};

  while (log_file.fgets(line, sizeof(line)) > 0) {
    if ((++scanned_rows & 0x7F) == 0) yield();
    uint16_t co2 = 0;
    float temperature = 0.0f;
    float humidity = 0.0f;
    char* timestamp = nullptr;
    if (!parseMonthlyRow(line, co2, temperature, humidity, timestamp) ||
        timestamp == nullptr || strlen(timestamp) < 16 ||
        strncmp(timestamp, month.c_str(), 7) != 0) {
      continue;
    }

    const int day = (timestamp[8] - '0') * 10 + timestamp[9] - '0';
    const int hour = (timestamp[11] - '0') * 10 + timestamp[12] - '0';
    const int minute = (timestamp[14] - '0') * 10 + timestamp[15] - '0';
    if (day < 1 || day > daysInMonth(year, month_number) ||
        hour < 0 || hour > 23 || minute < 0 || minute > 59) {
      continue;
    }
    if (first_timestamp[0] == '\0') {
      snprintf(first_timestamp, sizeof(first_timestamp), "%s", timestamp);
    }
    snprintf(last_timestamp, sizeof(last_timestamp), "%s", timestamp);
    ++sample_count;
    co2_total += co2;
    temperature_total += temperature;
    humidity_total += humidity;
    co2_min = std::min(co2_min, co2);
    co2_max = std::max(co2_max, co2);
    temperature_min = std::min(temperature_min, temperature);
    temperature_max = std::max(temperature_max, temperature);
    humidity_min = std::min(humidity_min, humidity);
    humidity_max = std::max(humidity_max, humidity);
  }
  log_file.close();

  std::fill(std::begin(web_history_co2_sum),
            std::end(web_history_co2_sum), 0);
  std::fill(std::begin(web_history_temperature_sum),
            std::end(web_history_temperature_sum), 0.0f);
  std::fill(std::begin(web_history_humidity_sum),
            std::end(web_history_humidity_sum), 0.0f);
  std::fill(std::begin(web_history_bucket_count),
            std::end(web_history_bucket_count), 0);

  const size_t point_count =
      std::min<size_t>(sample_count, config::kWebHistoryPoints);
  if (sample_count > 0 && log_file.open(file_name, O_RDONLY)) {
    uint32_t row_index = 0;
    scanned_rows = 0;
    while (log_file.fgets(line, sizeof(line)) > 0) {
      if ((++scanned_rows & 0x7F) == 0) yield();
      uint16_t co2 = 0;
      float temperature = 0.0f;
      float humidity = 0.0f;
      char* timestamp = nullptr;
      if (!parseMonthlyRow(line, co2, temperature, humidity, timestamp) ||
          timestamp == nullptr || strlen(timestamp) < 16 ||
          strncmp(timestamp, month.c_str(), 7) != 0) {
        continue;
      }
      const size_t bucket = std::min<size_t>(
          static_cast<uint64_t>(row_index) * point_count / sample_count,
          point_count - 1);
      web_history_co2_sum[bucket] += co2;
      web_history_temperature_sum[bucket] += temperature;
      web_history_humidity_sum[bucket] += humidity;
      ++web_history_bucket_count[bucket];
      ++row_index;
    }
    log_file.close();
  }

  String json;
  json.reserve(12000);
  json = F("{\"month\":");
  addJsonString(json, month.c_str());
  json += F(",\"samples\":");
  json += sample_count;
  json += F(",\"points\":");
  json += point_count;
  json += F(",\"first_timestamp\":");
  addJsonString(json, first_timestamp);
  json += F(",\"last_timestamp\":");
  addJsonString(json, last_timestamp);
  json += ',';
  appendWebHistoryValues(json, "co2", web_history_co2_sum, nullptr,
                         point_count);
  json += ',';
  appendWebHistoryValues(json, "temperature", nullptr,
                         web_history_temperature_sum, point_count);
  json += ',';
  appendWebHistoryValues(json, "humidity", nullptr,
                         web_history_humidity_sum, point_count);
  json += F(",\"stats\":");
  if (sample_count == 0) {
    json += F("null");
  } else {
    json += F("{\"co2\":{\"min\":");
    json += co2_min;
    json += F(",\"max\":");
    json += co2_max;
    json += F(",\"average\":");
    json += static_cast<float>(co2_total) / sample_count;
    json += F("},\"temperature\":{\"min\":");
    json += String(temperature_min, 2);
    json += F(",\"max\":");
    json += String(temperature_max, 2);
    json += F(",\"average\":");
    json += String(temperature_total / sample_count, 2);
    json += F("},\"humidity\":{\"min\":");
    json += String(humidity_min, 2);
    json += F(",\"max\":");
    json += String(humidity_max, 2);
    json += F(",\"average\":");
    json += String(humidity_total / sample_count, 2);
    json += F("}}");
  }
  json += '}';
  web_server.sendHeader(F("Cache-Control"), F("no-store"));
  web_server.send(200, F("application/json"), json);
}

void sendWebMessage(int status, bool ok, const char* message,
                    int16_t correction = 0, bool include_correction = false) {
  String json;
  json.reserve(128);
  json = F("{\"ok\":");
  json += ok ? F("true") : F("false");
  json += F(",\"message\":");
  addJsonString(json, message);
  if (include_correction) {
    json += F(",\"correction\":");
    json += correction;
  }
  json += '}';
  web_server.sendHeader(F("Cache-Control"), F("no-store"));
  web_server.send(status, F("application/json"), json);
}

void handleWebSettingsGet() {
  String json;
  json.reserve(180);
  json = F("{\"sensor_ready\":");
  json += sensor_ready && calibration_loaded ? F("true") : F("false");
  json += F(",\"asc\":");
  json += asc_enabled ? F("true") : F("false");
  json += F(",\"temperature_offset\":");
  json += String(calibration_temperature_offset, 1);
  json += F(",\"reference\":");
  json += calibration_target;
  json += '}';
  web_server.sendHeader(F("Cache-Control"), F("no-store"));
  web_server.send(200, F("application/json"), json);
}

void handleWebSettingsPost() {
  if (!sensor_ready || calibration_busy) {
    sendWebMessage(503, false, "Sensor is not ready.");
    return;
  }

  const String action = web_server.arg("action");
  if (action == "save") {
    const String asc_value = web_server.arg("asc");
    const String offset_value = web_server.arg("offset");
    if ((asc_value != "0" && asc_value != "1") ||
        offset_value.length() == 0) {
      sendWebMessage(400, false, "Invalid sensor settings.");
      return;
    }

    char* end = nullptr;
    const float offset = strtof(offset_value.c_str(), &end);
    if (end == offset_value.c_str() || *end != '\0' || !isfinite(offset) ||
        offset < 0.0f || offset > 10.0f) {
      sendWebMessage(400, false,
                     "Temperature offset must be between 0.0 and 10.0 C.");
      return;
    }

    calibration_busy = true;
    const bool next_asc = asc_value == "1";
    const bool ok =
        sensor.saveSettings(next_asc, offset, calibration_altitude);
    calibration_busy = false;
    last_reading_ms = millis();
    if (!ok) {
      sendWebMessage(500, false, "Sensor command failed.");
      return;
    }

    asc_enabled = next_asc;
    calibration_temperature_offset = offset;
    calibration_loaded = true;
    resetStabilizationWindows();
    sendWebMessage(200, true, "Sensor settings saved.");
    return;
  }

  if (action == "frc") {
    const int reference = web_server.arg("reference").toInt();
    if (reference != 400 && reference != 420 && reference != 450) {
      sendWebMessage(400, false,
                     "Reference must be 400, 420, or 450 ppm.");
      return;
    }

    calibration_busy = true;
    int16_t correction = 0;
    const bool ok = sensor.performForcedRecalibration(
        static_cast<uint16_t>(reference), correction);
    calibration_busy = false;
    current.valid = false;
    consecutive_sensor_misses = 0;
    last_reading_ms = millis();
    if (!ok) {
      sendWebMessage(500, false, "Forced recalibration failed.");
      return;
    }

    calibration_target = static_cast<uint16_t>(reference);
    resetStabilizationWindows();
    sendWebMessage(200, true, "Forced recalibration completed.", correction,
                   true);
    return;
  }

  sendWebMessage(400, false, "Unknown settings action.");
}

void handleWebDisplaySettingsGet() {
  String json;
  json.reserve(48);
  json = F("{\"color_order\":\"");
  json += display.colorOrderRgb() ? F("rgb") : F("bgr");
  json += F("\"}");
  web_server.sendHeader(F("Cache-Control"), F("no-store"));
  web_server.send(200, F("application/json"), json);
}

void handleWebDisplaySettingsPost() {
  const String color_order = web_server.arg("color_order");
  if (color_order != "rgb" && color_order != "bgr") {
    sendWebMessage(400, false, "Color order must be 'rgb' or 'bgr'.");
    return;
  }

  const bool rgb_order = color_order == "rgb";
  display.setColorOrderRgb(rgb_order);
  preferences.putUChar("color_order", rgb_order ? 1 : 0);
  sendWebMessage(200, true, "Display setting saved.");
}

void handleWebTrend();

void startWebDashboard() {
  if (web_server_started || WiFi.status() != WL_CONNECTED) return;

  web_server.on("/", HTTP_GET, []() {
    web_server.sendHeader(F("Cache-Control"), F("no-cache"));
    web_server.send_P(200, PSTR("text/html"), kDashboardHtml);
  });
  web_server.on("/api/status", HTTP_GET, handleWebStatus);
  web_server.on("/api/history", HTTP_GET, handleWebHistory);
  web_server.on("/api/trend", HTTP_GET, handleWebTrend);
  web_server.on("/api/months", HTTP_GET, handleWebMonths);
  web_server.on("/api/month-history", HTTP_GET, handleWebMonthHistory);
  web_server.on("/api/settings", HTTP_GET, handleWebSettingsGet);
  web_server.on("/api/settings", HTTP_POST, handleWebSettingsPost);
  web_server.on("/api/display-settings", HTTP_GET, handleWebDisplaySettingsGet);
  web_server.on("/api/display-settings", HTTP_POST, handleWebDisplaySettingsPost);
  web_server.onNotFound([]() {
    web_server.send(404, F("text/plain"), F("Not found"));
  });
  web_server.begin();
  web_server_started = true;

  const bool mdns_started = MDNS.begin("co2-dashboard");
  if (mdns_started) MDNS.addService("http", "tcp", 80);
  Serial.printf("Web dashboard: http://%s/ (%s)\n",
                WiFi.localIP().toString().c_str(),
                mdns_started ? "co2-dashboard.local" : "mDNS unavailable");
}

void serviceWebDashboard() {
  startWebDashboard();
  if (web_server_started) web_server.handleClient();
}

size_t longTermExpectedRows() {
  switch (long_term_range) {
    case LongTermRange::kDay:
      return 24 * 60;
    case LongTermRange::kWeek:
      return 7 * 24 * 60;
    case LongTermRange::kMonth:
      return 30 * 24 * 60;
    default:
      return 24 * 60;
  }
}

const char* longTermRangeLabel() {
  switch (long_term_range) {
    case LongTermRange::kDay:
      return "24 HOURS";
    case LongTermRange::kWeek:
      return "7 DAYS";
    case LongTermRange::kMonth:
      return "30 DAYS";
    default:
      return "";
  }
}

struct LogTail {
  uint32_t offset = 0;
  size_t rows = 0;
};

LogTail findLogTail(FsFile& file, size_t requested_rows) {
  LogTail result = {};
  uint32_t position = file.fileSize();
  size_t newline_count = 0;
  uint16_t blocks_read = 0;
  uint8_t buffer[config::kSdReadBlockSize] = {};

  while (position > 0) {
    if ((++blocks_read & 0x1F) == 0) yield();
    const size_t block_size =
        std::min<size_t>(position, sizeof(buffer));
    position -= block_size;
    if (!file.seekSet(position) ||
        file.read(buffer, block_size) != static_cast<int>(block_size)) {
      return {};
    }

    for (size_t i = block_size; i > 0; --i) {
      if (buffer[i - 1] != '\n') continue;
      ++newline_count;
      if (newline_count > requested_rows) {
        result.offset = position + i;
        result.rows = requested_rows;
        return result;
      }
    }
  }

  result.offset = 0;
  result.rows = newline_count > 0 ? newline_count - 1 : 0;
  return result;
}

bool parseLogRow(char* line, uint16_t& co2, char*& timestamp) {
  char* field = line;
  for (int column = 0; column < 2; ++column) {
    field = strchr(field, ',');
    if (field == nullptr) return false;
    ++field;
  }

  char* end = nullptr;
  const unsigned long parsed_co2 = strtoul(field, &end, 10);
  if (end == field || parsed_co2 == 0 || parsed_co2 > UINT16_MAX) {
    return false;
  }
  co2 = static_cast<uint16_t>(parsed_co2);

  timestamp = end;
  for (int column = 0; column < 3 && timestamp != nullptr; ++column) {
    timestamp = strchr(timestamp, ',');
    if (timestamp != nullptr) ++timestamp;
  }
  if (timestamp != nullptr) {
    timestamp[strcspn(timestamp, "\r\n")] = '\0';
  }
  return true;
}

size_t aggregateTrendFile(const char* file_name, size_t requested_rows,
                          size_t output_start, size_t expected_rows,
                          size_t point_count, const char* metric) {
  if (requested_rows == 0) return 0;

  FsFile log_file;
  if (!log_file.open(file_name, O_RDONLY)) return 0;
  const LogTail tail = findLogTail(log_file, requested_rows);
  if (tail.rows == 0 || !log_file.seekSet(tail.offset)) {
    log_file.close();
    return 0;
  }

  size_t row_index = 0;
  uint32_t scanned_rows = 0;
  char line[128] = {};
  while (row_index < tail.rows &&
         log_file.fgets(line, sizeof(line)) > 0) {
    if ((++scanned_rows & 0x7F) == 0) yield();
    uint16_t co2 = 0;
    float temperature = 0.0f;
    float humidity = 0.0f;
    char* timestamp = nullptr;
    if (!parseMonthlyRow(line, co2, temperature, humidity, timestamp)) {
      continue;
    }

    const size_t bucket = std::min<size_t>(
        (output_start + row_index) * point_count / expected_rows,
        point_count - 1);
    const float value =
        strcmp(metric, "temperature") == 0 ? temperature :
        strcmp(metric, "humidity") == 0 ? humidity :
                                           static_cast<float>(co2);
    web_history_temperature_sum[bucket] += value;
    ++web_history_bucket_count[bucket];
    ++row_index;
  }
  log_file.close();
  return row_index;
}

void sendTrendResponse(const char* metric, size_t requested_minutes,
                       size_t sample_count, size_t point_count) {
  String json;
  json.reserve(7200);
  json = F("{\"metric\":");
  addJsonString(json, metric);
  json += F(",\"minutes\":");
  json += requested_minutes;
  json += F(",\"samples\":");
  json += sample_count;
  json += F(",\"points\":");
  json += point_count;
  json += F(",\"values\":[");
  for (size_t i = 0; i < point_count; ++i) {
    if (i > 0) json += ',';
    if (web_history_bucket_count[i] == 0) {
      json += F("null");
    } else {
      json += String(web_history_temperature_sum[i] /
                         web_history_bucket_count[i],
                     strcmp(metric, "co2") == 0 ? 0 : 2);
    }
  }
  json += F("]}");
  web_server.sendHeader(F("Cache-Control"), F("no-store"));
  web_server.send(200, F("application/json"), json);
}

void handleRecentWebTrend(const char* metric, size_t requested_minutes) {
  const bool short_range = requested_minutes == 10;
  const size_t expected_samples =
      short_range ? config::kShortHistorySize : requested_minutes;
  const size_t capacity =
      short_range ? config::kShortHistorySize : config::kLongHistorySize;
  const size_t count = short_range ? short_history_count : long_history_count;
  const size_t head = short_range ? short_history_head : long_history_head;
  const size_t source_count = std::min(count, expected_samples);
  const size_t source_start = count - source_count;
  const size_t missing_samples = expected_samples - source_count;
  const size_t point_count =
      std::min(expected_samples, config::kWebHistoryPoints);

  std::fill(std::begin(web_history_temperature_sum),
            std::end(web_history_temperature_sum), 0.0f);
  std::fill(std::begin(web_history_bucket_count),
            std::end(web_history_bucket_count), 0);

  size_t samples = 0;
  for (size_t i = 0; i < source_count; ++i) {
    float value = NAN;
    if (strcmp(metric, "co2") == 0) {
      const uint16_t* values = short_range ? short_history : long_history;
      value = ringValueAt(values, capacity, count, head, source_start + i);
    } else {
      const float* values =
          strcmp(metric, "temperature") == 0
              ? (short_range ? short_temperature_history
                             : long_temperature_history)
              : (short_range ? short_humidity_history
                             : long_humidity_history);
      value = ringValueAt(values, capacity, count, head, source_start + i);
    }
    if (!isfinite(value)) continue;

    const size_t bucket = std::min<size_t>(
        (missing_samples + i) * point_count / expected_samples,
        point_count - 1);
    web_history_temperature_sum[bucket] += value;
    ++web_history_bucket_count[bucket];
    ++samples;
  }
  sendTrendResponse(metric, requested_minutes, samples, point_count);
}

void handleWebTrend() {
  const String metric_argument = web_server.arg("metric");
  const String range_argument = web_server.arg("minutes");
  const char* metric = metric_argument.c_str();
  const size_t requested_rows = range_argument.toInt();
  if ((metric_argument != "co2" && metric_argument != "temperature" &&
       metric_argument != "humidity") ||
      (requested_rows != 10 && requested_rows != 60 &&
       requested_rows != 360 && requested_rows != 720 &&
       requested_rows != 1440 && requested_rows != 10080)) {
    web_server.send(400, F("application/json"),
                    F("{\"message\":\"Invalid trend request\"}"));
    return;
  }
  // CO2 has persisted NVS history for 1-12 hour ranges. Temperature and
  // humidity only keep the 10-minute live history in RAM, so longer ranges
  // are rebuilt from monthly CSV after a reboot.
  if (requested_rows <= 720 &&
      (metric_argument == "co2" || requested_rows == 10)) {
    handleRecentWebTrend(metric, requested_rows);
    return;
  }
  if (!sd_ready && !beginSdCard()) {
    web_server.send(503, F("application/json"),
                    F("{\"message\":\"SD card unavailable\"}"));
    return;
  }

  struct tm local_time = {};
  if (!getLocalTime(&local_time, 0)) {
    web_server.send(
        503, F("application/json"),
        F("{\"message\":\"Network time has not synchronized yet\"}"));
    return;
  }

  char current_file[20] = {};
  strftime(current_file, sizeof(current_file), "co2-%Y-%m.csv",
           &local_time);
  const size_t current_rows = [&]() {
    FsFile file;
    if (!file.open(current_file, O_RDONLY)) return static_cast<size_t>(0);
    const LogTail tail = findLogTail(file, requested_rows);
    file.close();
    return tail.rows;
  }();

  size_t previous_rows = requested_rows - current_rows;
  char previous_file[20] = {};
  if (previous_rows > 0) {
    local_time.tm_mday = 1;
    local_time.tm_mon -= 1;
    mktime(&local_time);
    strftime(previous_file, sizeof(previous_file), "co2-%Y-%m.csv",
             &local_time);
    FsFile file;
    if (file.open(previous_file, O_RDONLY)) {
      const LogTail tail = findLogTail(file, previous_rows);
      previous_rows = tail.rows;
      file.close();
    } else {
      previous_rows = 0;
    }
  }

  std::fill(std::begin(web_history_temperature_sum),
            std::end(web_history_temperature_sum), 0.0f);
  std::fill(std::begin(web_history_bucket_count),
            std::end(web_history_bucket_count), 0);

  const size_t point_count =
      std::min(requested_rows, config::kWebHistoryPoints);
  const size_t available_rows = previous_rows + current_rows;
  const size_t missing_rows =
      requested_rows > available_rows ? requested_rows - available_rows : 0;
  size_t processed_rows = 0;
  if (previous_rows > 0) {
    processed_rows += aggregateTrendFile(
        previous_file, previous_rows, missing_rows, requested_rows,
        point_count, metric);
  }
  processed_rows += aggregateTrendFile(
      current_file, current_rows, missing_rows + processed_rows,
      requested_rows, point_count, metric);

  sendTrendResponse(metric, requested_rows, processed_rows, point_count);
}

void loadLongTermHistory() {
  std::fill(std::begin(long_term_plot), std::end(long_term_plot), 0);
  long_term_plot_count = 0;
  long_term_min = 0;
  long_term_max = 0;
  long_term_average = 0;
  long_term_last_timestamp[0] = '\0';
  long_term_dirty = false;

  if (!sd_ready && !beginSdCard()) return;

  char file_name[20] = {};
  activeLogFileName(file_name, sizeof(file_name));
  FsFile log_file;
  if (!log_file.open(file_name, O_RDONLY)) {
    sd_ready = false;
    return;
  }

  const size_t expected_rows = longTermExpectedRows();
  const LogTail tail = findLogTail(log_file, expected_rows);
  if (tail.rows == 0 || !log_file.seekSet(tail.offset)) {
    log_file.close();
    return;
  }

  uint32_t bucket_sum[config::kLongTermPlotPoints] = {};
  uint16_t bucket_count[config::kLongTermPlotPoints] = {};
  const size_t missing_rows = expected_rows - tail.rows;
  size_t row_index = 0;
  uint64_t total = 0;
  char line[128] = {};

  while (row_index < tail.rows &&
         log_file.fgets(line, sizeof(line)) > 0) {
    uint16_t co2 = 0;
    float temperature = 0.0f;
    float humidity = 0.0f;
    char* timestamp = nullptr;
    if (!parseMonthlyRow(line, co2, temperature, humidity, timestamp)) {
      continue;
    }
    float value = co2;
    if (display_metric == DisplayMetric::kTemperature) {
      value = temperature;
    } else if (display_metric == DisplayMetric::kHumidity) {
      value = humidity;
    }
    const uint16_t scaled_value = scaledMetricValue(display_metric, value);
    if (scaled_value == 0) continue;

    size_t bucket =
        (missing_rows + row_index) * config::kLongTermPlotPoints /
        expected_rows;
    bucket = std::min(bucket, config::kLongTermPlotPoints - 1);
    bucket_sum[bucket] += scaled_value;
    ++bucket_count[bucket];
    total += scaled_value;
    ++row_index;

    if (timestamp != nullptr && timestamp[0] != '\0' &&
        strcmp(timestamp, "unsynced") != 0) {
      snprintf(long_term_last_timestamp,
               sizeof(long_term_last_timestamp), "%s", timestamp);
    }
  }
  log_file.close();

  if (row_index == 0) return;
  long_term_average = static_cast<uint16_t>(total / row_index);
  long_term_plot_count = row_index;
  for (size_t i = 0; i < config::kLongTermPlotPoints; ++i) {
    if (bucket_count[i] == 0) continue;
    long_term_plot[i] =
        static_cast<uint16_t>(bucket_sum[i] / bucket_count[i]);
    if (long_term_min == 0 || long_term_plot[i] < long_term_min) {
      long_term_min = long_term_plot[i];
    }
    long_term_max = std::max(long_term_max, long_term_plot[i]);
  }
}

void drawRoundedPanel(int x, int y, int w, int h) {
  canvas.fillRoundRect(x, y, w, h, 10, kPanel);
  canvas.drawRoundRect(x, y, w, h, 10, kPanelLight);
}

void drawStaticDashboard() {
  canvas.fillScreen(kBackground);

  canvas.setTextDatum(textdatum_t::middle_left);
  canvas.setTextColor(kWhite);
  canvas.setFont(&fonts::Font2);
  canvas.drawString(metricTitle(display_metric), 12, 15);

  drawRoundedPanel(8, 31, 126, 133);
  canvas.setTextDatum(textdatum_t::top_left);
  canvas.setTextColor(kMuted);
  canvas.drawString(metricTitle(display_metric), 19, 42);

  drawRoundedPanel(142, 31, 170, 133);
  canvas.drawString(metricTrendTitle(display_metric), 152, 41);

  const DisplayMetric left_metric = leftBottomMetric();
  const DisplayMetric right_metric = rightBottomMetric();
  drawRoundedPanel(8, 172, 148, 58);
  canvas.fillRoundRect(17, 182, 4, 38, 2, currentMetricColor(left_metric));
  canvas.setTextDatum(textdatum_t::top_left);
  canvas.setTextColor(kMuted);
  canvas.drawString(metricTitle(left_metric), 30, 182);

  drawRoundedPanel(164, 172, 148, 58);
  canvas.fillRoundRect(173, 182, 4, 38, 2, currentMetricColor(right_metric));
  canvas.drawString(metricTitle(right_metric), 186, 182);
}

void drawDateTime() {
  canvas.fillRect(105, 5, 207, 20, kBackground);
  char date_time[20] = "--/--/-- --:--";
  struct tm local_time = {};
  if (getLocalTime(&local_time, 0)) {
    strftime(date_time, sizeof(date_time), "%d/%m/%y %H:%M",
             &local_time);
  }
  canvas.setTextDatum(textdatum_t::middle_right);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(kMuted);
  canvas.drawString(date_time, 310, 15);
}

void drawMainMetricValue() {
  canvas.fillRect(14, 62, 114, 96, kPanel);

  if (!metricReady(display_metric)) {
    canvas.setTextDatum(textdatum_t::middle_center);
    canvas.setFont(&fonts::Font4);
    canvas.setTextColor(kAmber);
    canvas.drawString("...", 71, 91);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(kMuted);
    canvas.drawString(display_metric == DisplayMetric::kCo2
                          ? "CO2 stabilizing"
                          : "Sensor stabilizing",
                      71, 126);
    canvas.drawString(display_metric == DisplayMetric::kCo2
                          ? "about 30 seconds"
                          : "about 3 minutes",
                      71, 140);
    return;
  }

  const float value = currentMetricValue(display_metric);
  const uint32_t color = metricColor(display_metric, value);
  char value_text[12] = {};
  formatMetricValue(display_metric, value, value_text, sizeof(value_text));
  canvas.setTextDatum(textdatum_t::middle_center);
  canvas.setFont(&fonts::Font7);
  canvas.setTextColor(color);
  canvas.drawString(value_text, 71, 88);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(kMuted);
  canvas.drawString(metricUnit(display_metric), 71, 125);

  canvas.fillRoundRect(18, 142, 106, 15, 7, color);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(kBackground);
  canvas.drawString(display_metric == DisplayMetric::kCo2
                        ? qualityLabel(current.co2)
                        : display_metric == DisplayMetric::kTemperature
                              ? "TEMPERATURE"
                              : "HUMIDITY",
                    71, 149);
}

void drawTrendRange() {
  canvas.fillRect(254, 39, 50, 18, kPanel);
  canvas.setTextDatum(textdatum_t::top_right);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(kMuted);
  canvas.drawString(trendRangeLabel(), 302, 45);
}

void drawGraphPlot() {
  constexpr int x = 142;
  constexpr int y = 31;
  constexpr int w = 170;
  constexpr int h = 133;

  constexpr int gx = x + 10;
  constexpr int gy = y + 35;
  constexpr int gw = w - 20;
  constexpr int gh = h - 46;
  canvas.fillRect(gx, gy, gw, gh + 1, kPanel);
  for (int i = 0; i <= 3; ++i) {
    const int line_y = gy + i * gh / 3;
    canvas.drawFastHLine(gx, line_y, gw, kPanelLight);
  }

  const bool short_range = trend_range == TrendRange::kTenMinutes;
  const size_t capacity = short_range ? config::kShortHistorySize
                                      : config::kLongHistorySize;
  const size_t available_count =
      short_range ? short_history_count : long_history_count;
  const size_t head = short_range ? short_history_head : long_history_head;
  size_t expected_count = config::kShortHistorySize;
  if (trend_range == TrendRange::kSixHours) expected_count = 360;
  if (trend_range == TrendRange::kTwelveHours) expected_count = 720;
  const size_t source_count = std::min(available_count, expected_count);
  const size_t source_start = available_count - source_count;

  if (source_count < 2) {
    canvas.setTextDatum(textdatum_t::middle_center);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(kMuted);
    canvas.drawString("Collecting history", x + w / 2, gy + gh / 2);
    return;
  }

  auto metric_value_at = [&](size_t index) -> float {
    if (display_metric == DisplayMetric::kCo2) {
      const uint16_t* values = short_range ? short_history : long_history;
      return ringValueAt(values, capacity, available_count, head, index);
    }
    const float* values =
        display_metric == DisplayMetric::kTemperature
            ? (short_range ? short_temperature_history
                           : long_temperature_history)
            : (short_range ? short_humidity_history
                           : long_humidity_history);
    return ringValueAt(values, capacity, available_count, head, index);
  };

  float min_value = display_metric == DisplayMetric::kCo2 ? 500.0f : 9999.0f;
  float max_value = display_metric == DisplayMetric::kCo2 ? 1400.0f : -9999.0f;
  size_t valid_count = 0;
  for (size_t i = 0; i < source_count; ++i) {
    const float value = metric_value_at(source_start + i);
    if (!isfinite(value)) continue;
    min_value = std::min(min_value, value);
    max_value = std::max(max_value, value);
    ++valid_count;
  }
  if (valid_count < 2) {
    canvas.setTextDatum(textdatum_t::middle_center);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(kMuted);
    canvas.drawString("Collecting history", x + w / 2, gy + gh / 2);
    return;
  }
  if (display_metric == DisplayMetric::kCo2) {
    min_value = std::floor(min_value / 100.0f) * 100.0f;
    max_value = std::ceil(max_value / 100.0f) * 100.0f;
    if (max_value <= min_value) max_value = min_value + 100.0f;
  } else {
    const float padding = std::max((max_value - min_value) * 0.15f, 1.0f);
    min_value = std::floor(min_value - padding);
    max_value = std::ceil(max_value + padding);
    if (max_value <= min_value) max_value = min_value + 1.0f;
  }

  bool have_previous = false;
  int previous_x = 0;
  int previous_y = 0;
  const size_t missing_count = expected_count - source_count;
  for (size_t i = 0; i < source_count; ++i) {
    const float value = metric_value_at(source_start + i);
    if (!isfinite(value)) {
      have_previous = false;
      continue;
    }
    const int px =
        gx + static_cast<int>((missing_count + i) * (gw - 1) /
                              (expected_count - 1));
    const int py = gy + gh -
                   static_cast<int>((value - min_value) * gh /
                                    (max_value - min_value));
    if (have_previous) {
      canvas.drawLine(previous_x, previous_y, px, py,
                      metricColor(display_metric, value));
    }
    previous_x = px;
    previous_y = py;
    have_previous = true;
  }
}

void drawMetricValue(int x, const char* value, const char* unit,
                     uint32_t accent) {
  canvas.fillRoundRect(x + 9, 182, 4, 38, 2, accent);
  canvas.fillRect(x + 18, 196, 122, 28, kPanel);
  canvas.setTextDatum(textdatum_t::top_left);
  canvas.setFont(&fonts::Font4);
  canvas.setTextColor(kWhite);
  canvas.drawString(value, x + 21, 197);
  const int value_width = canvas.textWidth(value);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(accent);
  canvas.drawString(unit, x + 24 + value_width, 204);
}

void drawDynamicValues() {
  display.startWrite();
  drawDateTime();
  drawMainMetricValue();
  drawTrendRange();
  drawGraphPlot();

  const DisplayMetric left_metric = leftBottomMetric();
  const DisplayMetric right_metric = rightBottomMetric();
  char left_value[12] = {};
  char right_value[12] = {};
  formatMetricValue(left_metric,
                    metricReady(left_metric) ? currentMetricValue(left_metric)
                                             : NAN,
                    left_value, sizeof(left_value));
  formatMetricValue(right_metric,
                    metricReady(right_metric) ? currentMetricValue(right_metric)
                                              : NAN,
                    right_value, sizeof(right_value));
  drawMetricValue(8, left_value, metricUnit(left_metric),
                  currentMetricColor(left_metric));
  drawMetricValue(164, right_value, metricUnit(right_metric),
                  currentMetricColor(right_metric));
  display.endWrite();
}

void drawLongTermScreen() {
  if (long_term_dirty) loadLongTermHistory();

  display.startWrite();
  canvas.fillScreen(kBackground);
  canvas.setTextDatum(textdatum_t::middle_left);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(kWhite);
  char title[32] = {};
  snprintf(title, sizeof(title), "LONG TERM %s", metricTitle(display_metric));
  canvas.drawString(title, 12, 15);
  canvas.setTextDatum(textdatum_t::middle_right);
  canvas.setTextColor(metricColor(display_metric, currentMetricValue(display_metric)));
  canvas.drawString(longTermRangeLabel(), 308, 15);

  drawRoundedPanel(8, 31, 304, 170);
  constexpr int gx = 20;
  constexpr int gy = 45;
  constexpr int gw = 280;
  constexpr int gh = 142;
  for (int i = 0; i <= 4; ++i) {
    canvas.drawFastHLine(gx, gy + i * gh / 4, gw, kPanelLight);
  }

  if (!sd_ready) {
    canvas.setTextDatum(textdatum_t::middle_center);
    canvas.setFont(&fonts::Font4);
    canvas.setTextColor(kAmber);
    canvas.drawString("NO SD CARD", 160, 105);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(kMuted);
    canvas.drawString("Insert a card, then reopen this screen", 160, 133);
  } else if (long_term_plot_count < 2) {
    canvas.setTextDatum(textdatum_t::middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(kMuted);
    canvas.drawString("No long-term readings yet", 160, 112);
  } else {
    uint16_t plot_min = 0;
    uint16_t plot_max = 0;
    if (display_metric == DisplayMetric::kCo2) {
      plot_min = static_cast<uint16_t>(
          (std::min<uint16_t>(long_term_min, 500) / 100) * 100);
      plot_max = static_cast<uint16_t>(
          ((std::max<uint16_t>(long_term_max, 1400) + 99) / 100) * 100);
      if (plot_max <= plot_min) plot_max = plot_min + 100;
    } else {
      const uint16_t padding = 10;
      plot_min = long_term_min > padding ? long_term_min - padding : 0;
      plot_max = long_term_max + padding;
      if (plot_max <= plot_min) plot_max = plot_min + 10;
    }

    bool have_previous = false;
    int previous_x = 0;
    int previous_y = 0;
    for (size_t i = 0; i < config::kLongTermPlotPoints; ++i) {
      const uint16_t value = long_term_plot[i];
      if (value == 0) {
        have_previous = false;
        continue;
      }
      const int px = gx + static_cast<int>(i);
      const int py =
          gy + gh -
          static_cast<int>((value - plot_min) * gh /
                           (plot_max - plot_min));
      if (have_previous) {
        canvas.drawLine(previous_x, previous_y, px, py,
                        metricColor(display_metric,
                                    unscaledMetricValue(display_metric, value)));
      }
      previous_x = px;
      previous_y = py;
      have_previous = true;
    }
  }

  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(kMuted);
  canvas.setTextDatum(textdatum_t::middle_left);
  char summary[48] = {};
  if (long_term_plot_count > 0) {
    if (display_metric == DisplayMetric::kCo2) {
      snprintf(summary, sizeof(summary), "AVG %u  MIN %u  MAX %u ppm",
               long_term_average, long_term_min, long_term_max);
    } else {
      snprintf(summary, sizeof(summary), "AVG %.1f  MIN %.1f  MAX %.1f %s",
               unscaledMetricValue(display_metric, long_term_average),
               unscaledMetricValue(display_metric, long_term_min),
               unscaledMetricValue(display_metric, long_term_max),
               metricUnit(display_metric));
    }
  } else {
    snprintf(summary, sizeof(summary), "Tap to change range");
  }
  canvas.drawString(summary, 12, 216);
  canvas.setTextDatum(textdatum_t::middle_right);
  canvas.drawString(long_term_last_timestamp[0] != '\0'
                        ? long_term_last_timestamp
                        : "SWIPE RIGHT",
                    308, 232);
  display.endWrite();
}

void drawSettingButton(int x, int y, int w, int h, const char* label,
                       uint32_t color = kBlue, bool selected = false) {
  canvas.fillRoundRect(x, y, w, h, 7, kPanelLight);
  canvas.drawRoundRect(x, y, w, h, 7, color);
  if (selected) {
    canvas.drawRoundRect(x - 2, y - 2, w + 4, h + 4, 9, kWhite);
  }
  canvas.setTextDatum(textdatum_t::middle_center);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(color);
  canvas.drawString(label, x + w / 2, y + h / 2);
}

void loadCalibrationSettings() {
  calibration_busy = true;
  snprintf(calibration_status, sizeof(calibration_status),
           "Reading sensor settings...");
  if (!sensor_ready ||
      !sensor.readSettings(asc_enabled, calibration_temperature_offset,
                           calibration_altitude)) {
    snprintf(calibration_status, sizeof(calibration_status),
             "Could not read sensor settings");
  } else {
    calibration_loaded = true;
    last_reading_ms = millis();
    resetStabilizationWindows();
    snprintf(calibration_status, sizeof(calibration_status),
             "Settings loaded");
  }
  calibration_busy = false;
}

void drawCalibrationScreen() {
  if (!calibration_loaded && !calibration_busy) {
    loadCalibrationSettings();
  }

  display.startWrite();
  canvas.fillScreen(kBackground);
  canvas.setTextDatum(textdatum_t::middle_left);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(kWhite);
  canvas.drawString("SCD40 CALIBRATION", 12, 15);
  drawRoundedPanel(28, 30, 264, 181);
  char label[40] = {};
  snprintf(label, sizeof(label), "AUTO CALIBRATION: %s",
           asc_enabled ? "ON" : "OFF");
  drawSettingButton(
      48, 36, 224, 28, label, asc_enabled ? kGreen : kAmber,
      calibration_item == CalibrationItem::kAutomaticCalibration);
  snprintf(label, sizeof(label), "TEMP OFFSET: %.1f C",
           calibration_temperature_offset);
  drawSettingButton(48, 70, 224, 28, label, kAmber,
                    calibration_item == CalibrationItem::kTemperatureOffset);
  snprintf(label, sizeof(label), "FRC REFERENCE: %u ppm",
           calibration_target);
  drawSettingButton(48, 104, 224, 28, label, kBlue,
                    calibration_item == CalibrationItem::kReference);

  const uint32_t frc_color = calibration_armed ? kRed : kAmber;
  drawSettingButton(
      48, 138, 224, 28,
      calibration_armed ? "HOLD AGAIN TO CALIBRATE" : "FORCED CALIBRATION",
      frc_color, calibration_item == CalibrationItem::kForcedCalibration);
  drawSettingButton(48, 172, 224, 28, "EXIT", kMuted,
                    calibration_item == CalibrationItem::kExit);

  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(kMuted);
  canvas.setTextDatum(textdatum_t::middle_center);
  canvas.drawString(calibration_status, 160, 216);
  canvas.drawString("Short press: select | Hold: activate", 160, 232);
  display.endWrite();
}

void finishCalibrationAction(bool ok, const char* success_message) {
  last_reading_ms = millis();
  resetStabilizationWindows();
  calibration_armed = false;
  snprintf(calibration_status, sizeof(calibration_status), "%s",
           ok ? success_message : "Sensor command failed");
  drawCalibrationScreen();
}

void activateCalibrationItem() {
  if (calibration_busy || !calibration_loaded) return;

  switch (calibration_item) {
    case CalibrationItem::kAutomaticCalibration: {
      calibration_busy = true;
      const bool next = !asc_enabled;
      const bool ok = sensor.setAutomaticSelfCalibration(next);
      if (ok) asc_enabled = next;
      calibration_busy = false;
      finishCalibrationAction(ok, asc_enabled ? "ASC enabled"
                                              : "ASC disabled");
      break;
    }
    case CalibrationItem::kTemperatureOffset: {
      float next = calibration_temperature_offset + 0.5f;
      if (next > 10.0f) next = 0.0f;
      calibration_busy = true;
      const bool ok = sensor.setTemperatureOffset(next);
      if (ok) calibration_temperature_offset = next;
      calibration_busy = false;
      char result[48] = {};
      snprintf(result, sizeof(result), "Temperature offset %.1f C",
               calibration_temperature_offset);
      finishCalibrationAction(ok, result);
      break;
    }
    case CalibrationItem::kReference:
      calibration_target =
          calibration_target == 400 ? 420 :
          calibration_target == 420 ? 450 : 400;
      calibration_armed = false;
      snprintf(calibration_status, sizeof(calibration_status),
               "Reference changed to %u ppm", calibration_target);
      drawCalibrationScreen();
      break;
    case CalibrationItem::kForcedCalibration: {
      if (!calibration_armed) {
        calibration_armed = true;
        snprintf(calibration_status, sizeof(calibration_status),
                 "Stable air 3+ min, then hold again");
        drawCalibrationScreen();
        break;
      }

      calibration_busy = true;
      int16_t correction = 0;
      const bool ok =
          sensor.performForcedRecalibration(calibration_target, correction);
      calibration_busy = false;
      current.valid = false;
      consecutive_sensor_misses = 0;
      last_reading_ms = millis();
      char result[48] = {};
      snprintf(result, sizeof(result), "FRC complete: correction %+d ppm",
               correction);
      finishCalibrationAction(ok, result);
      break;
    }
    case CalibrationItem::kExit:
      calibration_armed = false;
      display_page = DisplayPage::kLongTerm;
      drawLongTermScreen();
      Serial.println("Display page: long-term history");
      break;
    default:
      break;
  }
}

void selectNextCalibrationItem() {
  const uint8_t next =
      (static_cast<uint8_t>(calibration_item) + 1) %
      static_cast<uint8_t>(CalibrationItem::kCount);
  calibration_item = static_cast<CalibrationItem>(next);
  calibration_armed = false;
  snprintf(calibration_status, sizeof(calibration_status),
           "Hold BOOT to activate");
  drawCalibrationScreen();
}

void enterCalibrationScreen() {
  display_page = DisplayPage::kCalibration;
  calibration_armed = false;
  calibration_item = CalibrationItem::kAutomaticCalibration;
  snprintf(calibration_status, sizeof(calibration_status),
           "Short press BOOT to select");
  drawCalibrationScreen();
  Serial.println("Display page: calibration");
}

void cycleDisplayMetric() {
  const uint8_t next =
      (static_cast<uint8_t>(display_metric) + 1) %
      static_cast<uint8_t>(DisplayMetric::kCount);
  display_metric = static_cast<DisplayMetric>(next);
  preferences.putUChar("metric", next);
  long_term_dirty = true;

  if (display_page == DisplayPage::kDashboard) {
    drawStaticDashboard();
    drawDynamicValues();
  } else if (display_page == DisplayPage::kLongTerm) {
    drawLongTermScreen();
  }
  Serial.printf("Display metric: %s\n", metricTitle(display_metric));
}

void handleBootButton() {
  const bool raw_down = digitalRead(config::kBootButtonPin) == LOW;
  const unsigned long now = millis();

  if (raw_down != boot_button_raw_down) {
    boot_button_raw_down = raw_down;
    boot_button_changed_ms = now;
  }

  if (now - boot_button_changed_ms >= config::kBootButtonDebounceMs &&
      raw_down != boot_button_down) {
    boot_button_down = raw_down;
    if (boot_button_down) {
      boot_button_pressed_ms = now;
      boot_button_long_action = false;
    } else if (display_page == DisplayPage::kCalibration &&
               !boot_button_long_action) {
      selectNextCalibrationItem();
    } else if (display_page == DisplayPage::kDashboard &&
               !boot_button_long_action) {
      cycleDisplayMetric();
    }
  }

  if ((display_page == DisplayPage::kCalibration ||
       display_page == DisplayPage::kLongTerm) &&
      boot_button_down && !boot_button_long_action &&
      now - boot_button_pressed_ms >= config::kBootButtonLongPressMs) {
    boot_button_long_action = true;
    if (display_page == DisplayPage::kLongTerm) {
      enterCalibrationScreen();
    } else {
      activateCalibrationItem();
    }
  }
}

void handleTouch() {
  uint16_t x = 0;
  uint16_t y = 0;
  const bool touch_is_down = display.getTouch(&x, &y);

  if (touch_is_down) {
    if (!touch_was_down) {
      touch_start_x = x;
      touch_start_y = y;
    }
    last_touch_x = x;
    last_touch_y = y;
  }

  if (!touch_is_down && touch_was_down) {
    const int delta_x =
        static_cast<int>(last_touch_x) - static_cast<int>(touch_start_x);
    const int delta_y =
        static_cast<int>(last_touch_y) - static_cast<int>(touch_start_y);
    const bool horizontal_swipe =
        abs(delta_x) >= config::kSwipeThreshold &&
        abs(delta_x) > abs(delta_y);

    if (horizontal_swipe && delta_x < 0 &&
        display_page == DisplayPage::kDashboard) {
      display_page = DisplayPage::kLongTerm;
      long_term_dirty = true;
      drawLongTermScreen();
      Serial.println("Display page: long-term history");
    } else if (horizontal_swipe && delta_x > 0 &&
               display_page == DisplayPage::kLongTerm) {
      display_page = DisplayPage::kDashboard;
      drawStaticDashboard();
      drawDynamicValues();
      Serial.println("Display page: dashboard");
    } else if (horizontal_swipe && delta_x > 0 &&
               display_page == DisplayPage::kCalibration) {
      display_page = DisplayPage::kLongTerm;
      calibration_armed = false;
      drawLongTermScreen();
      Serial.println("Display page: long-term history");
    } else if (display_page == DisplayPage::kDashboard &&
               last_touch_x >= 142 && last_touch_x < 312 &&
        last_touch_y >= 31 && last_touch_y < 164) {
      const uint8_t next =
          (static_cast<uint8_t>(trend_range) + 1) %
          static_cast<uint8_t>(TrendRange::kCount);
      trend_range = static_cast<TrendRange>(next);
      preferences.putUChar("range", next);
      display.startWrite();
      drawTrendRange();
      drawGraphPlot();
      display.endWrite();
      Serial.printf("Trend range: %s\n", trendRangeLabel());
    } else if (display_page == DisplayPage::kLongTerm) {
      const uint8_t next =
          (static_cast<uint8_t>(long_term_range) + 1) %
          static_cast<uint8_t>(LongTermRange::kCount);
      long_term_range = static_cast<LongTermRange>(next);
      long_term_dirty = true;
      drawLongTermScreen();
      Serial.printf("Long-term range: %s\n", longTermRangeLabel());
    }
  }
  touch_was_down = touch_is_down;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nESP32 CO2 Dashboard");

  const bool display_ok = display.init();
  display.setRotation(0);
  pinMode(config::kBacklightPin, OUTPUT);
  pinMode(config::kBootButtonPin, INPUT_PULLUP);
  digitalWrite(config::kBacklightPin, HIGH);
  Serial.printf("Display init: %s, size: %d x %d\n",
                display_ok ? "OK" : "FAILED", display.width(), display.height());
  display.setColorDepth(16);
  display.fillScreen(kBackground);

  preferences.begin("co2dash", false);
  boot_id = preferences.getUInt("boot_id", 0) + 1;
  preferences.putUInt("boot_id", boot_id);
  const uint8_t saved_range = preferences.getUChar("range", 0);
  if (saved_range < static_cast<uint8_t>(TrendRange::kCount)) {
    trend_range = static_cast<TrendRange>(saved_range);
  }
  const uint8_t saved_metric = preferences.getUChar("metric", 0);
  if (saved_metric < static_cast<uint8_t>(DisplayMetric::kCount)) {
    display_metric = static_cast<DisplayMetric>(saved_metric);
  }
  display.setColorOrderRgb(preferences.getUChar("color_order", 0) != 0);
  loadHistory();
  display.startWrite();
  drawStaticDashboard();
  display.endWrite();
  drawDynamicValues();

  startSensor();
  initializeCalibrationDefaults();
  setupWifiAndTime();
  beginSdCard();
}

void loop() {
  digitalWrite(config::kBacklightPin, HIGH);
  serviceWifiAndTime();
  serviceWebDashboard();
  handleTouch();
  handleBootButton();

  const unsigned long now = millis();
  if (display_page == DisplayPage::kDashboard) {
    struct tm local_time = {};
    const int current_minute =
        getLocalTime(&local_time, 0) ? local_time.tm_min : -1;
    if (current_minute != last_clock_minute) {
      last_clock_minute = current_minute;
      display.startWrite();
      drawDateTime();
      display.endWrite();
    }
  }
  if (!sensor_ready) {
    if (now - last_sensor_attempt_ms >= config::kReadingIntervalMs) {
      startSensor();
      initializeCalibrationDefaults();
    }
    delay(20);
    return;
  }
  const uint32_t poll_interval =
      current.valid ? config::kReadingIntervalMs
                    : config::kSensorWarmupPollMs;
  if (now - last_reading_ms < poll_interval) {
    delay(20);
    return;
  }
  last_reading_ms = now;

  uint16_t co2 = 0;
  float temperature = NAN;
  float humidity = NAN;
  bool has_new_reading = false;
  if (sensor.readIfReady(co2, temperature, humidity)) {
    consecutive_sensor_misses = 0;
    sensor_recovery_attempts = 0;
    current.co2 = co2;
    current.temperature = temperature;
    current.humidity = humidity;
    current.valid = true;
    co2_reading_valid =
        static_cast<int32_t>(millis() - co2_ready_ms) >= 0;
    environmental_reading_valid =
        static_cast<int32_t>(millis() - environmental_logging_ready_ms) >= 0;
    if (co2_reading_valid) {
      addHistory(co2, temperature, humidity);
    }
    has_new_reading = true;
    Serial.printf("CO2: %u ppm, temperature: %.1f C, humidity: %.1f %%\n",
                  co2, temperature, humidity);
  } else if (++consecutive_sensor_misses >=
             config::kSensorMissesBeforeRestart) {
    Serial.println("SCD4x not producing readings; restarting measurement");
    current.valid = false;
    co2_reading_valid = false;
    environmental_reading_valid = false;
    sensor_ready = false;
    consecutive_sensor_misses = 0;
    ++sensor_recovery_attempts;
    if (display_page == DisplayPage::kDashboard) {
      drawDynamicValues();
    }
  }

  if (has_new_reading) {
    if (display_page == DisplayPage::kDashboard) {
      drawDynamicValues();
    } else if (display_page == DisplayPage::kLongTerm &&
               long_term_dirty) {
      drawLongTermScreen();
    }
  }
  delay(20);
}
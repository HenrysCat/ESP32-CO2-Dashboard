#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <Preferences.h>
#include <SdFat.h>
#include <WiFiManager.h>
#include <Wire.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iterator>
#include <time.h>

namespace config {

constexpr int kBacklightPin = 21;
constexpr int kI2cSdaPin = 27;
constexpr int kI2cSclPin = 22;
constexpr uint8_t kScd4xAddress = 0x62;
constexpr uint32_t kReadingIntervalMs = 10000;
constexpr size_t kShortHistorySize = 60;
constexpr size_t kLongHistorySize = 720;
constexpr uint8_t kReadingsPerMinute = 6;
constexpr uint8_t kMinutesPerSave = 10;
constexpr int kSdCsPin = 5;
constexpr int kSdMosiPin = 23;
constexpr int kSdMisoPin = 19;
constexpr int kSdSckPin = 18;
constexpr uint32_t kSdRetryIntervalMs = 60000;
constexpr char kLogFileName[] = "co2log.csv";
constexpr char kConfigPortalName[] = "CO2-Dashboard-Setup";
constexpr char kDefaultTimezone[] = "GMT0BST,M3.5.0/1,M10.5.0";
constexpr char kNtpServer1[] = "pool.ntp.org";
constexpr char kNtpServer2[] = "time.cloudflare.com";
constexpr size_t kLongTermPlotPoints = 280;
constexpr size_t kSdReadBlockSize = 512;
constexpr int kSwipeThreshold = 60;

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
};

class Scd4x {
 public:
  bool begin() {
    Wire.begin(config::kI2cSdaPin, config::kI2cSclPin);
    Wire.setClock(100000);

    // Stop a measurement that may have survived a soft reset.
    sendCommand(0x3F86);
    delay(500);
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
size_t short_history_count = 0;
size_t short_history_head = 0;
uint16_t long_history[config::kLongHistorySize] = {};
size_t long_history_count = 0;
size_t long_history_head = 0;
uint32_t minute_sum = 0;
float minute_temperature_sum = 0.0f;
float minute_humidity_sum = 0.0f;
uint8_t minute_sample_count = 0;
uint8_t minutes_since_save = 0;
unsigned long last_reading_ms = 0;
unsigned long last_sensor_attempt_ms = 0;
unsigned long last_sd_attempt_ms = 0;
uint32_t boot_id = 0;
bool sensor_ready = false;
bool sd_ready = false;
bool time_configured = false;
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

DisplayPage display_page = DisplayPage::kDashboard;
TrendRange trend_range = TrendRange::kTenMinutes;
LongTermRange long_term_range = LongTermRange::kDay;
uint16_t long_term_plot[config::kLongTermPlotPoints] = {};
size_t long_term_plot_count = 0;
uint16_t long_term_min = 0;
uint16_t long_term_max = 0;
uint16_t long_term_average = 0;
char long_term_last_timestamp[24] = {};
bool long_term_dirty = true;

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
  wifi_manager.setConfigPortalTimeout(300);
  wifi_manager.setConfigPortalBlocking(false);
  wifi_manager.setWiFiAutoReconnect(true);
  wifi_manager.setSaveParamsCallback(savePortalSettings);

  Serial.printf("Connecting to Wi-Fi or starting portal \"%s\"\n",
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

void startSensor() {
  last_sensor_attempt_ms = millis();
  sensor_ready = sensor.begin();
  Serial.printf("SCD4x init: %s\n", sensor_ready ? "OK" : "FAILED");
}

bool formatLocalTimestamp(char* output, size_t output_size) {
  struct tm local_time = {};
  if (!getLocalTime(&local_time, 0)) return false;
  return strftime(output, output_size, "%Y-%m-%dT%H:%M:%S",
                  &local_time) > 0;
}

bool beginSdCard() {
  last_sd_attempt_ms = millis();
  sd_ready = sd.begin(
      SdSpiConfig(config::kSdCsPin, DEDICATED_SPI, SD_SCK_MHZ(0), &sd_spi));
  if (!sd_ready) {
    Serial.println("SD card unavailable; logging will retry in one minute");
    return false;
  }

  FsFile log_file;
  if (!log_file.open(config::kLogFileName,
                     O_WRONLY | O_CREAT | O_APPEND)) {
    Serial.println("Could not open co2log.csv");
    sd_ready = false;
    return false;
  }
  if (log_file.fileSize() == 0) {
    log_file.println(
        "boot_id,uptime_seconds,co2_ppm,temperature_c,humidity_percent,"
        "local_timestamp");
  }
  log_file.close();
  Serial.println("SD logging ready: co2log.csv");
  return true;
}

void logMinuteReading(uint16_t co2, float temperature, float humidity) {
  if (!sd_ready &&
      millis() - last_sd_attempt_ms >= config::kSdRetryIntervalMs) {
    beginSdCard();
  }
  if (!sd_ready) return;

  FsFile log_file;
  if (!log_file.open(config::kLogFileName,
                     O_WRONLY | O_CREAT | O_APPEND)) {
    Serial.println("SD log open failed; logging paused");
    sd_ready = false;
    return;
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
  Serial.println("Saved one-minute average to SD card");
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

void addShortHistory(uint16_t co2) {
  short_history[short_history_head] = co2;
  short_history_head =
      (short_history_head + 1) % config::kShortHistorySize;
  short_history_count =
      std::min(short_history_count + 1, config::kShortHistorySize);
}

void addLongHistory(uint16_t co2, float temperature, float humidity) {
  long_history[long_history_head] = co2;
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
  addShortHistory(co2);
  minute_sum += co2;
  minute_temperature_sum += temperature;
  minute_humidity_sum += humidity;
  if (++minute_sample_count >= config::kReadingsPerMinute) {
    addLongHistory(
        static_cast<uint16_t>(minute_sum / config::kReadingsPerMinute),
        minute_temperature_sum / config::kReadingsPerMinute,
        minute_humidity_sum / config::kReadingsPerMinute);
    minute_sum = 0;
    minute_temperature_sum = 0.0f;
    minute_humidity_sum = 0.0f;
    minute_sample_count = 0;
  }
}

uint16_t ringValueAt(const uint16_t* values, size_t capacity, size_t count,
                     size_t head, size_t index) {
  const size_t oldest =
      (head + capacity - count) % capacity;
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
  uint8_t buffer[config::kSdReadBlockSize] = {};

  while (position > 0) {
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

void loadLongTermHistory() {
  std::fill(std::begin(long_term_plot), std::end(long_term_plot), 0);
  long_term_plot_count = 0;
  long_term_min = 0;
  long_term_max = 0;
  long_term_average = 0;
  long_term_last_timestamp[0] = '\0';
  long_term_dirty = false;

  if (!sd_ready && !beginSdCard()) return;

  FsFile log_file;
  if (!log_file.open(config::kLogFileName, O_RDONLY)) {
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
    char* timestamp = nullptr;
    if (!parseLogRow(line, co2, timestamp)) continue;

    size_t bucket =
        (missing_rows + row_index) * config::kLongTermPlotPoints /
        expected_rows;
    bucket = std::min(bucket, config::kLongTermPlotPoints - 1);
    bucket_sum[bucket] += co2;
    ++bucket_count[bucket];
    total += co2;
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
  canvas.drawString("AIR QUALITY", 12, 15);

  drawRoundedPanel(8, 31, 126, 133);
  canvas.setTextDatum(textdatum_t::top_left);
  canvas.setTextColor(kMuted);
  canvas.drawString("CO2", 19, 42);

  drawRoundedPanel(142, 31, 170, 133);
  canvas.drawString("CO2 TREND", 152, 41);

  drawRoundedPanel(8, 172, 148, 58);
  canvas.fillRoundRect(17, 182, 4, 38, 2, kAmber);
  canvas.setTextDatum(textdatum_t::top_left);
  canvas.setTextColor(kMuted);
  canvas.drawString("TEMPERATURE", 30, 182);

  drawRoundedPanel(164, 172, 148, 58);
  canvas.fillRoundRect(173, 182, 4, 38, 2, kBlue);
  canvas.drawString("HUMIDITY", 186, 182);
}

void drawStatus() {
  canvas.fillRect(240, 5, 72, 20, kBackground);
  const uint32_t dot = current.valid ? qualityColor(current.co2) : kAmber;
  canvas.fillCircle(252, 15, 4, dot);
  canvas.setTextDatum(textdatum_t::middle_right);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(kMuted);
  canvas.drawString(current.valid ? "LIVE" : "WARMING", 310, 15);
}

void drawCo2Value() {
  canvas.fillRect(14, 62, 114, 96, kPanel);

  if (!current.valid) {
    canvas.setTextDatum(textdatum_t::middle_center);
    canvas.setFont(&fonts::Font4);
    canvas.setTextColor(kAmber);
    canvas.drawString("...", 71, 91);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(kMuted);
    canvas.drawString("First reading takes", 71, 126);
    canvas.drawString("about 10 seconds", 71, 140);
    return;
  }

  const uint32_t color = qualityColor(current.co2);
  canvas.setTextDatum(textdatum_t::middle_center);
  canvas.setFont(&fonts::Font7);
  canvas.setTextColor(color);
  canvas.drawNumber(current.co2, 71, 88);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(kMuted);
  canvas.drawString("ppm", 71, 125);

  canvas.fillRoundRect(18, 142, 106, 15, 7, color);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(kBackground);
  canvas.drawString(qualityLabel(current.co2), 71, 149);
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
  const uint16_t* values = short_range ? short_history : long_history;
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

  uint16_t min_value = 500;
  uint16_t max_value = 1400;
  for (size_t i = 0; i < source_count; ++i) {
    const uint16_t value =
        ringValueAt(values, capacity, available_count, head, source_start + i);
    min_value = std::min(min_value, value);
    max_value = std::max(max_value, value);
  }
  min_value = static_cast<uint16_t>((min_value / 100) * 100);
  max_value = static_cast<uint16_t>(((max_value + 99) / 100) * 100);
  if (max_value <= min_value) max_value = min_value + 100;

  int previous_x = gx;
  int previous_y = gy + gh;
  const size_t missing_count = expected_count - source_count;
  for (size_t i = 0; i < source_count; ++i) {
    const uint16_t value =
        ringValueAt(values, capacity, available_count, head, source_start + i);
    const int px =
        gx + static_cast<int>((missing_count + i) * (gw - 1) /
                              (expected_count - 1));
    const int py = gy + gh -
                   static_cast<int>((value - min_value) * gh /
                                    (max_value - min_value));
    if (i > 0) {
      canvas.drawLine(previous_x, previous_y, px, py, qualityColor(value));
    }
    previous_x = px;
    previous_y = py;
  }
}

void drawMetricValue(int x, const char* value, const char* unit,
                     uint32_t accent) {
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
  drawStatus();
  drawCo2Value();
  drawTrendRange();
  drawGraphPlot();

  char temperature[12] = "--.-";
  char humidity[12] = "--.-";
  if (current.valid) {
    snprintf(temperature, sizeof(temperature), "%.1f", current.temperature);
    snprintf(humidity, sizeof(humidity), "%.1f", current.humidity);
  }
  drawMetricValue(8, temperature, "C", kAmber);
  drawMetricValue(164, humidity, "%", kBlue);
  display.endWrite();
}

void drawLongTermScreen() {
  if (long_term_dirty) loadLongTermHistory();

  display.startWrite();
  canvas.fillScreen(kBackground);
  canvas.setTextDatum(textdatum_t::middle_left);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(kWhite);
  canvas.drawString("LONG TERM CO2", 12, 15);
  canvas.setTextDatum(textdatum_t::middle_right);
  canvas.setTextColor(kBlue);
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
    uint16_t plot_min =
        static_cast<uint16_t>((std::min<uint16_t>(long_term_min, 500) /
                               100) *
                              100);
    uint16_t plot_max =
        static_cast<uint16_t>(((std::max<uint16_t>(long_term_max, 1400) +
                                99) /
                               100) *
                              100);
    if (plot_max <= plot_min) plot_max = plot_min + 100;

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
                        qualityColor(value));
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
    snprintf(summary, sizeof(summary), "AVG %u  MIN %u  MAX %u ppm",
             long_term_average, long_term_min, long_term_max);
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
  loadHistory();
  display.startWrite();
  drawStaticDashboard();
  display.endWrite();
  drawDynamicValues();

  startSensor();
  setupWifiAndTime();
  beginSdCard();
}

void loop() {
  digitalWrite(config::kBacklightPin, HIGH);
  serviceWifiAndTime();
  handleTouch();

  const unsigned long now = millis();
  if (!sensor_ready) {
    if (now - last_sensor_attempt_ms >= config::kReadingIntervalMs) {
      startSensor();
    }
    delay(20);
    return;
  }
  if (now - last_reading_ms < config::kReadingIntervalMs) {
    delay(20);
    return;
  }
  last_reading_ms = now;

  uint16_t co2 = 0;
  float temperature = NAN;
  float humidity = NAN;
  bool has_new_reading = false;
  if (sensor.readIfReady(co2, temperature, humidity)) {
    current.co2 = co2;
    current.temperature = temperature;
    current.humidity = humidity;
    current.valid = true;
    addHistory(co2, temperature, humidity);
    has_new_reading = true;
    Serial.printf("CO2: %u ppm, temperature: %.1f C, humidity: %.1f %%\n",
                  co2, temperature, humidity);
  }

  if (has_new_reading) {
    if (display_page == DisplayPage::kDashboard) {
      drawDynamicValues();
    } else if (long_term_dirty) {
      drawLongTermScreen();
    }
  }
  delay(20);
}

/*
  ================================================================
  MAGNIS v1 — Ground Station Receiver + Live Dashboard
  Ignitia Rocket Lab · Tecnológico de Monterrey GDL

  PRELIMINARY VERSION — NOT TESTED.
  This sketch was adapted from the previous ground station code
  (IGNIS_GroundStation_Heavy_3.ino), which ran on an IGNIS v2 board.
  It has NOT been compiled against or run on MAGNIS hardware yet,
  because the MAGNIS v1 board is not finished. Expect pin, library
  and timing fixes during bring-up.

  Hardware: schematic MAGNIS-v1-SCH-01 rev 1.1 (Option B layout,
  pin swaps of Oct 30, 2026).

  What it does
    - Two RFM95W receivers on one shared VSPI bus, listening to the
      same link profile (antenna diversity). Every packet is
      de-duplicated by its sequence number and the copy with the best
      RSSI is kept. Per-radio statistics show the diversity gain.
    - If one radio fails at boot, the station keeps working with the
      other one.
    - Logs every unique packet to microSD (onboard slot, or the
      external module on J2 as a backup path).
    - Onboard GPS: station position, range / bearing / elevation to
      the rocket (for aiming the Yagi), and UTC time disciplined by
      the PPS pulse (PC SETTIME:<epoch> kept as a fallback).
    - Wi-Fi access point with a live web dashboard for a phone
      (no internet needed): http://192.168.4.1
    - Serial dashboard + machine-readable "TLM," lines for the laptop.
    - 3x WS2812B status LEDs, MLT-8530 buzzer (PWM tone), multi-purpose
      button (input only; actions are left as hooks), I2C expansion
      header (scanned at boot).

  Kept from IGNIS_GroundStation_Heavy_3: TelemetryPacket layout, PC
  time-sync protocol (REQ_TIME / SETTIME:<epoch>), dashboard style.

  Arduino IDE (Tools menu): Board "ESP32 Dev Module", Flash Size 4MB,
  PSRAM Disabled, Partition "Default 4MB with spiffs". Upload over the
  DevKit micro-USB (CP2102).
  Libraries: RadioLib (jgromes, 7.x), Adafruit NeoPixel, TinyGPSPlus.
  WiFi, WebServer, SD, SPI and Wire come with the ESP32 core.

  Radio config and TelemetryPacket MUST MATCH the flight computer
  firmware exactly. The struct below is copied from the IGNIS v2 GS —
  re-check it against the FC firmware you will receive from.
  The FC transmits with RadioHead, which adds a 4-byte header
  (to, from, id, flags) in front of the payload; this sketch accepts
  packets with or without that header.
  ================================================================
*/

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <RadioLib.h>
#include <Adafruit_NeoPixel.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <WebServer.h>
#include <time.h>
#include <math.h>
#include "soc/gpio_sig_map.h"   // VSPIQ_IN_IDX, to re-route MISO for the external SD module

// ================================================================
// MAGNIS v1 pin map (schematic MAGNIS-v1-SCH-01 rev 1.1)
// ================================================================
// Shared SPI bus — native VSPI pins
#define PIN_SPI_SCK        18   // SNS_SPI_SCK
#define PIN_SPI_MISO       19   // SNS_SPI_MISO (both radios + onboard slot)
#define PIN_SPI_MOSI       23   // SNS_SPI_MOSI

// Radio 1 — U2, SMA J1 (ANT1)
#define PIN_R1_CS          21   // RFM95W_CS
#define PIN_R1_RST         25   // RFM95W_RESET
#define PIN_R1_DIO0        32   // RFM95W_INT

// Radio 2 — U3, SMA J3 (ANT2)
#define PIN_R2_CS          22   // RFM95W_CS_2
#define PIN_R2_RST         33   // RFM95W_RESET_2
#define PIN_R2_DIO0        34   // RFM95W_INT_2 (input-only pin, interrupt-safe)

// microSD
#define PIN_SD_CS          16   // SD_CS — onboard slot (CARD1), pull-up R5
#define PIN_SD_CS_EXT       4   // SD_CS_EXT — external module header J2
#define PIN_SD_MISO_EXT    39   // SD_MISO_EXT — input-only pin

// GPS — UART2
#define PIN_GPS_RX         17   // ESP32 RX <- GPS TX (net GPS_UART_TX)
#define PIN_GPS_TX          5   // ESP32 TX -> GPS RX (net GPS_UART_RX)
#define PIN_GPS_PPS        35   // GPS_PPS, input-only, rising edge = start of the UTC second

// User interface
#define PIN_WS2812B        14   // WS2812B_DATA (33 Ω R2 at LED1)
#define PIN_BUZZER         13   // BUZZER -> Q1 gate (NTS4001), R3 pull-down
#define PIN_BUTTON         36   // BTN, input-only, external pull-up R9, pressed = LOW
#define PIN_I2C_SDA        26   // I2C_SDA, expansion header J4
#define PIN_I2C_SCL        27   // I2C_SCL, expansion header J4

// ================================================================
// Options
// ================================================================
#define ENABLE_WIFI_DASHBOARD  1
#define WIFI_AP_SSID           "MAGNIS-GS"
#define WIFI_AP_PASS           "ignitia2026"   // 8+ characters
#define ENABLE_I2C_SCAN        1
#define SERIAL_DASHBOARD       1               // redrawn text dashboard
#define SERIAL_TLM_LINES       1               // one "TLM,<csv row>" line per unique packet

#define SD_USE_EXT_MODULE      0               // 0 = onboard slot (CARD1), 1 = external module (J2)
#define SD_SPI_FREQ_HZ         4000000
#define SD_MOUNT_RETRY_MS      10000

#define RADIO_SPI_FREQ_HZ      8000000
#define MERGE_WINDOW_MS        40              // both radios decode the same packet within a few ms
#define GPS_BAUD               9600
#define GS_TIMEZONE            "CST6"          // Mexico Central, UTC-6, no DST
#define TIME_REQ_INTERVAL_MS   3000
#define GPS_RESYNC_MS          10000
#define DASHBOARD_INTERVAL_MS  500
#define LINK_STALE_MS          5000
#define LED_BRIGHTNESS         40
#define BUZZER_FREQ_HZ         2700            // MLT-8530 resonant frequency
#define BUTTON_LONG_MS         1500

// ================================================================
// Link profiles — MUST MATCH the transmitter firmware.
// TODO: for LASC (Brazil) move both channels fully inside 915–928 MHz;
// 915.0 MHz with BW125 spills below 915 MHz.
// ================================================================
struct LinkProfile {
  const char* name;
  float   freqMhz;
  float   bwKhz;
  uint8_t sf;
  uint8_t crDenom;
  uint8_t syncWord;
};
static const LinkProfile PROFILE_ROCKET = { "ROCKET", 915.0, 125.0, 9, 5, 0x12 };
static const LinkProfile PROFILE_CANSAT = { "CANSAT", 915.5, 125.0, 9, 5, 0xAB };
static LinkProfile linkCfg = PROFILE_ROCKET;   // "link" would clash with POSIX link()

// ================================================================
// Packet struct — MUST MATCH the FC's TelemetryPacket byte-for-byte.
// ================================================================
struct __attribute__((packed)) TelemetryPacket {
  uint32_t seq;
  uint32_t timestamp_ms;
  uint8_t  state;

  float    pressure_pa;
  float    baro_alt_m;
  float    agl_m;
  float    baro_temp_c;
  uint8_t  baro_valid;

  float    sht_temp_c;
  float    humidity_pct;

  float    accel_x_g;
  float    accel_y_g;
  float    accel_z_g;
  float    accel_total_g;
  float    gyro_x_dps;
  float    gyro_y_dps;
  float    gyro_z_dps;
  float    mpu_temp_c;
  uint8_t  imu_valid;

  double   gps_lat;
  double   gps_lon;
  float    gps_alt_m;
  uint8_t  gps_sats;
  uint8_t  gps_fix;

  float    battery_v;

  uint8_t  pyro1_continuity;
  uint8_t  pyro2_continuity;
  uint8_t  pyro1_fired;
  uint8_t  pyro2_fired;

  uint8_t  sd_logging;
};
#define RH_HEADER_LEN 4   // RadioHead adds to/from/id/flags in front of the payload

// ---- All types are declared here, before any function, so the Arduino
// ---- prototype generator can see them.
struct RadioCh {
  const char* name;
  const char* antenna;
  SX1276*     lora;
  uint8_t     csPin;
  void      (*isr)(void);
  bool        ok;
  uint32_t    rxCount;      // valid packets received by this radio
  uint32_t    crcErrors;
  uint32_t    sizeErrors;
  int         lastRssi;
  float       lastSnr;
  uint32_t    lastRxMs;
};

// Diversity merge buffer
struct Pending {
  bool            active;
  uint32_t        seq;
  TelemetryPacket pkt;
  uint8_t         mask;      // bit0 = R1, bit1 = R2
  int             rssi[2];
  float           snr[2];
  uint32_t        firstMs;
};

enum TimeSource : uint8_t { TS_NONE = 0, TS_PC = 1, TS_GPS = 2, TS_GPS_PPS = 3 };

static const char* FLIGHT_STATE_NAMES[] = {
  "PAD_IDLE", "ARMED", "BOOST", "COAST", "APOGEE",
  "DROGUE_DESCENT", "MAIN_DESCENT", "LANDED"
};
static const char* flightStateName(uint8_t s) {
  return (s < 8) ? FLIGHT_STATE_NAMES[s] : "UNKNOWN";
}

// ================================================================
// Radios
// ================================================================
static SPISettings radioSpi(RADIO_SPI_FREQ_HZ, MSBFIRST, SPI_MODE0);
static SX1276 lora1 = new Module(PIN_R1_CS, PIN_R1_DIO0, PIN_R1_RST, RADIOLIB_NC, SPI, radioSpi);
static SX1276 lora2 = new Module(PIN_R2_CS, PIN_R2_DIO0, PIN_R2_RST, RADIOLIB_NC, SPI, radioSpi);

static volatile bool rxFlag[2] = { false, false };
static void IRAM_ATTR isrRadio1() { rxFlag[0] = true; }
static void IRAM_ATTR isrRadio2() { rxFlag[1] = true; }

static RadioCh radios[2] = {
  { "R1", "J1 ANT1", &lora1, PIN_R1_CS, isrRadio1, false, 0, 0, 0, 0, 0, 0 },
  { "R2", "J3 ANT2", &lora2, PIN_R2_CS, isrRadio2, false, 0, 0, 0, 0, 0, 0 },
};

// ================================================================
// Objects and state
// ================================================================
static Adafruit_NeoPixel leds(3, PIN_WS2812B, NEO_GRB + NEO_KHZ800);
static TinyGPSPlus gps;
static HardwareSerial gpsSerial(2);
#if ENABLE_WIFI_DASHBOARD
static WebServer web(80);
#endif

static Pending pending = {};

// Committed (unique) packets
static TelemetryPacket lastPkt = {};
static bool     havePacket   = false;
static uint32_t lastPacketMs = 0;
static uint8_t  lastMask     = 0;
static int      lastBestRssi = 0;
static float    lastBestSnr  = 0;
static int32_t  lastSeq      = -1;
static uint32_t uniqueCount  = 0;
static uint32_t lostCount    = 0;
static uint32_t onlyR1 = 0, onlyR2 = 0, bothRadios = 0;
static uint32_t fcReboots    = 0;
static float    maxAglM      = 0;
static bool     linkWasStale = true;

// Time (UTC epoch in ms, disciplined by GPS PPS when available)
static const char* TIME_SOURCE_NAMES[] = { "NONE", "PC", "GPS", "GPS+PPS" };
static TimeSource timeSource   = TS_NONE;
static int64_t    epochMsAtSync = 0;
static uint32_t   millisAtSync  = 0;
static uint32_t   lastGpsSyncMs = 0;
static uint32_t   lastTimeReqMs = 0;
static volatile uint32_t ppsMillis = 0;
static volatile uint32_t ppsCount  = 0;
static void IRAM_ATTR isrPps() { ppsMillis = millis(); ppsCount++; }

// SD
static File     gsLogFile;
static char     gsLogPath[32] = { 0 };
static bool     gsSdReady = false;
static uint32_t gsLastMountAttemptMs = 0;
static uint32_t sdWriteErrors = 0;

// UI
static uint32_t lastDashboardMs = 0;
static uint32_t lastLedMs = 0;
static uint8_t  beepsLeft = 0;
static uint16_t beepLenMs = 0;
static uint32_t nextBeepMs = 0;
static char     i2cDevices[48] = "none";
static uint32_t buttonShortCount = 0, buttonLongCount = 0;

// Serial command line buffer
static char     serialLineBuf[64];
static uint8_t  serialLineLen = 0;

static const char CSV_HEADER[] =
  "datetime,unix_ms,recv_millis,time_src,profile,rx_mask,rssi_r1,snr_r1,rssi_r2,snr_r2,"
  "seq,fc_timestamp_ms,state,"
  "pressure_pa,baro_alt_m,agl_m,baro_temp_c,baro_valid,"
  "sht_temp_c,humidity_pct,"
  "accel_x_g,accel_y_g,accel_z_g,accel_total_g,gyro_x_dps,gyro_y_dps,gyro_z_dps,mpu_temp_c,imu_valid,"
  "gps_lat,gps_lon,gps_alt_m,gps_sats,gps_fix,"
  "battery_v,fc_sd_logging,"
  "pyro1_cont,pyro2_cont,pyro1_fired,pyro2_fired,"
  "gs_lat,gs_lon,gs_alt_m,gs_sats,range_m,bearing_deg,elevation_deg";

static void runCommand(const char* cmd);

// ================================================================
// Buzzer — MLT-8530 is passive: PWM tone, never a DC level
// ================================================================
static void beepN(uint16_t ms, uint8_t times) {
  beepLenMs = ms;
  beepsLeft = times;
  nextBeepMs = millis();
}
static void beep(uint16_t ms) { beepN(ms, 1); }
static void serviceBuzzer() {
  if (beepsLeft == 0 || (int32_t)(millis() - nextBeepMs) < 0) return;
  tone(PIN_BUZZER, BUZZER_FREQ_HZ, beepLenMs);   // non-blocking on ESP32
  beepsLeft--;
  nextBeepMs = millis() + beepLenMs + 120;
}

// ================================================================
// Time
// ================================================================
static int64_t daysFromCivil(int y, int m, int d) {   // Howard Hinnant's algorithm, UTC
  y -= (m <= 2);
  const int era = (y >= 0 ? y : y - 399) / 400;
  const int yoe = y - era * 400;
  const int doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return (int64_t)era * 146097 + doe - 719468;
}

static bool timeValid() { return timeSource != TS_NONE; }

static int64_t epochMsAt(uint32_t ms) {
  return epochMsAtSync + (int32_t)(ms - millisAtSync);
}

static void setTime(int64_t epochMs, uint32_t atMillis, TimeSource src) {
  bool first = (timeSource == TS_NONE) || (src != timeSource);
  epochMsAtSync = epochMs;
  millisAtSync  = atMillis;
  timeSource    = src;
  if (first) {
    time_t t = (time_t)(epochMs / 1000);
    struct tm lt;
    localtime_r(&t, &lt);
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &lt);
    Serial.printf("[TIME] Synced from %s. Local time: %s (%s)\n", TIME_SOURCE_NAMES[src], buf, GS_TIMEZONE);
  }
}

// "YYYY-MM-DD HH:MM:SS.mmm" local time, or "UNSYNCED+hh:mm:ss"
static void formatDateTime(char* out, size_t size, uint32_t ms) {
  if (!timeValid()) {
    uint32_t s = ms / 1000;
    snprintf(out, size, "UNSYNCED+%02lu:%02lu:%02lu",
             (unsigned long)((s / 3600) % 100), (unsigned long)((s / 60) % 60), (unsigned long)(s % 60));
    return;
  }
  int64_t e = epochMsAt(ms);
  time_t t = (time_t)(e / 1000);
  struct tm lt;
  localtime_r(&t, &lt);
  char base[24];
  strftime(base, sizeof(base), "%Y-%m-%d %H:%M:%S", &lt);
  snprintf(out, size, "%s.%03d", base, (int)(e % 1000));
}

// GPS time (UTC). With a recent PPS edge, the NMEA second refers to that
// edge, so the sync point is the PPS timestamp (ms accuracy).
static void maybeSyncTimeFromGps() {
  if (!gps.time.isUpdated() || !gps.date.isValid() || !gps.time.isValid()) return;
  if (!gps.location.isValid() || gps.location.age() > 2000) return;   // only trust time with a fix
  if (gps.date.year() < 2025) return;
  uint32_t now = millis();
  if (timeSource >= TS_GPS && now - lastGpsSyncMs < GPS_RESYNC_MS) return;

  int64_t days = daysFromCivil(gps.date.year(), gps.date.month(), gps.date.day());
  int64_t epochS = days * 86400LL + gps.time.hour() * 3600LL + gps.time.minute() * 60LL + gps.time.second();

  noInterrupts();
  uint32_t ppsAt = ppsMillis;
  uint32_t ppsN  = ppsCount;
  interrupts();

  if (ppsN > 0 && now - ppsAt < 900) {
    setTime(epochS * 1000LL, ppsAt, TS_GPS_PPS);
  } else {
    setTime(epochS * 1000LL + gps.time.centisecond() * 10, now, TS_GPS);
  }
  lastGpsSyncMs = now;
}

static void maybeRequestTimeSync() {
  if (timeValid()) return;
  uint32_t now = millis();
  if (now - lastTimeReqMs < TIME_REQ_INTERVAL_MS) return;
  lastTimeReqMs = now;
  Serial.println("REQ_TIME");
}

// ================================================================
// GPS
// ================================================================
static void readGps() {
  while (gpsSerial.available()) gps.encode(gpsSerial.read());
}
static bool gsHasFix() {
  return gps.location.isValid() && gps.location.age() < 5000;
}
static bool fcHasFix() { return havePacket && lastPkt.gps_fix; }

static float rangeToRocketM() {
  if (!gsHasFix() || !fcHasFix()) return -1.0f;
  return (float)TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), lastPkt.gps_lat, lastPkt.gps_lon);
}
static float bearingToRocketDeg() {
  if (!gsHasFix() || !fcHasFix()) return -1.0f;
  return (float)TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), lastPkt.gps_lat, lastPkt.gps_lon);
}
// Elevation angle for aiming the Yagi: uses the FC GPS altitude when it has a fix
static float elevationToRocketDeg() {
  float r = rangeToRocketM();
  if (r < 0 || !gps.altitude.isValid()) return NAN;
  float dh = lastPkt.gps_alt_m - (float)gps.altitude.meters();
  return atan2f(dh, fmaxf(r, 1.0f)) * 57.2958f;
}

// ================================================================
// Radios
// ================================================================
static bool initRadio(RadioCh& r) {
  int16_t st = r.lora->begin(linkCfg.freqMhz, linkCfg.bwKhz, linkCfg.sf, linkCfg.crDenom, linkCfg.syncWord,
                             10, 8, 0);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("[LORA] %s (%s) init FAILED, code %d\n", r.name, r.antenna, st);
    return false;
  }
  r.lora->setCRC(true);
  r.lora->setPacketReceivedAction(r.isr);
  st = r.lora->startReceive();
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("[LORA] %s startReceive FAILED, code %d\n", r.name, st);
    return false;
  }
  Serial.printf("[LORA] %s (%s) listening\n", r.name, r.antenna);
  return true;
}

static void applyLinkProfile(const LinkProfile& p) {
  linkCfg = p;
  for (auto& r : radios) {
    if (!r.ok) continue;
    r.lora->standby();
    r.lora->setFrequency(linkCfg.freqMhz);
    r.lora->setBandwidth(linkCfg.bwKhz);
    r.lora->setSpreadingFactor(linkCfg.sf);
    r.lora->setCodingRate(linkCfg.crDenom);
    r.lora->setSyncWord(linkCfg.syncWord);
    r.lora->startReceive();
  }
  pending.active = false;
  lastSeq = -1;
  Serial.printf("[LORA] %s profile: %.1f MHz, SF%d, BW%.0f kHz, CR4/%d, sync 0x%02X\n",
                linkCfg.name, linkCfg.freqMhz, linkCfg.sf, linkCfg.bwKhz, linkCfg.crDenom, linkCfg.syncWord);
}

static void radiosStandby() {
  for (auto& r : radios) if (r.ok) r.lora->standby();
}
static void radiosResume() {
  for (int i = 0; i < 2; i++) {
    if (!radios[i].ok) continue;
    rxFlag[i] = false;
    radios[i].lora->startReceive();
  }
}

// ================================================================
// SD logging — onboard slot, or external module through J2
// ================================================================
#if SD_USE_EXT_MODULE
  #define SD_ACTIVE_CS   PIN_SD_CS_EXT
  #define SD_SRC_NAME    "external module (J2)"
#else
  #define SD_ACTIVE_CS   PIN_SD_CS
  #define SD_SRC_NAME    "onboard slot (CARD1)"
#endif

// The external module has its own MISO (IO39) because its level-shifter
// buffer never releases MISO. While it is accessed, the VSPI MISO input is
// re-routed to IO39 and both radios are put in standby (a few ms per write).
static void sdBusBegin() {
#if SD_USE_EXT_MODULE
  radiosStandby();
  pinMatrixInAttach(PIN_SD_MISO_EXT, VSPIQ_IN_IDX, false);
#endif
}
static void sdBusEnd() {
#if SD_USE_EXT_MODULE
  pinMatrixInAttach(PIN_SPI_MISO, VSPIQ_IN_IDX, false);
  radiosResume();
#endif
}

static bool openNewLogFile() {
  for (int i = 0; i < 1000; i++) {
    snprintf(gsLogPath, sizeof(gsLogPath), "/MAGNIS%03d.CSV", i);
    if (!SD.exists(gsLogPath)) break;
  }
  gsLogFile = SD.open(gsLogPath, FILE_WRITE);
  if (!gsLogFile) {
    Serial.printf("[SD] Failed to create %s\n", gsLogPath);
    return false;
  }
  gsLogFile.println(CSV_HEADER);
  gsLogFile.flush();
  Serial.printf("[SD] Logging to %s on " SD_SRC_NAME "\n", gsLogPath);
  return true;
}

static bool gsAttemptSDMount() {
  sdBusBegin();
  bool ok = SD.begin(SD_ACTIVE_CS, SPI, SD_SPI_FREQ_HZ);
  if (!ok) {
    Serial.println("[SD] SD.begin() failed on " SD_SRC_NAME " — continuing without logging.");
  } else if (SD.cardType() == CARD_NONE) {
    Serial.println("[SD] No card — continuing without logging.");
    SD.end();
    ok = false;
  } else {
    ok = openNewLogFile();
  }
  sdBusEnd();
  return ok;
}

static void gsMaintainSDConnection() {
  if (gsSdReady) return;
  uint32_t now = millis();
  if (now - gsLastMountAttemptMs < SD_MOUNT_RETRY_MS) return;
  gsLastMountAttemptMs = now;
  gsSdReady = gsAttemptSDMount();
}

static void gsWriteLine(const char* line) {
  if (!gsSdReady || !gsLogFile) return;
  sdBusBegin();
  size_t written = gsLogFile.println(line);
  gsLogFile.flush();   // flushed every row: ~1.5 Hz max, and a power loss keeps everything
  if (written == 0) {
    sdWriteErrors++;
    Serial.println("[SD] Write failed — will retry mount.");
    gsLogFile.close();
    SD.end();
    gsSdReady = false;
  }
  sdBusEnd();
}

// ================================================================
// Diversity: merge copies of the same packet from both radios
// ================================================================
static void buildCsvRow(char* out, size_t size, const Pending& p, uint32_t recvMs) {
  char dt[32];
  formatDateTime(dt, sizeof(dt), recvMs);
  char unixMs[24] = "";
  if (timeValid()) snprintf(unixMs, sizeof(unixMs), "%lld", (long long)epochMsAt(recvMs));
  char r1[24] = ",", r2[24] = ",";
  if (p.mask & 1) snprintf(r1, sizeof(r1), "%d,%.1f", p.rssi[0], p.snr[0]);
  if (p.mask & 2) snprintf(r2, sizeof(r2), "%d,%.1f", p.rssi[1], p.snr[1]);
  char gs[96] = ",,,,,,";
  if (gsHasFix()) {
    float range = rangeToRocketM();
    float elev  = elevationToRocketDeg();
    char rb[48] = ",,";
    if (range >= 0) snprintf(rb, sizeof(rb), "%.1f,%.1f,%s", range, bearingToRocketDeg(),
                             isnan(elev) ? "" : String(elev, 1).c_str());
    snprintf(gs, sizeof(gs), "%.6f,%.6f,%.1f,%u,%s", gps.location.lat(), gps.location.lng(),
             gps.altitude.meters(), (unsigned)gps.satellites.value(), rb);
  }
  const TelemetryPacket& k = p.pkt;
  snprintf(out, size,
    "%s,%s,%lu,%s,%s,%u,%s,%s,"
    "%lu,%lu,%u,"
    "%.1f,%.2f,%.2f,%.2f,%u,"
    "%.2f,%.2f,"
    "%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%u,"
    "%.6f,%.6f,%.2f,%u,%u,"
    "%.2f,%u,"
    "%u,%u,%u,%u,"
    "%s",
    dt, unixMs, (unsigned long)recvMs, TIME_SOURCE_NAMES[timeSource], linkCfg.name, p.mask, r1, r2,
    (unsigned long)k.seq, (unsigned long)k.timestamp_ms, k.state,
    k.pressure_pa, k.baro_alt_m, k.agl_m, k.baro_temp_c, k.baro_valid,
    k.sht_temp_c, k.humidity_pct,
    k.accel_x_g, k.accel_y_g, k.accel_z_g, k.accel_total_g, k.gyro_x_dps, k.gyro_y_dps, k.gyro_z_dps,
    k.mpu_temp_c, k.imu_valid,
    k.gps_lat, k.gps_lon, k.gps_alt_m, k.gps_sats, k.gps_fix,
    k.battery_v, k.sd_logging,
    k.pyro1_continuity, k.pyro2_continuity, k.pyro1_fired, k.pyro2_fired,
    gs);
}

static void commitPending() {
  if (!pending.active) return;
  pending.active = false;
  const uint32_t seq = pending.seq;

  if (lastSeq >= 0) {
    if ((int32_t)seq == lastSeq) return;                    // late duplicate, already logged
    if ((int32_t)seq > lastSeq) {
      lostCount += (uint32_t)((int32_t)seq - lastSeq - 1);
    } else if (lastSeq - (int32_t)seq > 10) {
      fcReboots++;                                          // seq restarted: FC reboot, not loss
      Serial.println("[LINK] Sequence restarted — FC reboot?");
    } else {
      return;                                               // small backward jump: stale copy
    }
  }
  lastSeq = (int32_t)seq;

  if (pending.mask == 3) bothRadios++;
  else if (pending.mask == 1) onlyR1++;
  else onlyR2++;

  int best = (pending.mask == 2 || ((pending.mask == 3) && pending.rssi[1] > pending.rssi[0])) ? 1 : 0;
  lastBestRssi = pending.rssi[best];
  lastBestSnr  = pending.snr[best];
  lastMask     = pending.mask;
  lastPkt      = pending.pkt;
  lastPacketMs = pending.firstMs;
  havePacket   = true;
  uniqueCount++;
  if (lastPkt.agl_m > maxAglM) maxAglM = lastPkt.agl_m;

  if (linkWasStale) beep(60);   // first packet or link recovered
  linkWasStale = false;

  char row[640];
  buildCsvRow(row, sizeof(row), pending, pending.firstMs);
  gsWriteLine(row);
#if SERIAL_TLM_LINES
  Serial.print("TLM,");
  Serial.println(row);
#endif
}

static void onPacket(int idx, const TelemetryPacket& pkt, int rssi, float snr) {
  if (pending.active && pending.seq == pkt.seq) {
    pending.mask |= (1 << idx);
    pending.rssi[idx] = rssi;
    pending.snr[idx]  = snr;
    if (pending.mask == 3) commitPending();   // both copies in: no need to wait
    return;
  }
  commitPending();
  pending.active   = true;
  pending.seq      = pkt.seq;
  pending.pkt      = pkt;
  pending.mask     = (1 << idx);
  pending.rssi[idx] = rssi;
  pending.snr[idx]  = snr;
  pending.firstMs  = millis();
}

static void serviceRadio(int idx) {
  RadioCh& r = radios[idx];
  if (!r.ok || !rxFlag[idx]) return;
  rxFlag[idx] = false;

  uint8_t buf[256];
  size_t len = r.lora->getPacketLength();
  if (len > sizeof(buf)) len = sizeof(buf);
  int16_t st = r.lora->readData(buf, len);
  int   rssi = (int)lroundf(r.lora->getRSSI());
  float snr  = r.lora->getSNR();
  r.lora->startReceive();

  if (st == RADIOLIB_ERR_CRC_MISMATCH) { r.crcErrors++; return; }
  if (st != RADIOLIB_ERR_NONE) return;

  size_t offset;
  if (len == sizeof(TelemetryPacket) + RH_HEADER_LEN) offset = RH_HEADER_LEN;   // RadioHead TX
  else if (len == sizeof(TelemetryPacket))            offset = 0;               // raw payload
  else {
    r.sizeErrors++;
    Serial.printf("[LORA] %s size mismatch: %u bytes (expected %u or %u), RSSI %d dBm\n", r.name,
                  (unsigned)len, (unsigned)sizeof(TelemetryPacket),
                  (unsigned)(sizeof(TelemetryPacket) + RH_HEADER_LEN), rssi);
    return;
  }
  TelemetryPacket pkt;
  memcpy(&pkt, buf + offset, sizeof(pkt));
  r.rxCount++;
  r.lastRssi = rssi;
  r.lastSnr  = snr;
  r.lastRxMs = millis();
  onPacket(idx, pkt, rssi, snr);
}

// Share of packets each radio caught, out of all packets the FC sent
static float radioPct(int idx) {
  uint32_t expected = uniqueCount + lostCount;
  return expected ? 100.0f * radios[idx].rxCount / expected : 0.0f;
}
static float combinedPct() {
  uint32_t expected = uniqueCount + lostCount;
  return expected ? 100.0f * uniqueCount / expected : 0.0f;
}

// ================================================================
// Button — input only, multi-purpose. Actions are hooks, to be assigned.
// ================================================================
static void onButtonShort() {
  // TODO: assign (e.g. new log file, switch profile, mute buzzer)
  beep(40);
}
static void onButtonLong() {
  // TODO: assign
  beepN(40, 2);
}

static void serviceButton() {
  static bool     stable = true;      // HIGH = released (pull-up)
  static bool     lastRaw = true;
  static uint32_t changeMs = 0, pressMs = 0;
  static bool     longFired = false;
  bool raw = digitalRead(PIN_BUTTON);
  uint32_t now = millis();
  if (raw != lastRaw) { lastRaw = raw; changeMs = now; }
  if (now - changeMs < 30 || raw == stable) {
    if (!stable && !longFired && now - pressMs >= BUTTON_LONG_MS) {
      longFired = true;
      buttonLongCount++;
      Serial.println("[BTN] long press");
      onButtonLong();
    }
    return;
  }
  stable = raw;
  if (!stable) {                       // pressed
    pressMs = now;
    longFired = false;
  } else if (!longFired) {             // released before the long-press time
    buttonShortCount++;
    Serial.println("[BTN] short press");
    onButtonShort();
  }
}

// ================================================================
// Status LEDs (LED1 = index 0, at R2)
//   LED1 SD    green logging · red off
//   LED2 LINK  blue flash on packet · green both radios · yellow one radio
//              · red stale (> 5 s) · amber waiting
//   LED3 GPS   green fix · amber searching · red no NMEA data
// ================================================================
static void updateLeds() {
  uint32_t now = millis();
  if (now - lastLedMs < 100) return;
  lastLedMs = now;

  const uint32_t RED = leds.Color(255, 0, 0), GREEN = leds.Color(0, 255, 0), BLUE = leds.Color(0, 0, 255);
  const uint32_t AMBER = leds.Color(255, 120, 0), YELLOW = leds.Color(200, 200, 0);

  leds.setPixelColor(0, gsSdReady ? GREEN : RED);

  uint32_t link;
  if (!havePacket)                             link = AMBER;
  else if (now - lastPacketMs < 100)           link = BLUE;
  else if (now - lastPacketMs < LINK_STALE_MS) link = (lastMask == 3) ? GREEN : YELLOW;
  else                                         link = RED;
  leds.setPixelColor(1, link);

  uint32_t g;
  if (gps.charsProcessed() < 10 || gps.time.age() > 3000) g = RED;
  else g = gsHasFix() ? GREEN : AMBER;
  leds.setPixelColor(2, g);
  leds.show();
}

// ================================================================
// I2C expansion header (J4) — scan only, for a future OLED or sensor
// ================================================================
static void scanI2C() {
#if ENABLE_I2C_SCAN
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setTimeOut(20);
  size_t n = 0;
  i2cDevices[0] = '\0';
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0 && n < sizeof(i2cDevices) - 6) {
      n += snprintf(i2cDevices + n, sizeof(i2cDevices) - n, "%s0x%02X", n ? " " : "", a);
    }
  }
  if (n == 0) strcpy(i2cDevices, "none");
  Serial.printf("[I2C] J4 devices: %s\n", i2cDevices);
#endif
}

// ================================================================
// Wi-Fi dashboard (access point, no internet needed)
// ================================================================
#if ENABLE_WIFI_DASHBOARD
static const char INDEX_HTML[] = R"HTML(<!doctype html><html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>MAGNIS GS</title>
<style>
body{margin:0;font-family:system-ui,sans-serif;background:#0f1412;color:#e8ece9}
header{padding:12px 16px;background:#18201c;display:flex;justify-content:space-between;align-items:center}
h1{font-size:18px;margin:0}#age{font-size:13px;color:#9fb0a6}
main{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:10px;padding:12px}
.c{background:#18201c;border-radius:10px;padding:10px 12px}.l{font-size:12px;color:#9fb0a6}
.v{font-size:22px;font-weight:600;margin-top:2px}.s{font-size:12px;color:#c6d2cb;margin-top:4px}
.ok{color:#5be08a}.warn{color:#f5c542}.bad{color:#ff6b6b}
footer{padding:0 12px 16px;display:flex;gap:8px;flex-wrap:wrap}
button{background:#26332c;color:#e8ece9;border:0;border-radius:8px;padding:10px 14px;font-size:14px}
</style></head><body>
<header><h1>MAGNIS v1 · <span id="prof">-</span></h1><span id="age">sin datos</span></header>
<main>
<div class="c"><div class="l">Estado de vuelo</div><div class="v" id="state">-</div><div class="s" id="seq"></div></div>
<div class="c"><div class="l">Altura AGL</div><div class="v" id="agl">-</div><div class="s" id="maxagl"></div></div>
<div class="c"><div class="l">Enlace combinado</div><div class="v" id="comb">-</div><div class="s" id="lost"></div></div>
<div class="c"><div class="l">R1 · J1</div><div class="v" id="r1">-</div><div class="s" id="r1s"></div></div>
<div class="c"><div class="l">R2 · J3</div><div class="v" id="r2">-</div><div class="s" id="r2s"></div></div>
<div class="c"><div class="l">Apuntar Yagi</div><div class="v" id="aim">-</div><div class="s" id="range"></div></div>
<div class="c"><div class="l">GPS cohete</div><div class="v" id="fcgps">-</div><div class="s" id="fcpos"></div></div>
<div class="c"><div class="l">GPS estación</div><div class="v" id="gsgps">-</div><div class="s" id="gspos"></div></div>
<div class="c"><div class="l">Batería FC</div><div class="v" id="bat">-</div><div class="s" id="pyro"></div></div>
<div class="c"><div class="l">SD · Hora</div><div class="v" id="sd">-</div><div class="s" id="time"></div></div>
</main>
<footer><button onclick="cmd('PROFILE:ROCKET')">Perfil cohete</button>
<button onclick="cmd('PROFILE:CANSAT')">Perfil CanSat</button><button onclick="cmd('NEWLOG')">Nuevo log</button></footer>
<script>
const $=id=>document.getElementById(id);
function cls(e,c){e.className='v '+c}
function cmd(c){fetch('/api/cmd?c='+encodeURIComponent(c))}
async function tick(){try{const d=await (await fetch('/api/state')).json();
$('prof').textContent=d.profile;
if(d.have){$('age').textContent='último paquete hace '+(d.age_ms/1000).toFixed(1)+' s';
$('age').className=d.age_ms>5000?'bad':'ok';
$('state').textContent=d.state;$('seq').textContent='seq #'+d.seq;
$('agl').textContent=d.agl.toFixed(1)+' m';$('maxagl').textContent='máx '+d.max_agl.toFixed(1)+' m · baro '+d.baro_alt.toFixed(1)+' m';
$('fcgps').textContent=d.fc_fix?'FIX':'sin fix';$('fcpos').textContent=d.fc_lat.toFixed(6)+', '+d.fc_lon.toFixed(6)+' · '+d.fc_sats+' sats';
$('bat').textContent=d.bat.toFixed(2)+' V';$('pyro').textContent='CH1 '+(d.p1c?'OK':'OPEN')+(d.p1f?' FIRED':'')+' · CH2 '+(d.p2c?'OK':'OPEN')+(d.p2f?' FIRED':'');}
$('comb').textContent=d.comb_pct.toFixed(1)+' %';cls($('comb'),d.comb_pct>95?'ok':d.comb_pct>80?'warn':'bad');
$('lost').textContent=d.unique+' recibidos · '+d.lost+' perdidos';
$('r1').textContent=d.r1_ok?d.r1_rssi+' dBm':'FALLA';$('r1s').textContent=d.r1_pct.toFixed(1)+' % · SNR '+d.r1_snr.toFixed(1)+' · solo R1: '+d.only_r1;
$('r2').textContent=d.r2_ok?d.r2_rssi+' dBm':'FALLA';$('r2s').textContent=d.r2_pct.toFixed(1)+' % · SNR '+d.r2_snr.toFixed(1)+' · solo R2: '+d.only_r2;
$('aim').textContent=d.range>=0?('az '+d.bearing.toFixed(0)+'° · el '+(d.elev===null?'-':d.elev.toFixed(0)+'°')):'-';
$('range').textContent=d.range>=0?('distancia '+d.range.toFixed(0)+' m'):'necesita fix en ambos GPS';
$('gsgps').textContent=d.gs_fix?'FIX':'sin fix';$('gspos').textContent=d.gs_fix?(d.gs_lat.toFixed(6)+', '+d.gs_lon.toFixed(6)+' · '+d.gs_sats+' sats'):'';
$('sd').textContent=d.sd;$('time').textContent=d.time+' ('+d.time_src+')';
}catch(e){$('age').textContent='sin conexión';$('age').className='bad'}}
setInterval(tick,500);tick();
</script></body></html>)HTML";

static void handleRoot() { web.send(200, "text/html", INDEX_HTML); }

static void handleState() {
  static char json[1600];
  uint32_t now = millis();
  char dt[32];
  formatDateTime(dt, sizeof(dt), now);
  float range = rangeToRocketM();
  float elev  = elevationToRocketDeg();
  char elevStr[16];
  if (isnan(elev)) strcpy(elevStr, "null"); else snprintf(elevStr, sizeof(elevStr), "%.1f", elev);
  snprintf(json, sizeof(json),
    "{\"profile\":\"%s\",\"have\":%s,\"age_ms\":%lu,\"state\":\"%s\",\"seq\":%lu,"
    "\"agl\":%.2f,\"max_agl\":%.2f,\"baro_alt\":%.2f,"
    "\"fc_fix\":%u,\"fc_lat\":%.6f,\"fc_lon\":%.6f,\"fc_sats\":%u,\"bat\":%.2f,"
    "\"p1c\":%u,\"p1f\":%u,\"p2c\":%u,\"p2f\":%u,"
    "\"unique\":%lu,\"lost\":%lu,\"comb_pct\":%.2f,"
    "\"r1_ok\":%s,\"r1_rssi\":%d,\"r1_snr\":%.1f,\"r1_pct\":%.2f,\"only_r1\":%lu,"
    "\"r2_ok\":%s,\"r2_rssi\":%d,\"r2_snr\":%.1f,\"r2_pct\":%.2f,\"only_r2\":%lu,"
    "\"range\":%.1f,\"bearing\":%.1f,\"elev\":%s,"
    "\"gs_fix\":%s,\"gs_lat\":%.6f,\"gs_lon\":%.6f,\"gs_sats\":%u,"
    "\"sd\":\"%s\",\"time\":\"%s\",\"time_src\":\"%s\",\"i2c\":\"%s\"}",
    linkCfg.name, havePacket ? "true" : "false", (unsigned long)(havePacket ? now - lastPacketMs : 0),
    flightStateName(lastPkt.state), (unsigned long)lastPkt.seq,
    lastPkt.agl_m, maxAglM, lastPkt.baro_alt_m,
    lastPkt.gps_fix, lastPkt.gps_lat, lastPkt.gps_lon, lastPkt.gps_sats, lastPkt.battery_v,
    lastPkt.pyro1_continuity, lastPkt.pyro1_fired, lastPkt.pyro2_continuity, lastPkt.pyro2_fired,
    (unsigned long)uniqueCount, (unsigned long)lostCount, combinedPct(),
    radios[0].ok ? "true" : "false", radios[0].lastRssi, radios[0].lastSnr, radioPct(0), (unsigned long)onlyR1,
    radios[1].ok ? "true" : "false", radios[1].lastRssi, radios[1].lastSnr, radioPct(1), (unsigned long)onlyR2,
    range, range >= 0 ? bearingToRocketDeg() : -1.0f, elevStr,
    gsHasFix() ? "true" : "false", gps.location.lat(), gps.location.lng(), (unsigned)gps.satellites.value(),
    gsSdReady ? gsLogPath : "OFF", dt, TIME_SOURCE_NAMES[timeSource], i2cDevices);
  web.send(200, "application/json", json);
}

static void handleCmd() {
  if (web.hasArg("c")) runCommand(web.arg("c").c_str());
  web.send(200, "text/plain", "ok");
}

static void startWifiDashboard() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
  WiFi.setSleep(false);   // no modem sleep: avoids the GPIO36/39 glitch and keeps latency low
  web.on("/", handleRoot);
  web.on("/api/state", handleState);
  web.on("/api/cmd", handleCmd);
  web.begin();
  Serial.printf("[WIFI] AP \"%s\" up — open http://%s\n", WIFI_AP_SSID, WiFi.softAPIP().toString().c_str());
}
#endif

// ================================================================
// Commands (USB serial and web):
//   SETTIME:<unix_epoch_utc>   PROFILE:ROCKET   PROFILE:CANSAT   NEWLOG   BEEP
// ================================================================
static void runCommand(const char* cmd) {
  if (strncmp(cmd, "SETTIME:", 8) == 0) {
    long long epoch = atoll(cmd + 8);
    if (epoch <= 0) { Serial.println("[TIME] Ignored malformed SETTIME line."); return; }
    if (timeSource >= TS_GPS) { Serial.println("[TIME] GPS time active — SETTIME ignored."); return; }
    setTime(epoch * 1000LL, millis(), TS_PC);
  } else if (strcmp(cmd, "PROFILE:ROCKET") == 0) {
    applyLinkProfile(PROFILE_ROCKET);
  } else if (strcmp(cmd, "PROFILE:CANSAT") == 0) {
    applyLinkProfile(PROFILE_CANSAT);
  } else if (strcmp(cmd, "NEWLOG") == 0) {
    if (gsSdReady) {
      sdBusBegin();
      gsLogFile.close();
      gsSdReady = openNewLogFile();
      sdBusEnd();
    }
  } else if (strcmp(cmd, "BEEP") == 0) {
    beep(100);
  }
}

static void handleSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialLineLen > 0) {
        serialLineBuf[serialLineLen] = '\0';
        runCommand(serialLineBuf);
        serialLineLen = 0;
      }
    } else if (serialLineLen < sizeof(serialLineBuf) - 1) {
      serialLineBuf[serialLineLen++] = c;
    }
  }
}

// ================================================================
// Serial dashboard
// ================================================================
static void printDashboard() {
#if SERIAL_DASHBOARD
  uint32_t now = millis();
  char dt[32];
  formatDateTime(dt, sizeof(dt), now);
  Serial.println();
  Serial.println(F("============================================================"));
  Serial.printf(" MAGNIS v1 [%s] | %s (%s) | SD: %s\n", linkCfg.name, dt, TIME_SOURCE_NAMES[timeSource],
                gsSdReady ? gsLogPath : "OFF");
  for (int i = 0; i < 2; i++) {
    const RadioCh& r = radios[i];
    if (!r.ok) { Serial.printf(" %s %-8s FAILED AT BOOT\n", r.name, r.antenna); continue; }
    Serial.printf(" %s %-8s RSSI %5d dBm  SNR %5.1f dB  rx %-6lu (%5.1f%%)  crc %lu  size %lu\n", r.name,
                  r.antenna, r.lastRssi, r.lastSnr, (unsigned long)r.rxCount, radioPct(i),
                  (unsigned long)r.crcErrors, (unsigned long)r.sizeErrors);
  }
  Serial.printf(" COMBINED unique %lu  lost %lu  (%.1f%%)  both %lu  only R1 %lu  only R2 %lu\n",
                (unsigned long)uniqueCount, (unsigned long)lostCount, combinedPct(),
                (unsigned long)bothRadios, (unsigned long)onlyR1, (unsigned long)onlyR2);
  Serial.println(F("------------------------------------------------------------"));
  if (!havePacket) {
    Serial.printf(" Waiting for first packet...   GS GPS: %s (%u sats)\n", gsHasFix() ? "FIX" : "no fix",
                  (unsigned)gps.satellites.value());
    Serial.println(F("============================================================"));
    return;
  }
  uint32_t since = now - lastPacketMs;
  Serial.printf(" STATE %-15s seq #%-6lu last RX %5lu ms ago%s\n", flightStateName(lastPkt.state),
                (unsigned long)lastPkt.seq, (unsigned long)since, since > LINK_STALE_MS ? "  [STALE]" : "");
  Serial.printf(" BARO  AGL %8.2f m (max %.1f)  ABS %8.2f m  T %6.2f C [%s]\n", lastPkt.agl_m, maxAglM,
                lastPkt.baro_alt_m, lastPkt.baro_temp_c, lastPkt.baro_valid ? "OK" : "STALE");
  Serial.printf(" IMU   |a| %6.2f g  gyro %7.1f %7.1f %7.1f dps [%s]\n", lastPkt.accel_total_g,
                lastPkt.gyro_x_dps, lastPkt.gyro_y_dps, lastPkt.gyro_z_dps, lastPkt.imu_valid ? "OK" : "STALE");
  Serial.printf(" FC GPS %s sats %2u  %10.6f, %11.6f  alt %.1f m\n", lastPkt.gps_fix ? "FIX   " : "NO FIX",
                lastPkt.gps_sats, lastPkt.gps_lat, lastPkt.gps_lon, lastPkt.gps_alt_m);
  float range = rangeToRocketM();
  if (range >= 0) {
    float elev = elevationToRocketDeg();
    Serial.printf(" AIM   range %.0f m  azimuth %.0f deg  elevation %s deg\n", range, bearingToRocketDeg(),
                  isnan(elev) ? "-" : String(elev, 0).c_str());
  } else {
    Serial.printf(" AIM   needs a fix on both GPS (GS: %s, %u sats)\n", gsHasFix() ? "FIX" : "no fix",
                  (unsigned)gps.satellites.value());
  }
  Serial.printf(" PWR   FC battery %5.2f V  | FC SD %s\n", lastPkt.battery_v, lastPkt.sd_logging ? "LOGGING" : "OFF");
  Serial.printf(" PYRO  CH1 cont %-4s fired %-3s | CH2 cont %-4s fired %-3s\n",
                lastPkt.pyro1_continuity ? "OK" : "OPEN", lastPkt.pyro1_fired ? "YES" : "no",
                lastPkt.pyro2_continuity ? "OK" : "OPEN", lastPkt.pyro2_fired ? "YES" : "no");
  Serial.println(F("============================================================"));
#endif
}

// ================================================================
// Setup
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("=== MAGNIS v1 Ground Station (preliminary, untested) ===");

  // Every SPI chip select high before anything touches the bus
  const uint8_t csPins[] = { PIN_R1_CS, PIN_R2_CS, PIN_SD_CS, PIN_SD_CS_EXT };
  for (uint8_t p : csPins) { pinMode(p, OUTPUT); digitalWrite(p, HIGH); }

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_BUTTON, INPUT);        // input-only pin, external pull-up R9
  pinMode(PIN_GPS_PPS, INPUT);
  pinMode(PIN_SD_MISO_EXT, INPUT);

  leds.begin();
  leds.setBrightness(LED_BRIGHTNESS);
  leds.clear();
  leds.show();

  setenv("TZ", GS_TIMEZONE, 1);
  tzset();

  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

  int okCount = 0;
  for (auto& r : radios) {
    r.ok = initRadio(r);
    okCount += r.ok;
  }
  if (okCount == 0) {
    Serial.println("[LORA] No radio available. Check both RFM95W.");
    leds.fill(leds.Color(255, 0, 0));
    leds.show();
    tone(PIN_BUZZER, BUZZER_FREQ_HZ, 1000);
    while (true) delay(1000);
  }
  if (okCount == 1) {
    Serial.println("[LORA] Running with ONE radio (no diversity).");
    beep(400);
  }
  applyLinkProfile(linkCfg);
  Serial.println("[LORA] Send PROFILE:ROCKET or PROFILE:CANSAT to switch.");

  gsSdReady = gsAttemptSDMount();
  gsLastMountAttemptMs = millis();

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  gpsSerial.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");   // RMC + GGA only
  attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS), isrPps, RISING);
  Serial.println("[GPS] UART2 started, waiting for fix...");

  scanI2C();

#if ENABLE_WIFI_DASHBOARD
  startWifiDashboard();
#endif

  Serial.println("[TIME] Waiting for GPS time or PC sync (send \"SETTIME:<unix_epoch>\\n\")...");
  lastTimeReqMs = millis();
  beep(150);
  Serial.println("=== Ground station ready ===");
}

// ================================================================
// Loop
// ================================================================
void loop() {
  handleSerialCommands();
#if ENABLE_WIFI_DASHBOARD
  web.handleClient();
#endif
  readGps();
  maybeSyncTimeFromGps();
  maybeRequestTimeSync();

  serviceRadio(0);
  serviceRadio(1);
  if (pending.active && millis() - pending.firstMs >= MERGE_WINDOW_MS) commitPending();

  if (havePacket && !linkWasStale && millis() - lastPacketMs > LINK_STALE_MS) {
    linkWasStale = true;
    beep(300);   // link lost
  }

  serviceButton();
  gsMaintainSDConnection();
  updateLeds();
  serviceBuzzer();

  uint32_t now = millis();
  if (now - lastDashboardMs >= DASHBOARD_INTERVAL_MS) {
    lastDashboardMs = now;
    printDashboard();
  }
}

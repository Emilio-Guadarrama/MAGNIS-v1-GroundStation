/*
  ================================================================
  VESTA v1 — Ground Station Receiver + Live Dashboard
  Ignitia Rocket Lab · Tecnológico de Monterrey GDL

  PRELIMINARY VERSION — NOT TESTED.
  This sketch was adapted from the previous ground station code
  (IGNIS_GroundStation_Heavy_3.ino), which ran on an IGNIS v2 board.
  It has NOT been compiled against or run on VESTA hardware yet,
  because the VESTA v1 board is not finished. Expect pin, library
  and timing fixes during bring-up.

  What changed from IGNIS_GroundStation_Heavy_3:
    - MCU: ESP32-DevKitC V4 (ESP32-WROOM-32D) instead of ESP32-S3.
      All pins re-mapped to the VESTA v1 netlist.
    - LoRa and microSD on SEPARATE SPI buses (VSPI / HSPI), so the
      SD card can no longer break the radio.
    - MODE_SEL jumper (IO36) picks the link profile at boot:
      open = rocket (sync 0x12), jumper fitted = CanSat (sync 0xAB).
    - SD source selectable at compile time: onboard slot or the
      external microSD module header (each has its own MISO and CS).
      No card-detect pin on VESTA.
    - Onboard GPS (Adafruit Ultimate GPS on UART2): GS position on
      the dashboard, distance to the rocket, and time sync from GPS
      (the PC SETTIME:<epoch> sync is kept as a fallback/override).
    - 3x WS2812B status LEDs and a buzzer.
  Kept unchanged: TelemetryPacket layout, CSV format (plus GS GPS
  columns), PC time-sync protocol, dashboard, RSSI/SNR register reads.

  Arduino IDE (Tools menu): Board "ESP32 Dev Module", Flash Size 4MB,
  PSRAM Disabled. Upload over the DevKit micro-USB (CP2102).
  Libraries: RadioHead, Adafruit NeoPixel, TinyGPSPlus.

  Radio config and TelemetryPacket MUST MATCH the flight computer
  firmware exactly. The struct below is copied from the IGNIS v2 GS —
  re-check it against the FC firmware you will receive from.
  ================================================================
*/

#include <SPI.h>
#include <SD.h>
#include <RH_RF95.h>
#include <Adafruit_NeoPixel.h>
#include <TinyGPSPlus.h>
#include <time.h>
#include <math.h>

// ================================================================
// VESTA v1 pin map (see VESTA_v1_Netlist.xlsx)
// ================================================================
// LoRa — VSPI
#define PIN_RFM95_SCK      18
#define PIN_RFM95_MISO     19
#define PIN_RFM95_MOSI     23
#define PIN_RFM95_CS        5
#define PIN_RFM95_RST      27
#define PIN_RFM95_DIO0     26
#define PIN_RFM95_DIO1     33   // not used yet

// microSD — HSPI (dedicated bus)
#define PIN_SD_SCK         14
#define PIN_SD_MOSI        13
#define PIN_SD_MISO_SLOT   34   // input-only pin, external pull-up on board
#define PIN_SD_CS_SLOT     15
#define PIN_SD_MISO_EXT    35   // input-only pin
#define PIN_SD_CS_EXT       4

// GPS — UART2
#define PIN_GPS_RX         16   // ESP32 RX  <- GPS TX (net GPS_TX)
#define PIN_GPS_TX         17   // ESP32 TX  -> GPS RX (net GPS_RX)
#define PIN_GPS_PPS        32   // not used yet

// Indicators / config
#define PIN_WS2812B        25
#define PIN_BUZZER         22   // active buzzer through Q1 (HIGH = on)
#define PIN_MODE_SEL       36   // input-only, external pull-up. LOW = CanSat

// ================================================================
// Options
// ================================================================
#define SD_USE_EXT_MODULE   0          // 0 = onboard slot (J2), 1 = external module (J3)
#define SD_SPI_FREQ_HZ      4000000
#define SD_MOUNT_RETRY_MS   10000

#define GPS_BAUD            9600
#define GS_TIMEZONE         "CST6"     // Mexico Central, UTC-6, no DST
#define TIME_REQ_INTERVAL_MS 3000
#define DASHBOARD_INTERVAL_MS 500
#define LINK_STALE_MS       5000
#define LED_BRIGHTNESS      40

// ================================================================
// Link profiles — MUST MATCH the transmitter firmware.
// TODO: for LASC (Brazil) move both channels inside 915–928 MHz;
// 915.0 MHz with BW125 spills below 915 MHz.
// ================================================================
struct LinkProfile {
  const char* name;
  float   freqMhz;
  uint8_t sf;
  long    bwHz;
  uint8_t crDenom;
  uint8_t syncWord;
};
static const LinkProfile PROFILE_ROCKET = { "ROCKET", 915.0, 9, 125000, 5, 0x12 };
static const LinkProfile PROFILE_CANSAT = { "CANSAT", 915.5, 9, 125000, 5, 0xAB };
static LinkProfile linkCfg;   // "link" would clash with POSIX link()

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

static const char* FLIGHT_STATE_NAMES[] = {
  "PAD_IDLE", "ARMED", "BOOST", "COAST", "APOGEE",
  "DROGUE_DESCENT", "MAIN_DESCENT", "LANDED"
};
static const char* flightStateName(uint8_t s) {
  return (s < 8) ? FLIGHT_STATE_NAMES[s] : "UNKNOWN";
}

// ================================================================
// Objects and state
// ================================================================
static RH_RF95 rf95(PIN_RFM95_CS, PIN_RFM95_DIO0);   // uses the global SPI (VSPI)
static SPIClass sdSPI(HSPI);
static Adafruit_NeoPixel leds(3, PIN_WS2812B, NEO_GRB + NEO_KHZ800);
static TinyGPSPlus gps;
static HardwareSerial gpsSerial(2);

// Link
static TelemetryPacket lastPkt = {};
static bool          havePacket      = false;
static unsigned long lastPacketMs    = 0;
static uint32_t      packetsReceived = 0;
static uint32_t      packetsLost     = 0;
static int32_t       lastSeq         = -1;
static int           lastRssiDbm     = 0;
static float         lastSnrDb       = 0;
static bool          linkWasStale    = true;

// Time
static time_t        baseRealTime  = 0;    // 0 = not synced
static unsigned long syncMillis    = 0;
static const char*   timeSource    = "NONE";
static bool          gpsTimeSynced = false;
static unsigned long lastTimeReqMs = 0;
static char          serialLineBuf[64];
static uint8_t       serialLineLen = 0;

// SD
static File          gsLogFile;
static char          gsLogPath[32] = {0};
static bool          gsSdReady = false;
static unsigned long gsLastMountAttemptMs = 0;

// Timers
static unsigned long lastDashboardMs = 0;
static unsigned long lastLedMs       = 0;
static unsigned long buzzerOffAt     = 0;

const char GS_CSV_HEADER[] =
  "datetime,recv_millis,rssi_dbm,snr_db,seq,fc_timestamp_ms,state,"
  "pressure_pa,baro_alt_m,agl_m,baro_temp_c,baro_valid,"
  "sht_temp_c,humidity_pct,"
  "accel_x_g,accel_y_g,accel_z_g,accel_total_g,gyro_x_dps,gyro_y_dps,gyro_z_dps,mpu_temp_c,imu_valid,"
  "gps_lat,gps_lon,gps_alt_m,gps_sats,gps_fix,"
  "battery_v,fc_sd_logging,"
  "pyro1_cont,pyro2_cont,pyro1_fired,pyro2_fired,"
  "gs_lat,gs_lon,gs_fix,range_m";

// ================================================================
// Buzzer (non-blocking)
// ================================================================
static void beep(unsigned long ms) {
  digitalWrite(PIN_BUZZER, HIGH);
  buzzerOffAt = millis() + ms;
}
static void serviceBuzzer() {
  if (buzzerOffAt && (long)(millis() - buzzerOffAt) >= 0) {
    digitalWrite(PIN_BUZZER, LOW);
    buzzerOffAt = 0;
  }
}

// ================================================================
// Time — GPS first, PC SETTIME:<epoch> as fallback / override
// ================================================================
static void applyTimeSync(time_t epochUtc, const char* source) {
  setenv("TZ", GS_TIMEZONE, 1);
  tzset();
  baseRealTime = epochUtc;
  syncMillis   = millis();
  timeSource   = source;

  char buf[32];
  struct tm* ti = localtime(&baseRealTime);
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", ti);
  Serial.printf("[TIME] Synced from %s. Local time: %s (%s)\n", source, buf, GS_TIMEZONE);
}

static void getRealDateTimeString(char* buffer, size_t size, unsigned long currentMillis) {
  if (baseRealTime > 0) {
    time_t t = baseRealTime + (currentMillis - syncMillis) / 1000;
    strftime(buffer, size, "%Y-%m-%d %H:%M:%S", localtime(&t));
  } else {
    unsigned long s = currentMillis / 1000;
    snprintf(buffer, size, "UNSYNCED+%02lu:%02lu:%02lu", (s / 3600) % 100, (s / 60) % 60, s % 60);
  }
}

static void handleSerialTimeSync() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialLineLen > 0) {
        serialLineBuf[serialLineLen] = '\0';
        if (strncmp(serialLineBuf, "SETTIME:", 8) == 0) {
          time_t epoch = (time_t)strtoul(serialLineBuf + 8, nullptr, 10);
          if (epoch > 0) applyTimeSync(epoch, "PC");
          else Serial.println("[TIME] Ignored malformed SETTIME line.");
        }
        serialLineLen = 0;
      }
    } else if (serialLineLen < sizeof(serialLineBuf) - 1) {
      serialLineBuf[serialLineLen++] = c;
    }
  }
}

static void maybeRequestTimeSync() {
  if (baseRealTime > 0) return;
  unsigned long now = millis();
  if (now - lastTimeReqMs < TIME_REQ_INTERVAL_MS) return;
  lastTimeReqMs = now;
  Serial.println("REQ_TIME");
}

// GPS time is UTC: convert with TZ temporarily set to UTC.
static void maybeSyncTimeFromGps() {
  if (gpsTimeSynced) return;
  if (!gps.date.isValid() || !gps.time.isValid() || gps.date.year() < 2025) return;
  if (gps.time.age() > 1500) return;

  struct tm t = {};
  t.tm_year = gps.date.year() - 1900;
  t.tm_mon  = gps.date.month() - 1;
  t.tm_mday = gps.date.day();
  t.tm_hour = gps.time.hour();
  t.tm_min  = gps.time.minute();
  t.tm_sec  = gps.time.second();

  setenv("TZ", "UTC0", 1);
  tzset();
  time_t epochUtc = mktime(&t);
  applyTimeSync(epochUtc, "GPS");    // restores GS_TIMEZONE
  gpsTimeSynced = true;
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

// Great-circle distance GS -> rocket (m). -1 if either has no fix.
static float rangeToRocketM() {
  if (!gsHasFix() || !havePacket || !lastPkt.gps_fix) return -1.0f;
  return (float)TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(),
                                             lastPkt.gps_lat, lastPkt.gps_lon);
}

// ================================================================
// SD logging — onboard slot or external module, on HSPI
// ================================================================
static bool gsAttemptSDMount() {
#if SD_USE_EXT_MODULE
  const int misoPin = PIN_SD_MISO_EXT, csPin = PIN_SD_CS_EXT;
  const char* srcName = "external module (J3)";
#else
  const int misoPin = PIN_SD_MISO_SLOT, csPin = PIN_SD_CS_SLOT;
  const char* srcName = "onboard slot (J2)";
#endif
  sdSPI.end();
  sdSPI.begin(PIN_SD_SCK, misoPin, PIN_SD_MOSI, csPin);

  if (!SD.begin(csPin, sdSPI, SD_SPI_FREQ_HZ)) {
    Serial.printf("[SD] SD.begin() failed on %s — continuing without logging.\n", srcName);
    return false;
  }
  if (SD.cardType() == CARD_NONE) {
    Serial.println("[SD] Card type NONE — continuing without logging.");
    return false;
  }

  for (int i = 0; i < 1000; i++) {
    snprintf(gsLogPath, sizeof(gsLogPath), "/VESTA%03d.CSV", i);
    if (!SD.exists(gsLogPath)) break;
  }

  gsLogFile = SD.open(gsLogPath, FILE_WRITE);
  if (!gsLogFile) {
    Serial.printf("[SD] Failed to create %s\n", gsLogPath);
    return false;
  }
  gsLogFile.println(GS_CSV_HEADER);
  gsLogFile.flush();
  Serial.printf("[SD] Logging to %s on %s\n", gsLogPath, srcName);
  return true;
}

static void gsMaintainSDConnection() {
  if (gsSdReady) return;
  unsigned long now = millis();
  if (now - gsLastMountAttemptMs < SD_MOUNT_RETRY_MS) return;
  gsLastMountAttemptMs = now;
  Serial.println("[SD] Retry mount attempt...");
  gsSdReady = gsAttemptSDMount();
}

static void gsLogPacket(const TelemetryPacket& pkt, unsigned long recvMillis, int rssiDbm, float snrDb) {
  if (!gsSdReady || !gsLogFile) return;

  char dateTimeBuf[32];
  getRealDateTimeString(dateTimeBuf, sizeof(dateTimeBuf), recvMillis);

  size_t written = gsLogFile.printf(
    "%s,%lu,%d,%.1f,%lu,%lu,%u,"
    "%.1f,%.2f,%.2f,%.2f,%d,"
    "%.2f,%.2f,"
    "%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%d,"
    "%.6f,%.6f,%.2f,%u,%d,"
    "%.2f,%d,"
    "%d,%d,%d,%d,"
    "%.6f,%.6f,%d,%.1f\n",
    dateTimeBuf, recvMillis, rssiDbm, snrDb,
    (unsigned long)pkt.seq, (unsigned long)pkt.timestamp_ms, pkt.state,
    pkt.pressure_pa, pkt.baro_alt_m, pkt.agl_m, pkt.baro_temp_c, pkt.baro_valid,
    pkt.sht_temp_c, pkt.humidity_pct,
    pkt.accel_x_g, pkt.accel_y_g, pkt.accel_z_g, pkt.accel_total_g,
    pkt.gyro_x_dps, pkt.gyro_y_dps, pkt.gyro_z_dps, pkt.mpu_temp_c, pkt.imu_valid,
    pkt.gps_lat, pkt.gps_lon, pkt.gps_alt_m, pkt.gps_sats, pkt.gps_fix,
    pkt.battery_v, pkt.sd_logging,
    pkt.pyro1_continuity, pkt.pyro2_continuity, pkt.pyro1_fired, pkt.pyro2_fired,
    gsHasFix() ? gps.location.lat() : 0.0, gsHasFix() ? gps.location.lng() : 0.0,
    gsHasFix() ? 1 : 0, rangeToRocketM()
  );
  gsLogFile.flush();

  if (written == 0) {               // card pulled or failed: drop to retry mode
    Serial.println("[SD] Write failed — will retry mount.");
    gsLogFile.close();
    SD.end();
    gsSdReady = false;
  }
}

// ================================================================
// Status LEDs
//   LED1 = SD    green logging · red off
//   LED2 = LINK  blue flash on packet · green live · red stale · amber waiting
//   LED3 = GPS   green fix · amber searching
// ================================================================
static void updateLeds() {
  unsigned long now = millis();
  if (now - lastLedMs < 100) return;
  lastLedMs = now;

  const uint32_t RED   = leds.Color(255, 0, 0);
  const uint32_t GREEN = leds.Color(0, 255, 0);
  const uint32_t BLUE  = leds.Color(0, 0, 255);
  const uint32_t AMBER = leds.Color(255, 120, 0);

  leds.setPixelColor(0, gsSdReady ? GREEN : RED);

  uint32_t linkColor;
  if (!havePacket)                          linkColor = AMBER;
  else if (now - lastPacketMs < 100)        linkColor = BLUE;
  else if (now - lastPacketMs < LINK_STALE_MS) linkColor = GREEN;
  else                                      linkColor = RED;
  leds.setPixelColor(1, linkColor);

  leds.setPixelColor(2, gsHasFix() ? GREEN : AMBER);
  leds.show();
}

// ================================================================
// Setup
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("=== VESTA v1 Ground Station (preliminary, untested) ===");

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  leds.begin();
  leds.setBrightness(LED_BRIGHTNESS);
  leds.clear();
  leds.show();

  setenv("TZ", GS_TIMEZONE, 1);
  tzset();

  // Link profile from the MODE_SEL jumper (read once at boot)
  pinMode(PIN_MODE_SEL, INPUT);   // GPIO36 has no internal pull-up: R13 on board
  linkCfg = (digitalRead(PIN_MODE_SEL) == LOW) ? PROFILE_CANSAT : PROFILE_ROCKET;
  Serial.printf("[MODE] MODE_SEL -> %s profile\n", linkCfg.name);

  // LoRa on VSPI (global SPI object, used by RadioHead)
  SPI.begin(PIN_RFM95_SCK, PIN_RFM95_MISO, PIN_RFM95_MOSI, PIN_RFM95_CS);

  pinMode(PIN_RFM95_RST, OUTPUT);
  digitalWrite(PIN_RFM95_RST, HIGH); delay(10);
  digitalWrite(PIN_RFM95_RST, LOW);  delay(10);
  digitalWrite(PIN_RFM95_RST, HIGH); delay(10);

  if (!rf95.init()) {
    Serial.println("[LORA] rf95.init() FAILED. Check RFM95W wiring/pins.");
    leds.fill(leds.Color(255, 0, 0));
    leds.show();
    beep(1000);
    while (true) { serviceBuzzer(); delay(10); }
  }
  rf95.setFrequency(linkCfg.freqMhz);
  rf95.setSpreadingFactor(linkCfg.sf);
  rf95.setSignalBandwidth(linkCfg.bwHz);
  rf95.setCodingRate4(linkCfg.crDenom);
  rf95.spiWrite(0x39, linkCfg.syncWord);   // RegSyncWord, written directly (same as FC)

  Serial.printf("[LORA] Listening at %.1f MHz, SF%d, BW%ldkHz, sync 0x%02X\n",
                linkCfg.freqMhz, linkCfg.sf, linkCfg.bwHz / 1000, linkCfg.syncWord);

  // microSD on HSPI
  gsSdReady = gsAttemptSDMount();
  gsLastMountAttemptMs = millis();

  // GPS on UART2
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  Serial.println("[GPS] UART2 started, waiting for fix...");

  Serial.println("[TIME] Waiting for GPS time or PC sync (send \"SETTIME:<unix_epoch>\\n\")...");
  lastTimeReqMs = millis();

  beep(150);
  Serial.println("=== Ground station ready ===");
}

// ================================================================
// Dashboard
// ================================================================
static void printDashboard() {
  unsigned long now = millis();

  Serial.println();
  Serial.println(F("============================================================"));

  if (!havePacket) {
    Serial.printf(" VESTA v1 GS [%s] — waiting for first packet...\n", linkCfg.name);
    Serial.printf(" TIME: %s   GS GPS: %s (%u sats)\n",
                  baseRealTime > 0 ? timeSource : "NOT SYNCED",
                  gsHasFix() ? "FIX" : "no fix", (unsigned)gps.satellites.value());
    Serial.println(F("============================================================"));
    return;
  }

  unsigned long sinceMs = now - lastPacketMs;
  uint32_t totalExpected = packetsReceived + packetsLost;
  float lossPct = (totalExpected > 0) ? (100.0f * packetsLost / totalExpected) : 0.0f;

  Serial.printf(" VESTA v1 [%s] | STATE: %-16s | seq #%-6lu | last RX %4lums ago\n",
                linkCfg.name, flightStateName(lastPkt.state), (unsigned long)lastPkt.seq, sinceMs);
  Serial.println(F("------------------------------------------------------------"));

  Serial.printf(" LINK    RSSI: %5d dBm   SNR: %6.1f dB   RX:%-6lu  lost:%-5lu (%.1f%%)%s\n",
                lastRssiDbm, lastSnrDb, (unsigned long)packetsReceived, (unsigned long)packetsLost, lossPct,
                (sinceMs > LINK_STALE_MS) ? "   [STALE]" : "");

  Serial.printf(" BARO    AGL: %8.2f m   ABS: %8.2f m   T: %6.2f C   [%s]\n",
                lastPkt.agl_m, lastPkt.baro_alt_m, lastPkt.baro_temp_c,
                lastPkt.baro_valid ? "OK" : "STALE");

  Serial.printf(" IMU     accel X:%6.2f Y:%6.2f Z:%6.2f |%6.2f|g  gyro X:%7.1f Y:%7.1f Z:%7.1f dps  [%s]\n",
                lastPkt.accel_x_g, lastPkt.accel_y_g, lastPkt.accel_z_g, lastPkt.accel_total_g,
                lastPkt.gyro_x_dps, lastPkt.gyro_y_dps, lastPkt.gyro_z_dps,
                lastPkt.imu_valid ? "OK" : "STALE");

  if (lastPkt.gps_fix) {
    Serial.printf(" FC GPS  FIX  sats:%2u  lat:%10.6f  lon:%11.6f  alt:%8.2f m\n",
                  lastPkt.gps_sats, lastPkt.gps_lat, lastPkt.gps_lon, lastPkt.gps_alt_m);
  } else {
    Serial.printf(" FC GPS  NO FIX  sats:%2u\n", lastPkt.gps_sats);
  }

  if (gsHasFix()) {
    float range = rangeToRocketM();
    Serial.printf(" GS GPS  FIX  sats:%2u  lat:%10.6f  lon:%11.6f  range: %s",
                  (unsigned)gps.satellites.value(), gps.location.lat(), gps.location.lng(),
                  range >= 0 ? "" : "n/a\n");
    if (range >= 0) Serial.printf("%.0f m\n", range);
  } else {
    Serial.printf(" GS GPS  NO FIX  sats:%2u\n", (unsigned)gps.satellites.value());
  }

  Serial.printf(" PWR     FC battery: %5.2f V\n", lastPkt.battery_v);

  Serial.printf(" SD      FC:%-12s  GS:%s   TIME: %s\n",
                lastPkt.sd_logging ? "LOGGING" : "NOT logging",
                gsSdReady ? gsLogPath : "OFF",
                baseRealTime > 0 ? timeSource : "NOT SYNCED");

  Serial.printf(" PYRO    CH1 cont:%-4s fired:%-3s   |   CH2 cont:%-4s fired:%-3s\n",
                lastPkt.pyro1_continuity ? "OK" : "OPEN", lastPkt.pyro1_fired ? "YES" : "no",
                lastPkt.pyro2_continuity ? "OK" : "OPEN", lastPkt.pyro2_fired ? "YES" : "no");

  Serial.println(F("============================================================"));
}

// ================================================================
// Loop
// ================================================================
void loop() {
  handleSerialTimeSync();
  maybeRequestTimeSync();
  readGps();
  maybeSyncTimeFromGps();

  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      lastRssiDbm = (int)rf95.spiRead(0x1A) - 157;     // RegRssiValue
      lastSnrDb   = (int8_t)rf95.spiRead(0x19) / 4.0f; // RegPktSnrValue

      if (len == sizeof(TelemetryPacket)) {
        TelemetryPacket pkt;
        memcpy(&pkt, buf, sizeof(pkt));

        // Seq-gap loss estimate (a backward jump = FC reboot, not loss)
        if (lastSeq >= 0 && (int32_t)pkt.seq > lastSeq) {
          int32_t gap = (int32_t)pkt.seq - lastSeq - 1;
          if (gap > 0) packetsLost += gap;
        }
        lastSeq = (int32_t)pkt.seq;

        if (!havePacket || linkWasStale) beep(60);   // first packet / link recovered
        linkWasStale = false;

        lastPkt = pkt;
        havePacket = true;
        lastPacketMs = millis();
        packetsReceived++;

        gsLogPacket(pkt, lastPacketMs, lastRssiDbm, lastSnrDb);
      } else {
        Serial.printf("[LORA] Packet size mismatch: got %u bytes, expected %u "
                      "(RSSI %d dBm). Check TelemetryPacket on both sides.\n",
                      len, (unsigned)sizeof(TelemetryPacket), lastRssiDbm);
      }
    }
  }

  // Link-lost alert (once per loss)
  if (havePacket && !linkWasStale && millis() - lastPacketMs > LINK_STALE_MS) {
    linkWasStale = true;
    beep(300);
  }

  gsMaintainSDConnection();
  updateLeds();
  serviceBuzzer();

  unsigned long now = millis();
  if (now - lastDashboardMs >= DASHBOARD_INTERVAL_MS) {
    lastDashboardMs = now;
    printDashboard();
  }
}
